#include <SPI.h>
#include "ads1120.h"
#include "printf.h"
#include "inttypes.h"

/* PINS */
#define VC1_BAL   A5
#define VC2_BAL   3
#define VC3_BAL   4
#define VC4_BAL   A6
#define VC5_BAL   8
#define VC6_BAL   9

#define ADC_CS    10

#define PWM_SINK  5
#define PWM_SRC   6

#define RELAY     7

#define ZERO_CAP  2
#define EN_MUX    A0
#define MUX_A0    A1
#define MUX_A1    A2
#define MUX_A2    A3
#define READ      A4

#define LED_PIN   A7

#define SW_CLK  13
#define SW_MOSI 12
#define SW_MISO 11

/* relay parameters */
#define RELAY_BREAK_TIME  15
#define RELAY_MAKE_TIME   15


/* limits */
#define MAX_CHG_CURRENT         1.0
#define MAX_DISCHG_CURRENT      1.0
#define MAX_MOSFET_PWR          5.0
#define LI_ION_MAX_CELL_VOLTAGE 4.2
#define ALLOWABLE_CELL_MARGIN   0.01
#define MAX_NUM_CELLS           6


/* various parameters */
#define LI_ION_STORAGE_VOLTAGE  3.8
#define CELL_SAMPLES            4
#define CAP_DISCHARGE_DELAY_US  250
#define CAP_CHARGE_DELAY_US     1000
#define ADC_SETTLING_TIME_US    10000
#define ADC_SAMPLE_PERIOD       3030
#define ADC_FS                  4.096
#define ADC_COUNTS_FS           32767.0
#define HI_V_SNS_GAIN           (2.0/12.0)
#define SCRATCH_BUF_LEN         256


/* charge modes */
#define MODE_CHECK_BATTERY    0
#define MODE_CHARGE_INIT      1
#define MODE_CHARGE           2
#define MODE_DISCHARGE_INIT   3  
#define MODE_DISCHARGE        4
#define MODE_NO_POWER         5
#define MODE_NO_BATT          6
#define MODE_BATT_ERR         7
#define MODE_IDLE             8

#define LED_BLINK_DUR_MS 750

#define LED_OFF   0
#define LED_SOLID 1
#define LED_BLINK 2

/* MODE_IDLE loop delay in milliseconds */
#define IDLE_LOOP_DELAY       500
#define CHECK_BATT_DELAY      100
#define NO_BATT_LOOP_DELAY    100
#define BATT_ERR_LOOP_DELAY   100
#define CHARGE_LOOP_DELAY     250
#define DISCHARGE_LOOP_DELAY  250

////////////////////////////////////////////////////////////////////////////////
//
//  FUNCTION PROTOTYPES
//
////////////////////////////////////////////////////////////////////////////////
float zero_offset(uint8_t cell);
void balance_cells();
void goto_state(uint8_t next_state);
void set_cell_shunt(uint8_t cell, uint8_t state);
void set_mux(uint8_t cell);
void set_charge_current(float amps);
void set_discharge_current(float amps);
void disable_cell_shunt(uint8_t cell);
void enable_cell_shunt(uint8_t cell);
float get_cell_voltage(uint8_t cell);
float get_pwr_node_voltage();
float get_batt_voltage();
int find_min(float* values, int size);
// float min(float a, float b);
void debug(uint8_t* str);

////////////////////////////////////////////////////////////////////////////////
//
//  GLOBAL VARIABLES
//
////////////////////////////////////////////////////////////////////////////////
float seconds = 0.0;
uint8_t scratch_buf[SCRATCH_BUF_LEN];
uint8_t debug_buf[SCRATCH_BUF_LEN];
uint8_t ads1120_config[4] = {};
float vin = 0.0;
float cell_offsets[6] = {};
float cells[6] = {}; // cell voltages
ADS1120 ads(ADC_CS, SW_CLK, SW_MOSI, SW_MISO);
uint8_t n_cells = MAX_NUM_CELLS; // start by assuming the most cells
bool allow_charging = true;
float pack_voltage = 0.0;
float ideal_voltage = 0.0;
float batt_voltage = 0.0;
float pwr_node_voltage = 0.0;
bool pwr_conn_ok = false;
uint8_t mode = MODE_IDLE;
float chg_current = 0.0;
float max_chg_current = MAX_CHG_CURRENT;
float max_dischg_current = MAX_DISCHG_CURRENT;
float dischg_current = 0.0;
uint8_t led_mode = LED_OFF; 
long long chg_dischg_start = 0;

////////////////////////////////////////////////////////////////////////////////
//
//  SETUP
//
////////////////////////////////////////////////////////////////////////////////
void setup() {
  analogReference(EXTERNAL);
  pinMode(VC1_BAL, OUTPUT);
  pinMode(VC2_BAL, OUTPUT);
  pinMode(VC3_BAL, OUTPUT);
  pinMode(VC4_BAL, OUTPUT);
  pinMode(VC5_BAL, OUTPUT);
  pinMode(VC6_BAL, OUTPUT);
  pinMode(ADC_CS, OUTPUT);
  pinMode(PWM_SINK, OUTPUT);
  pinMode(PWM_SRC, OUTPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(ZERO_CAP, OUTPUT);
  pinMode(EN_MUX, OUTPUT);
  pinMode(MUX_A0, OUTPUT);
  pinMode(MUX_A1, OUTPUT);
  pinMode(MUX_A2, OUTPUT);
  pinMode(READ, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  digitalWrite(VC1_BAL, LOW);
  digitalWrite(VC2_BAL, LOW);
  digitalWrite(VC3_BAL, LOW);
  digitalWrite(VC4_BAL, LOW);
  digitalWrite(VC5_BAL, LOW);
  digitalWrite(VC6_BAL, LOW);
  digitalWrite(ADC_CS, HIGH);
  analogWrite(PWM_SINK, 0);
  analogWrite(PWM_SRC, 0);
  digitalWrite(RELAY, LOW);
  digitalWrite(ZERO_CAP, LOW);
  digitalWrite(EN_MUX, LOW);
  digitalWrite(MUX_A0, LOW);
  digitalWrite(MUX_A1, LOW);
  digitalWrite(MUX_A2, LOW);
  digitalWrite(READ, LOW);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);

  debug("--- Li-Ion/LiPo battery storage charger ---");
  delay(1000);

  ads1120_config[0] = ADS1120_MUX_P0_N1 | ADS1120_PGA_GAIN_1 | ADS1120_PGA_BYPASS;
  ads1120_config[1] = ADS1120_MODE_NORMAL | ADS1120_DR_4 | ADS1120_CM_CONT;
  ads1120_config[2] = ADS1120_VREF_EXT;
  ads1120_config[3] = ADS1120_DRDYM_BOTH;

  ads.init(ads1120_config);
  ads.start_sync();

  ads.set_input_mux(ADS1120_MUX_P3_NVSS);
  float weight = 0.1;
  float voltage = 0.0;
  for(int i = 0; i < 10; i++){
    float conv = (float)ads.get_conv();
    voltage += weight * ADC_FS * (conv / ADC_COUNTS_FS);
  }
  vin = voltage * 6.0;
  snprintf(debug_buf, SCRATCH_BUF_LEN, "Vin = %.3f", vin);
  debug(debug_buf);
}

////////////////////////////////////////////////////////////////////////////////
//
//  MAIN LOOP
//
////////////////////////////////////////////////////////////////////////////////
void loop() {

  switch(mode){
    case MODE_CHECK_BATTERY:{
      set_charge_current(0.0);
      set_discharge_current(0.0);

      // turn off all cell balancing shunts
      for(uint8_t cell = 0; cell < MAX_NUM_CELLS; cell++){
        disable_cell_shunt(cell + 1);
      }

      delay(CHECK_BATT_DELAY);

      // read all cell positions
      int i = 0; // list index
      for(i = 0; i < MAX_NUM_CELLS; i++){
        float total = get_cell_voltage(i+1);
        delay(10);
        total += get_cell_voltage(i+1);
        delay(10);
        total += get_cell_voltage(i+1);
        cells[i] = total/3;
      }
      snprintf(debug_buf, SCRATCH_BUF_LEN, "CELLS: %.3f,\t%.3f,\t%.3f,\t%.3f,\t%.3f,\t%.3f", cells[0], cells[1], cells[2], cells[3], cells[4], cells[5]);
      debug(debug_buf);

      // find out how many cells we actually have
      n_cells = 0;
      i = 0;
      bool end_reached = false;
      while(!end_reached){
        if(i >= 6 || cells[i] < 1.0){
          // unlikely that there is actually a cell attached here
          end_reached = true;
        }else{
          // else it's likely that there is a cell here
          // so increment our counter to reflect this
          n_cells++;
        }
        i++;
      }

      // calculate pack voltage from individual cell voltages
      pack_voltage = 0.0;
      ideal_voltage = LI_ION_STORAGE_VOLTAGE * n_cells;
      float margin = ALLOWABLE_CELL_MARGIN * n_cells;
      for(i = 0; i < n_cells; i++){
        pack_voltage += cells[i];
      }

      batt_voltage = get_batt_voltage(); // see what the highest voltage in the pack is

      // makes sure the battery is connected to the power terminals 
      pwr_node_voltage = get_pwr_node_voltage();
      if(pwr_node_voltage <= (batt_voltage - 1.0) || pwr_node_voltage <= pack_voltage - 1.0){
        // battery is likely not connected to power terminals yet
      }

      // snprintf(debug_buf, SCRATCH_BUF_LEN, "Cells = %d;\tBatt_Voltage = %.3fV;\tPack Voltage = %.3fV;\tPwr Node V = %.2fV", n_cells, batt_voltage, pack_voltage, pwr_node_voltage);
      // debug(debug_buf);

      if(n_cells == 0){
        // if no cells detected, we don't care about anything else
        mode = MODE_NO_BATT;
      }else if(pack_voltage < (batt_voltage - 1.5)){
        // if sum of all detected cells is significantly less than highest voltage in system, we have a problem
        // someone probably goofed the battery connections
        mode = MODE_BATT_ERR;
      }else if(pack_voltage < (ideal_voltage - margin)){
        mode = MODE_CHARGE_INIT;    // need to charge up some
      }else if(pack_voltage > (ideal_voltage + margin)){
        mode = MODE_DISCHARGE_INIT; // need to discharge some
      }else if(cells[find_min(cells, n_cells)] < (LI_ION_STORAGE_VOLTAGE - ALLOWABLE_CELL_MARGIN)){
        mode = MODE_CHARGE_INIT;    // at least one cell is too low, need to charge
      }else{
        mode = MODE_IDLE;      // no need to charge or discharge
      }

      break;
    }case MODE_IDLE:{
      set_discharge_current(0.0);
      set_charge_current(0.0);
      digitalWrite(RELAY, LOW); // open the relay
      delay(RELAY_BREAK_TIME);
      led_mode = LED_SOLID;
      delay(IDLE_LOOP_DELAY);
      mode = MODE_CHECK_BATTERY;
      break;

    }case MODE_BATT_ERR:{
      debug("Battery error...");
      led_mode = LED_OFF;
      delay(BATT_ERR_LOOP_DELAY);
      mode = MODE_CHECK_BATTERY;
      break;
    }case MODE_NO_BATT:{
      debug("No battery detected...");
      led_mode = LED_OFF;
      digitalWrite(RELAY, LOW); // open the relay
      delay(RELAY_BREAK_TIME);
      delay(NO_BATT_LOOP_DELAY);
      mode = MODE_CHECK_BATTERY;
      break;

    }case MODE_CHARGE_INIT:{
      set_discharge_current(0.0);
      digitalWrite(RELAY, HIGH); // close the relay
      delay(RELAY_MAKE_TIME); // wait for it to actuate
      delay(20); // additional settling time
      led_mode = LED_BLINK;
      mode = MODE_CHARGE;
      break;

    }case MODE_CHARGE:{
      // see how much current we can draw for our rated power, or just the absolute max we set above
      max_chg_current = min(MAX_CHG_CURRENT, MAX_MOSFET_PWR/(24 - pack_voltage));
      
      // calculate charge current based on error
      if((chg_current = (ideal_voltage - pack_voltage)) > max_chg_current){
        chg_current = max_chg_current;
      }
      if(allow_charging){
        snprintf(debug_buf, SCRATCH_BUF_LEN, "Charging at %.3fA", chg_current);
        debug(debug_buf);
  
        set_charge_current(chg_current);
      }else{
        debug("Charging not allowed.");
        set_charge_current(0.0);
      }

      balance_cells();

      delay(CHARGE_LOOP_DELAY);

      mode = MODE_CHECK_BATTERY;

      break;
    }case MODE_DISCHARGE_INIT:{
      set_charge_current(0.0);
      digitalWrite(RELAY, HIGH); // close the relay
      delay(RELAY_MAKE_TIME); // wait for it to actuate
      led_mode = LED_BLINK;
      mode = MODE_DISCHARGE;
      break;

    }case MODE_DISCHARGE:{
      // see how much current we can sink for our rated power, or just the absolute max we set above
      max_dischg_current = min(MAX_DISCHG_CURRENT, MAX_MOSFET_PWR/pack_voltage);
      
      // calculate discharge current
      if((dischg_current = (ideal_voltage - pack_voltage)) > max_dischg_current){
        dischg_current = max_dischg_current;
      }
      
      snprintf(debug_buf, SCRATCH_BUF_LEN, "Discharging at %.3fA", chg_current);
      debug(debug_buf);
      set_discharge_current(dischg_current);

      balance_cells();

      delay(DISCHARGE_LOOP_DELAY);

      mode = MODE_CHECK_BATTERY;
      
      break;
    }default:{
      mode = MODE_CHECK_BATTERY;
      break;
    }
  }
  

  switch(led_mode){
    case LED_OFF:
      digitalWrite(LED_PIN, LOW);
      break;
    case LED_SOLID:
      digitalWrite(LED_PIN, HIGH);
      break;
    case LED_BLINK:
      if(((millis()/LED_BLINK_DUR_MS) % 2) == 0){
        digitalWrite(LED_PIN, HIGH);
      }else{
        digitalWrite(LED_PIN, LOW);
      }
      break;
    default:
      led_mode = LED_OFF;
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////
//
//  FUNCTION DEFINITIONS
//
////////////////////////////////////////////////////////////////////////////////

/**
 * 
 */
void balance_cells(){
  // we only have the capability to drop the cell voltages, so find which cells 
  // are too high and enable the shunt network
  for(uint8_t i = 0; i < n_cells; i++){
    if(cells[i] > LI_ION_STORAGE_VOLTAGE + ALLOWABLE_CELL_MARGIN){
      enable_cell_shunt(i+1);
      allow_charging |= cells[i] < LI_ION_MAX_CELL_VOLTAGE;
    }else{
      disable_cell_shunt(i+1);
    }
  }
}


void goto_state(uint8_t next_state){
  switch(next_state){
    case MODE_CHECK_BATTERY:{
      set_discharge_current(0.0);
      set_charge_current(0.0);
      break;
    }
    case MODE_NO_BATT:{
      led_mode = LED_OFF;
      set_discharge_current(0.0);
      set_charge_current(0.0);
      digitalWrite(RELAY, LOW); // open the relay
      delay(RELAY_BREAK_TIME);
      break;
    }case MODE_IDLE:{
      led_mode = LED_SOLID;
      set_discharge_current(0.0);
      set_charge_current(0.0);
      digitalWrite(RELAY, LOW); // open the relay
      delay(RELAY_BREAK_TIME);
      break;
    }case MODE_CHARGE:{
      led_mode = LED_BLINK;
      set_discharge_current(0.0);
      digitalWrite(RELAY, HIGH); // close the relay
      delay(RELAY_MAKE_TIME); // wait for it to actuate
      delay(20); // additional settling time
      chg_dischg_start = millis();
      break;
    }case MODE_DISCHARGE:{
      led_mode = LED_BLINK;
      set_charge_current(0.0);
      digitalWrite(RELAY, HIGH); // close the relay
      delay(RELAY_MAKE_TIME); // wait for it to actuate
      delay(20); // additional settling time
      chg_dischg_start = millis();
      break;
    }default:{
      break;
    }
  }
  mode = next_state;
}

void disable_cell_shunt(uint8_t cell){
  set_cell_shunt(cell, LOW);
}

void enable_cell_shunt(uint8_t cell){
  set_cell_shunt(cell, HIGH);
}

void set_cell_shunt(uint8_t cell, uint8_t state){
  switch(cell){
    case 1:{
      digitalWrite(VC1_BAL, state);
      break;
    }case 2:{
      digitalWrite(VC2_BAL, state);
      break;
    }case 3:{
      digitalWrite(VC3_BAL, state);
      break;
    }case 4:{
      digitalWrite(VC4_BAL, state);
      break;
    }case 5:{
      digitalWrite(VC5_BAL, state);
      break;
    }case 6:{
      digitalWrite(VC6_BAL, state);
      break;
    }default:
      break;
  }
}

// sets the mux to the specified number
void set_mux(uint8_t cell){
  digitalWrite(MUX_A0, cell & 1);
  digitalWrite(MUX_A1, (cell >> 1) & 1);
  digitalWrite(MUX_A2, (cell >> 2) & 1);
}


void set_charge_current(float amps){
  int value = (int)(( (4.7*amps)/5.0 )*255.0 + 0.5);
  analogWrite(PWM_SRC, value);
}

void set_discharge_current(float amps){
  if(amps > 0.0){
    snprintf(debug_buf, SCRATCH_BUF_LEN, "Setting discharge current to %.3f A", amps);
    debug(debug_buf);
  }
  
  int value = (int)(( (4.4*amps)/5.0 )*255.0 + 0.5);
  analogWrite(PWM_SINK, value);
}

/**
 * @brief get_cell_voltage reads and returns the voltage of specified cell 
 */
float get_cell_voltage(uint8_t cell){
  if(cell > 6){
    return 0.0;
  }
  disable_cell_shunt(cell); // disable balancing on this cell

  // zero the flying capacitor
  digitalWrite(ZERO_CAP, HIGH);
  delayMicroseconds(CAP_DISCHARGE_DELAY_US);
  digitalWrite(ZERO_CAP, LOW);

  // disable mux
  digitalWrite(EN_MUX, LOW);
  delayMicroseconds(1);

  // set the mux as specified to charge the flying capacitor
  set_mux(cell);

  // enable mux
  digitalWrite(EN_MUX, HIGH);

  // wait for capacitor to be charged
  delayMicroseconds(CAP_CHARGE_DELAY_US);

  // disable mux
  digitalWrite(EN_MUX, LOW);
  // delayMicroseconds(1);

  // set the mux to 0
  set_mux(0);

  // re-enable the mux
  digitalWrite(EN_MUX, HIGH);

  // capacitor is now referenced to GND and has cell voltage across it
  // close the READ switches
  digitalWrite(READ, HIGH);

  // set the ADC mux
  ads.set_input_mux(ADS1120_MUX_P0_N1);

  // allow ADC to settle and acquire a good sample
  delayMicroseconds(ADC_SETTLING_TIME_US);

  // get conversion(s) from ADS1120
  float retval = 0.0;
  for(int i = 0; i < CELL_SAMPLES; i++){
    delayMicroseconds(ADC_SAMPLE_PERIOD);
    float tmp = ADC_FS * ((float)ads.get_conv()/ADC_COUNTS_FS);
    retval += tmp;
  }
  retval = retval / CELL_SAMPLES;

  // open the READ switches
  digitalWrite(READ, LOW);

  return retval - cell_offsets[cell-1];
}

/**
 * @brief read ADC and calculate voltage at the output of the relay
 */
float get_pwr_node_voltage(){
  ads.set_input_mux(ADS1120_MUX_P3_NVSS);

  uint32_t total = 0;
  for(int i = 0; i < 4; i++){
    delayMicroseconds(ADC_SAMPLE_PERIOD);
    total += ads.get_conv();
  }
  float retval = (float)(total/4);
  retval = ADC_FS * (retval/ADC_COUNTS_FS);
  return retval / HI_V_SNS_GAIN;
}

/**
 * @brief read ADC and calculate voltage of battery pack read from sense pins
 */
float get_batt_voltage(){
  ads.set_input_mux(ADS1120_MUX_P2_NVSS);

  float retval = 0.0;
  for(int i = 0; i < 4; i++){
    delayMicroseconds(ADC_SAMPLE_PERIOD);
    retval += 0.25 * ADC_FS * (float)ads.get_conv()/ADC_COUNTS_FS;
  }
  return retval / HI_V_SNS_GAIN;
}


int find_min(float* values, int size){
  int min_ind = 0;
  for(int i = 0; i < size; i++){
    if(values[i] < values[min_ind]){
      min_ind = i;
    }
  }
  return min_ind;
}

// float min(float a, float b){
//   if(a < b){
//     return a;
//   }else{
//     return b;
//   }
// }

/**
 * @brief print str along with current timestamp. Does a safety check to make sure we can write to the buffer
 */
void debug(uint8_t* str){
  seconds = millis()/1000.0;
  snprintf(scratch_buf, SCRATCH_BUF_LEN, "[%10.3f] %s\r\n", seconds, str);
  int len = strlen(scratch_buf);
  
  Serial.write(scratch_buf, len);
}
