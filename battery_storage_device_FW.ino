#include <SPI.h>
#include "ads1120.h"

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


/* relay parameters */
#define RELAY_BREAK_TIME  15
#define RELAY_MAKE_TIME   15


/* limits */
#define MAX_CHG_CURRENT         1.0
#define MAX_DISCHG_CURRENT      1.0
#define LI_ION_MAX_CELL_VOLTAGE 4.2
#define ALLOWABLE_CELL_MARGIN   0.01
#define MAX_NUM_CELLS           6


/* various parameters */
#define LI_ION_STORAGE_VOLTAGE  3.8
#define CELL_SAMPLES            4
#define CAP_DISCHARGE_DELAY_US  250
#define CAP_CHARGE_DELAY_US     250
#define ADC_SETTLING_TIME_US    10000
#define ADC_SAMPLE_PERIOD       1000
#define ADC_FS                  4.096
#define ADC_COUNTS_FS           65535.0


/* charge modes */
#define MODE_IDLE       0
#define MODE_CHARGE     1
#define MODE_DISCHARGE  2
#define MODE_NO_BATT    3

#define LED_BLINK_DUR_MS 750

#define LED_OFF   0
#define LED_SOLID 1
#define LED_BLINK 2

/* MODE_IDLE loop delay in milliseconds */
#define IDLE_LOOP_DELAY 5000

////////////////////////////////////////////////////////////////////////////////
//
//  FUNCTION PROTOTYPES
//
////////////////////////////////////////////////////////////////////////////////
void set_cell_shunt(uint8_t cell, uint8_t state);
void set_mux(uint8_t cell);
void set_charge_current(float amps);
void set_discharge_current(float amps);
void disable_cell_shunt(uint8_t cell);
void enable_cell_shunt(uint8_t cell);
float get_cell_voltage(uint8_t cell);
int find_min(float* values, int size);

////////////////////////////////////////////////////////////////////////////////
//
//  GLOBAL VARIABLES
//
////////////////////////////////////////////////////////////////////////////////
uint8_t ads1120_config[4] = {};
float vin = 0.0;
float cells[6] = {}; // cell voltages
ADS1120 ads(ADC_CS);
uint8_t n_cells = MAX_NUM_CELLS; // start by assuming the most cells
bool allow_charging = true;
float pack_voltage = 0.0;
uint8_t mode = MODE_IDLE;
float chg_current = 0.0;
float dischg_current = 0.0;
uint8_t led_mode = LED_OFF; 

////////////////////////////////////////////////////////////////////////////////
//
//  SETUP
//
////////////////////////////////////////////////////////////////////////////////
void setup() {
  // put your setup code here, to run once:
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

  SPI.begin();

  ads1120_config[0] = ADS1120_MUX_P0_N1 | ADS1120_PGA_GAIN_1 | ADS1120_PGA_BYPASS;
  ads1120_config[1] = ADS1120_MODE_NORMAL | ADS1120_DR_6 | ADS1120_CM_CONT;
  ads1120_config[2] = ADS1120_VREF_EXT;
  ads1120_config[3] = ADS1120_DRDYM_BOTH;

  ads.init(ads1120_config);

  ads.set_input_mux(ADS1120_MUX_P3_NVSS);
  float weight = 0.1;
  float voltage = 0.0;
  for(int i = 0; i < 10; i++){
    float conv = (float)ads.get_conv();
    voltage += weight * ADC_FS * (conv / ADC_COUNTS_FS);
  }
  vin = voltage * 6.0;
}

////////////////////////////////////////////////////////////////////////////////
//
//  MAIN LOOP
//
////////////////////////////////////////////////////////////////////////////////
void loop() {

  int i = 0; // list index
  // read all cell positions
  for(i = 0; i < MAX_NUM_CELLS; i++){
    cells[i] = get_cell_voltage(i+1);
  }

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

  // decide whether we need to charge or discharge the pack
  pack_voltage = 0.0;
  float ideal = LI_ION_STORAGE_VOLTAGE * n_cells;
  float margin = ALLOWABLE_CELL_MARGIN * n_cells;
  for(i = 0; i < n_cells; i++){
    pack_voltage += cells[i];
  }

  if(pack_voltage < (ideal - margin)){
    // need to charge up some
    mode = MODE_CHARGE;
    // calculate charge current based on error
    if((chg_current = (ideal - pack_voltage)) > MAX_CHG_CURRENT){
      chg_current = MAX_CHG_CURRENT;
    }
  }else if(pack_voltage > (ideal + margin)){
    // need to discharge some
    mode = MODE_DISCHARGE;
  }else{
    if(cells[find_min(cells, n_cells)] < (LI_ION_STORAGE_VOLTAGE - ALLOWABLE_CELL_MARGIN)){
      // at least one cell is too low, need to charge
      mode = MODE_CHARGE;
      chg_current = 0.1;
    }else{
      // no need to charge or discharge
      mode = MODE_IDLE;
    }
    
  }

  // do things based on charge mode
  switch(mode){
    case MODE_IDLE:{
      led_mode = LED_SOLID;
      set_discharge_current(0.0);
      set_charge_current(0.0);
      digitalWrite(RELAY, LOW); // open the relay
      delay(RELAY_BREAK_TIME);
      delay(IDLE_LOOP_DELAY);
      break;
    }case MODE_CHARGE:{
      led_mode = LED_BLINK;
      set_discharge_current(0.0);
      digitalWrite(RELAY, HIGH); // close the relay
      delay(RELAY_MAKE_TIME); // wait for it to actuate

      if(allow_charging){
        set_charge_current(chg_current);
      }else{
        set_charge_current(0.0);
      }
      break;
    }case MODE_DISCHARGE:{
      led_mode = LED_BLINK;
      set_charge_current(0.0);
      digitalWrite(RELAY, HIGH); // close the relay
      delay(RELAY_MAKE_TIME); // wait for it to actuate

      // calculate discharge current
      if((dischg_current = (ideal - pack_voltage)) > MAX_DISCHG_CURRENT){
        dischg_current = MAX_DISCHG_CURRENT;
      }
      set_discharge_current(dischg_current);
      break;
    }default:{
      mode = MODE_IDLE;
      break;
    }
  }

  // CELL BALANCING
  // do this regardless of the charge mode
  // we only have the capability to drop the cell voltages, so find which cells 
  // are too high and enable the shunt network
  for(i = 0; i < n_cells; i++){
    if(cells[i] > LI_ION_STORAGE_VOLTAGE + ALLOWABLE_CELL_MARGIN){
      enable_cell_shunt(i+1);
      allow_charging = cells[i] < LI_ION_MAX_CELL_VOLTAGE);
    }else{
      disable_cell_shunt(i+1);
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
      if(((millis()/LED_BLINK_DUR_MS) % 2) == 1){
        digitalWrite(LED_PIN, HIGH);
      }else{
        digitalWrite(LED_PIN, LOW);
      }
  }
}

////////////////////////////////////////////////////////////////////////////////
//
//  FUNCTION DEFINITIONS
//
////////////////////////////////////////////////////////////////////////////////
void disable_cell_shunt(uint8_t cell){
  set_cell_shunt(cell, HIGH);
}

void enable_cell_shunt(uint8_t cell){
  set_cell_shunt(cell, LOW);
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
  delayMicroseconds(1);

  // set the mux to 0
  set_mux(0);

  // re-enable the mux
  digitalWrite(EN_MUX, HIGH);

  // capacitor is now referenced to GND and has cell voltage across it
  // close the READ switches
  digitalWrite(READ, HIGH);

  // allow ADC to settle and acquire a good sample
  delayMicroseconds(ADC_SETTLING_TIME_US);

  // get conversion(s) from ADS1120
  uint32_t total = 0;
  uint16_t values[CELL_SAMPLES];
  for(int i = 0; i < CELL_SAMPLES; i++){
    values[i] = ads.get_conv();
    total += values[i];
    delayMicroseconds(ADC_SAMPLE_PERIOD);
  }
  float raw_value = (float)(total/CELL_SAMPLES);
  float retval = ADC_FS * (raw_value/ADC_COUNTS_FS);

  // open the READ switches
  digitalWrite(READ, LOW);

  return retval;
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