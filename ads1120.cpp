#include "ads1120.h"
#include "inttypes.h"

ADS1120::ADS1120(int cs_pin, int clk, int mosi, int miso) : spibus(clk, mosi, miso){
    this->cs_pin = cs_pin;
    pinMode(this->cs_pin, OUTPUT);
    digitalWrite(this->cs_pin, HIGH);
}

void ADS1120::init(uint8_t* configs){
    spibus.begin();

    this->reset();
    delayMicroseconds(100);
    uint8_t cmd = ADS1120_CMD_WREG | 3; // starts at register 0
    spibus.beginTransaction(1000000, SWSPI_MSBFIRST, 1);
    digitalWrite(this->cs_pin, LOW);
    PAUSE_SHORT;
    spibus.transfer(&cmd, 1);
    spibus.transfer(configs, 4);
    digitalWrite(this->cs_pin, HIGH);
    spibus.endTransaction();

}

void ADS1120::reset(){
    uint8_t cmd = ADS1120_CMD_RESET;
    spibus.beginTransaction(1000000, SWSPI_MSBFIRST, 1);
    digitalWrite(this->cs_pin, LOW);
    PAUSE_SHORT;
    spibus.transfer(&cmd, 1);
    digitalWrite(this->cs_pin, HIGH);
    spibus.endTransaction();
    delayMicroseconds(100); 
}

void ADS1120::start_sync(){
    uint8_t cmd = ADS1120_CMD_START_SYNC;
    spibus.beginTransaction(1000000, SWSPI_MSBFIRST, 1);
    digitalWrite(this->cs_pin, LOW);
    PAUSE_SHORT;
    spibus.transfer(&cmd, 1);
    digitalWrite(this->cs_pin, HIGH);
    spibus.endTransaction();
}

int16_t ADS1120::get_conv(){
    uint8_t buf[3];
    buf[0] = ADS1120_CMD_RDATA;
    buf[1] = 0;
    buf[2] = 0;
    spibus.beginTransaction(1000000, SWSPI_MSBFIRST, 1);
    digitalWrite(this->cs_pin, LOW);
    PAUSE_SHORT;
    spibus.transfer(buf, 3);
    digitalWrite(this->cs_pin, HIGH);
    spibus.endTransaction();

    return (int16_t)((buf[1] << 8) | buf[2]);

}

/**
 * @brief set_input_mux changes the input mux configuration without changing anything else
 */
void ADS1120::set_input_mux(uint8_t config){
    uint8_t cmd = ADS1120_CMD_RREG; // register 0 (rr = 0), 1 bytes (nn = 0)
    uint8_t reg = 0;

    spibus.beginTransaction(1000000, SWSPI_MSBFIRST, 1);

    digitalWrite(this->cs_pin, LOW);
    PAUSE_SHORT;
    spibus.transfer(&cmd, 1);
    spibus.transfer(&reg, 1);
    digitalWrite(this->cs_pin, HIGH);
    PAUSE_SHORT;

    
    reg &= 0x0F; // clear mux bits [7:4]
    reg |= config;
    cmd = ADS1120_CMD_WREG; // rr = 0, nn = 0

    digitalWrite(this->cs_pin, LOW);
    PAUSE_SHORT;
    spibus.transfer(&cmd, 1);
    spibus.transfer(&reg, 1);
    digitalWrite(this->cs_pin, HIGH);

    spibus.endTransaction();
}