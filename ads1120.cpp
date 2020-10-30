#include "ads1120.h"

ADS1120::ADS1120(int cs_pin) : spi_settings(4000000, MSBFIRST, SPI_MODE1){
    this->cs_pin = cs_pin;
}

void ADS1120::init(uint8_t* configs){
    pinMode(this->cs_pin, OUTPUT);
    digitalWrite(this->cs_pin, HIGH);

    this->reset();

    uint8_t cmd = ADS1120_CMD_WREG | 3; // starts at register 0
    SPI.beginTransaction(this->spi_settings);
    digitalWrite(this->cs_pin, LOW);
    PAUSE_SHORT;
    SPI.transfer(&cmd, 1);
    SPI.transfer(configs, 4);
    digitalWrite(this->cs_pin, HIGH);
    SPI.endTransaction();

}

void ADS1120::reset(){
    uint8_t cmd = ADS1120_CMD_RESET;
    SPI.beginTransaction(this->spi_settings);
    digitalWrite(this->cs_pin, LOW);
    PAUSE_SHORT;
    SPI.transfer(&cmd, 1);
    digitalWrite(this->cs_pin, HIGH);
    SPI.endTransaction();
    delayMicroseconds(100); 
}

uint16_t ADS1120::get_conv(){
    uint8_t buf[3];
    buf[0] = ADS1120_CMD_RDATA;
    buf[1] = 0;
    buf[2] = 0;
    SPI.beginTransaction(this->spi_settings);
    digitalWrite(this->cs_pin, LOW);
    PAUSE_SHORT;
    SPI.transfer(buf, 3);
    digitalWrite(this->cs_pin, HIGH);
    SPI.endTransaction();

    return (uint16_t)((buf[1] << 8) | buf[2]);

}

/**
 * @brief set_input_mux changes the input mux configuration without changing anything else
 */
void ADS1120::set_input_mux(uint8_t config){
    uint8_t cmd = ADS1120_CMD_RREG; // register 0 (rr = 0), 1 bytes (nn = 0)
    uint8_t reg = 0;

    SPI.beginTransaction(this->spi_settings);

    digitalWrite(this->cs_pin, LOW);
    PAUSE_SHORT;
    SPI.transfer(&cmd, 1);
    SPI.transfer(&reg, 1);
    digitalWrite(this->cs_pin, HIGH);
    PAUSE_SHORT;

    
    reg &= 0x0F; // clear mux bits [7:4]
    reg |= config;
    cmd = ADS1120_CMD_WREG; // rr = 0, nn = 0

    digitalWrite(this->cs_pin, LOW);
    PAUSE_SHORT;
    SPI.transfer(&cmd, 1);
    SPI.transfer(&reg, 1);
    digitalWrite(this->cs_pin, HIGH);

    SPI.endTransaction();
}