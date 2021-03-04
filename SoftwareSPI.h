#ifndef _SOFTWARE_SPI_H_
#define _SOFTWARE_SPI_H_

#include "Arduino.h"
#include <SPI.h>

#define SWSPI_MSBFIRST 0
#define SWSPI_LSBFIRST 1

#define DELAY_250_NS asm("nop \n nop \n nop \n nop \n")

class SwSPI {
private:
    uint8_t clk_pin;
    uint8_t mosi_pin;
    uint8_t miso_pin;
    uint8_t bit_order;
    uint8_t cpol;
    uint8_t cpha;
    uint32_t clk_half_pd_counts; // units of 250ns
    uint8_t xferByte(uint8_t b);
    void delay_counts(int counts);
public:
    SwSPI(int clk, int mosi, int miso);
    void begin();
    void beginTransaction(uint32_t clock, uint8_t bitOrder, uint8_t dataMode);
    void endTransaction();
    void transfer(uint8_t* buffer, int size);
};

#endif // _SOFTWARE_SPI_H_