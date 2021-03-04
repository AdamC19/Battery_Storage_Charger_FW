#include "SoftwareSPI.h"

SwSPI::SwSPI(int clk, int mosi, int miso){
    this->clk_pin = clk;
    this->mosi_pin = mosi;
    this->miso_pin = miso;
}

void SwSPI::begin(){
    pinMode(clk_pin, OUTPUT);
    pinMode(mosi_pin, OUTPUT);
    pinMode(miso_pin, INPUT);
}

void SwSPI::delay_counts(int counts){
    int i = 0;
    do{
        i++;
        // asm("nop \n");
        asm("nop \n");
    }while(i < counts);
}

void SwSPI::beginTransaction(uint32_t clock, uint8_t bitOrder, uint8_t dataMode){
    
    float clk_pd = (1.0/clock) * 1.0e9; // clock period in ns
    if(clk_pd < 500){
        clk_pd = 500;
    }

    clk_half_pd_counts = uint32_t((clk_pd / (2.0 * 250.0)) + 0.5);

    bit_order = bitOrder;

    cpha = dataMode & 1;
    cpol = (dataMode >> 1) & 1;

    
    digitalWrite(clk_pin, cpol); // de-assert clock pin
    
}

uint8_t SwSPI::xferByte(uint8_t b){
    uint8_t retval = 0;
    uint8_t bit = 0;
    for(int i = 0; i < 8; i++){
        if(cpha){
            // change data with leading edge, sample on trailing edge
            digitalWrite(clk_pin, !cpol);                // assert clock pin
            // if(!cpol){
            //     PORTB |= (1 << 5);
            // }else{
            //     PORTB &= ~(1 << 5);}

            // bit = (b >> (7-i)) & 1;
            digitalWrite(mosi_pin, (b >> (7-i)) & 1);   // change data with leading edge
            // if(bit){
            //     PORTB |= (1 << 4);
            // }else{
            //     PORTB &= ~(1 << 4);}
            
            DELAY_250_NS;// delay_counts(clk_half_pd_counts);
            digitalWrite(clk_pin, cpol);               // de-assert clock pin
            // if(cpol){
            //     PORTB |= (1 << 5);
            // }else{
            //     PORTB &= ~(1 << 5);}

            retval |= digitalRead(miso_pin) << (7-i);   // sample data on trailing edge
            // retval |= ((PORTB >> 3) & 1) << (7-i);
            DELAY_250_NS;// delay_counts(clk_half_pd_counts);
        }else{
            // capture on leading edge, change data on trailing edge
            digitalWrite(clk_pin, !cpol);                // assert clock pin
            // if(!cpol){
            //     PORTB |= (1 << 5);
            // }else{
            //     PORTB &= ~(1 << 5);}

            retval |= digitalRead(miso_pin) << (7-i);   // sample data on leading edge
            // retval |= ((PORTB >> 3) & 1) << (7-i);
            DELAY_250_NS;// delay_counts(clk_half_pd_counts);
            digitalWrite(clk_pin, cpol);               // de-assert clock pin
            // if(cpol){
            //     PORTB |= (1 << 5);
            // }else{
            //     PORTB &= ~(1 << 5);}

            digitalWrite(mosi_pin, (b >> (7-i)) & 1 );  // change data with trailing edge
            // if(bit){
            //     PORTB |= (1 << 4);
            // }else{
            //     PORTB &= ~(1 << 4);}
            DELAY_250_NS;// delay_counts(clk_half_pd_counts);
        }
        
    }
    return retval;
}


void SwSPI::endTransaction(){
    digitalWrite(clk_pin, cpol);               // de-assert clock pin
}

void SwSPI::transfer(uint8_t* buffer, int size){
    for(int i = 0; i < size; i++){
        buffer[i] = this->xferByte(buffer[i]);
    }
}

