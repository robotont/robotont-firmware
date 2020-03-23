#include "WS2812.h"

//void __NOP(void);
 
WS2812::WS2812(PinName pin, int size, int zeroHigh, int zeroLow, int oneHigh, int oneLow) : __gpo(pin)
{
    __size = size;
    __transmitBuf = new bool[size * FRAME_SIZE];
    __use_II = OFF;
    __II = 0xFF; // set global intensity to full
    __outPin = pin;
    
    // Default values designed for K64f. Assumes GPIO toggle takes ~0.4us
    setDelays(zeroHigh, zeroLow, oneHigh, oneLow);
}
 
 
WS2812::~WS2812()
{
    delete[] __transmitBuf;
}
 
void WS2812::setDelays(int zeroHigh, int zeroLow, int oneHigh, int oneLow) {
    __zeroHigh = zeroHigh;
    __zeroLow = zeroLow;
    __oneHigh = oneHigh;
    __oneLow = oneLow;
}
 
void WS2812::__loadBuf(int buf[],int r_offset, int g_offset, int b_offset) {
    for (int i = 0; i < __size; i++) {
        int color = 0;
               
        color |= ((buf[(i+g_offset)%__size] & 0x0000FF00));
        color |= ((buf[(i+r_offset)%__size] & 0x00FF0000));
        color |=  (buf[(i+b_offset)%__size] & 0x000000FF);
        color |= (buf[i] & 0xFF000000);
        
        // Outut format : GGRRBB
        // Inout format : IIRRGGBB
        unsigned char agrb[4] = {0x0, 0x0, 0x0, 0x0};
    
        unsigned char sf; // scaling factor for  II
    
        // extract colour fields from incoming
        // 0 = green, 1 = red, 2 = blue, 3 = brightness        
        agrb[0] = (color & 0x0000FF00) >> 8;
        agrb[1] = (color & 0x00FF0000) >> 16;
        agrb[2] = color  & 0x000000FF;
        agrb[3] = (color & 0xFF000000) >> 24;
    
        // set the intensity scaling factor (global, per pixel, none)
        if (__use_II == GLOBAL) {
            sf = __II;
        } else if (__use_II == PER_PIXEL) {
            sf = agrb[3];
        } else {
            sf = 0xFF;
        }
        
        // Apply the scaling factor to each othe colour components
        for (int clr = 0; clr < 3; clr++) {
            agrb[clr] = ((agrb[clr] * sf) >> 8);
            
            for (int j = 0; j < 8; j++) {
                if (((agrb[clr] << j) & 0x80) == 0x80) {
                    // Bit is set (checks MSB fist)
                    __transmitBuf[(i * FRAME_SIZE) + (clr * 8) + j] = 1;
                } else {
                    // Bit is clear
                    __transmitBuf[(i * FRAME_SIZE) + (clr * 8) + j] = 0;
                }
            }
        }
    }
}
 
void WS2812::write(int buf[]) {
    write_offsets(buf, 0, 0, 0);
}
 
void WS2812::write_offsets (int buf[],int r_offset, int g_offset, int b_offset) {
    int i, j;
    
    // Load the transmit buffer
    __loadBuf(buf, r_offset, g_offset, b_offset);
 
    // Entering timing critical section, so disabling interrupts
    __disable_irq();
    
    // Begin bit-banging
    for (i = 0; i < FRAME_SIZE * __size; i++) {
        j = 0;
        if (__transmitBuf[i]){
            __gpo = 1;
            for (; j < __oneHigh; j++) {
                __NOP();
            }
            __gpo = 0;
            for (; j < __oneLow; j++) {
                __NOP();
            }
        } else {
            __gpo = 1;
            for (; j < __zeroHigh; j++) {
                __NOP();
            }
            __gpo = 0;
            for (; j < __zeroLow; j++) {
                __NOP();
            }
        }
    }
    
    // Exiting timing critical section, so enabling interrutps
    __enable_irq();

    wait(0.075);
}
 
 
void WS2812::useII(BrightnessControl bc)
{
    if (bc > OFF) {
        __use_II = bc;
    } else {
        __use_II = OFF;
    }
}
 
void WS2812::setII(unsigned char II)
{
    __II = II;
}
