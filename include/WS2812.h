	

/* Copyright (c) 2012 cstyles, MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
 
#ifndef WS2812_H
#define WS2812_H
 
#include "mbed.h"
 
#define FRAME_SIZE 24
 
//!Library for the WS2812 RGB LED with integrated controller
/*!
The WS2812 is controller that is built into a range of LEDs
*/
class WS2812
{
public:
    enum BrightnessControl { OFF, GLOBAL, PER_PIXEL };
 
    /**
    *   Constructor
    *
    * @param pin Output pin. Connect to "Din" on the first WS2812 in the strip
    * @param size Number of LEDs in your strip
    * @param zeroHigh How many NOPs to insert to ensure TOH is properly generated. See library description for more information.
    * @param zeroLow How many NOPs to insert to ensure TOL is properly generated. See library description for more information.
    * @param oneHigh How many NOPs to insert to ensure T1H is properly generated. See library description for more information.
    * @param oneLow How many NOPs to insert to ensure T1L is properly generated. See library description for more information.
    *
    */
    WS2812(PinName pin, int size, int zeroHigh, int zeroLow, int oneHigh, int oneLow);
 
    /*!
    Destroys instance.
    */
    ~WS2812();
    
    /**
    *   Sets the timing parameters for the bit-banged signal
    *
    * @param zeroHigh How many NOPs to insert to ensure TOH is properly generated. See library description for more information.
    * @param zeroLow How many NOPs to insert to ensure TOL is properly generated. See library description for more information.
    * @param oneHigh How many NOPs to insert to ensure T1H is properly generated. See library description for more information.
    * @param oneLow How many NOPs to insert to ensure T1L is properly generated. See library description for more information.
    *
    */
    void setDelays(int zeroHigh, int zeroLow, int oneHigh, int oneLow);
 
    /**
    *   Writes the given buffer to the LED strip with the given offsets.
    *   NOTE: This function is timing critical, therefore interrupts are disabled during the transmission section.
    *
    * @param buf Pointer to the PixelArray buffer
    * @param r_offset The offset where each each pixel pulls its red component. Wraps to beginning if end is reached.
    * @param g_offset The offset where each each pixel pulls its green component. Wraps to beginning if end is reached.
    * @param b_offset The offset where each each pixel pulls its blue component. Wraps to beginning if end is reached.
    *
    */
    void write_offsets(int buf[], int r_offset = 0, int g_offset = 0, int b_offset = 0);
 
 
    /**
    *   Writes the given buffer to the LED strip
    *   NOTE: This function is timing critical, therefore interrupts are disabled during the transmission section.
    *
    * @param buf Pointer to the PixelArray buffer
    *
    */
    void write(int buf[]);
    
    /**
    *   Sets the brightness mode
    *
    * @param bc The brightness control. Defaults to OFF. Possible values include OFF, GLOBAL, and PER_PIXEL
    *
    */
    void useII(BrightnessControl bc);
    
    /**
    *   Sets the global brightness level.
    *
    * @param II The brightness level. Possible values include 0 - 255 (0x00 - 0xFF).
    *
    */
    void setII(unsigned char II);
    
 
 
private:
 
    int __size;
    int __zeroHigh, __zeroLow, __oneHigh, __oneLow;
    unsigned char __II;
    BrightnessControl __use_II;
    bool *__transmitBuf;
    void __loadBuf(int buf[],int r_offset=0, int g_offset=0, int b_offset=0);
    PinName __outPin;
    DigitalOut __gpo;
};
 
#endif
