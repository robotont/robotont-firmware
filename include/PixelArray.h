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
 
#ifndef PixelArray_H
#define PixelArray_H
 
#include "mbed.h"
 
//!Library for the WS2812 RGB LED with integrated controller
/*!
PixelArray
*/
class PixelArray
{
public:
    //!Creates an instance of the class.
    /*!
    Pixel Array
    */
    PixelArray(int);
 
    /*!
    Destroys instance.
    */
    ~PixelArray();
 
    int* getBuf();
    
    void SetAll(unsigned int);
    void SetAllI(unsigned char);
    void SetAllR(unsigned char);
    void SetAllG(unsigned char);
    void SetAllB(unsigned char);
 
    // location, value
    void Set(int, unsigned int);
    void SetI(int, unsigned char);
    void SetR(int, unsigned char);
    void SetG(int, unsigned char);
    void SetB(int, unsigned char);
 
private:
 
    int *pbuf;
    int pbufsize;
 
    void __set_pixel_component(int index, int channel, int value);
    void __set_pixel(int index, int value);
 
};
 
#endif
