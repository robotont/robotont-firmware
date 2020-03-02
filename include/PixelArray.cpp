#include "PixelArray.h"
 
PixelArray::PixelArray(int size)
{
    pbufsize = size;
    pbuf = new int[pbufsize];
    SetAll(0x0); // initialise memory to zeros
    
}
 
PixelArray::~PixelArray()
{
    delete[] pbuf;
}
 
void PixelArray::SetAll(unsigned int value)
{
    // for each pixel
    for (int i=0 ; i < pbufsize; i++) {
        __set_pixel(i,value);
    }
}
 
 
void PixelArray::SetAllI(unsigned char value)
{
    // for each pixel
    for (int i=0 ; i < pbufsize; i++) {
        __set_pixel_component(i,3,value);
    }
}
 
 
 
void PixelArray::SetAllR(unsigned char value)
{
    // for each pixel
    for (int i=0 ; i < pbufsize; i++) {
        __set_pixel_component(i,2,value);
    }
}
 
void PixelArray::SetAllG(unsigned char value)
{
    // for each pixel
    for (int i=0 ; i < pbufsize; i++) {
        __set_pixel_component(i,1,value);
    }
}
 
void PixelArray::SetAllB(unsigned char value)
{
    // for each pixel
    for (int i=0 ; i < pbufsize; i++) {
        __set_pixel_component(i,0,value);
    }
}
 
 
 
 
 
void PixelArray::Set(int i, unsigned int value)
{
    if ((i >= 0) && (i < pbufsize)) {
        __set_pixel(i,value);
    }
}
 
 
 
void PixelArray::SetI(int i, unsigned char value)
{
    if ((i >= 0) && (i < pbufsize)) {
        __set_pixel_component(i,3,value);
    }
}
 
 
void PixelArray::SetR(int i, unsigned char value)
{
    if ((i >= 0) && (i < pbufsize)) {
        __set_pixel_component(i,2,value);
    }
}
 
void PixelArray::SetG(int i, unsigned char value)
{
    if ((i >= 0) && (i < pbufsize)) {
        __set_pixel_component(i,1,value);
    }
}
 
void PixelArray::SetB(int i, unsigned char value)
{
    if ((i >= 0) && (i < pbufsize)) {
        __set_pixel_component(i,0,value);
    }
}
 
 
int* PixelArray::getBuf()
{
    return (pbuf);
}
 
 
// set either the I,R,G,B value of specific pixel channel
void PixelArray::__set_pixel_component(int index, int channel, int value)
{
 
    // AND with 0x00 shifted to the right location to clear the bits
    pbuf[index] &= ~(0xFF << (8 * channel));
 
    // Set the bits with an OR
    pbuf[index] |= (value << (8 * channel));
}
 
 
// set either the I,R,G,B value of specific pixel channel
void PixelArray::__set_pixel(int index, int value)
{
    // AND with 0x00 shifted to the right location to clear the bits
    pbuf[index] = value;
}
