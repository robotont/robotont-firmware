#include <stdio.h>


uint8_t I2C_Data[8];

float MtrCurrent;
float NucCurrent;
float WallVoltage;
float BatVoltage;

void printMeasurements();
void processData();