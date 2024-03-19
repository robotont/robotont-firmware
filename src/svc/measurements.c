#include "measurements.h"

    
void processData(){
	// Process the received data
    uint16_t MtrCurrentReading = (I2C_Data[0] << 8) | I2C_Data[1];
    MtrCurrent = ((MtrCurrentReading * 3.3) / (1024 * 40)) * 200; // AMPS

    uint16_t NucCurrentReading = (I2C_Data[2] << 8) | I2C_Data[3];
    NucCurrent = ((NucCurrentReading * 3.3) / (1024 * 60)) * 100; // AMPS

    uint16_t WallVoltageReading = (I2C_Data[4] << 8) | I2C_Data[5];
    WallVoltage = (((WallVoltageReading * 3.3) / 1024) * 118) / 18; // VOLTS

    uint16_t BatVoltageReading = (I2C_Data[6] << 8) | I2C_Data[7];
    BatVoltage = (((BatVoltageReading * 3.3) / 1024) * 118) / 18; // VOLTS

    printMeasurements();
}

void printMeasurements(){
    // Print the processed data
    printf("BATSTATE:%f:%f:%f:%f\n", MtrCurrent, NucCurrent, WallVoltage, BatVoltage);
    //printf("Motor Current: %f A, Nuc Current: %f A, Wall Voltage: %f V, Bat Voltage: %f V\n",MtrCurrent, NucCurrent, WallVoltage, BatVoltage);
}