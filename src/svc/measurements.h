#include <stdio.h>
#include "i2cif.h"

#define PWR_MGMNT_PACKET_SIZE 8
typedef struct
{
    float motor_current;
    float nuc_current;
    float wall_voltage;
    float bat_voltage;
} MeasurementsType;

MeasurementsType pwr_mgmnt_data;
uint8_t pwr_mgmnt_data_raw[PWR_MGMNT_PACKET_SIZE];

void measurements_init(void);
void measurements_receiveCallback(I2CRxCallbackType ifi2c_handler, uint8_t *I2C_Data);
void measurements_processData(void);