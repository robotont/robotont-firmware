#include <stdio.h>
#include "i2cif.h"
#include "stdbool.h"

#define PWR_MGMNT_PACKET_SIZE 25 // Size of the incoming data packet

// Status byte bit positions
// bit 0: stop button pressed
// bit 1: power switch pressed
// bit 2: wall power present
// bit 3: motor power enabled
// bit 4: system power enabled
// bits 5-7: reserved for future use

typedef struct
{
    bool stop_btn_pressed;
    bool power_sw_pressed;
    bool wall_power_present;
    bool motor_power_enabled;
    bool sys_power_enabled;
    float motor_current;
    float nuc_current;
    float wall_voltage;
    float bat_voltage;
    float bat_pack_voltage;
    float bat_cell_voltages[5];
    float bat_cell_temp;
    float bat_mosfet_temp;
} MeasurementsType;

MeasurementsType pwr_mgmnt_data;
uint8_t pwr_mgmnt_data_raw[PWR_MGMNT_PACKET_SIZE];

void measurements_init(void);
void measurements_receiveCallback(I2C_HandleTypeDef *ifi2c_handler, uint8_t *I2C_Data);
void measurements_processData(void);