#include "measurements.h"
#include <string.h>

void measurements_init(void)
{
  // Initialize the measurements
    i2cif_init();
    i2cif_setReceiveCallback(measurements_receiveCallback);
    
    pwr_mgmnt_data.motor_current = 0.0f;
    pwr_mgmnt_data.nuc_current = 0.0f;
    pwr_mgmnt_data.wall_voltage = 0.0f;
    pwr_mgmnt_data.bat_voltage = 0.0f;
}
    
void measurements_receiveCallback(I2C_HandleTypeDef *ifi2c_handler, uint8_t *ptr_rx_buf)
{
  if (ifi2c_handler == I2C_HANDLER_1)
  {
    // Copy the data using memcopy
    memcpy(pwr_mgmnt_data_raw, ptr_rx_buf, 8);
  }

  // Process and forward the data immediately upon receiving
  measurements_processData();
}
	

void measurements_processData(void)
{
  // Process the received data
    uint16_t mtr_current_reading = (pwr_mgmnt_data_raw[0] << 8) | pwr_mgmnt_data_raw[1];
    pwr_mgmnt_data.motor_current = ((mtr_current_reading * 3.3) / (1024 * 40)) * 200; // AMPS

    uint16_t nuc_current_reading = (pwr_mgmnt_data_raw[2] << 8) | pwr_mgmnt_data_raw[3];
    pwr_mgmnt_data.nuc_current = ((nuc_current_reading * 3.3) / (1024 * 60)) * 100; // AMPS

    uint16_t wall_voltage_reading = (pwr_mgmnt_data_raw[4] << 8) | pwr_mgmnt_data_raw[5];
    pwr_mgmnt_data.wall_voltage = (((wall_voltage_reading * 3.3) / 1024) * 118) / 18; // VOLTS

    uint16_t bat_voltage_reading = (pwr_mgmnt_data_raw[6] << 8) | pwr_mgmnt_data_raw[7];
    pwr_mgmnt_data.bat_voltage = (((bat_voltage_reading * 3.3) / 1024) * 118) / 18; // VOLTS

    // Print the processed data
    printf("BATSTATE:%f:%f:%f:%f\n", pwr_mgmnt_data.motor_current, pwr_mgmnt_data.nuc_current, pwr_mgmnt_data.wall_voltage, pwr_mgmnt_data.bat_voltage);
    //printf("Motor Current: %f A, Nuc Current: %f A, Wall Voltage: %f V, Bat Voltage: %f V\n",MtrCurrent, NucCurrent, WallVoltage, BatVoltage);
}