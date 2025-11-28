#include "measurements.h"
#include <string.h>

#include "macros.h"

void measurements_init(void)
{
    // Initialize the power management data with zeros
    pwr_mgmnt_data.motor_current = 0.0f;
    pwr_mgmnt_data.nuc_current = 0.0f;
    pwr_mgmnt_data.wall_voltage = 0.0f;
    pwr_mgmnt_data.bat_voltage = 0.0f;
    
    // Initialize i2c interface and set up the receive mechanism
    i2cif_init();
    i2cif_setReceiveCallback(measurements_receiveCallback);
    HAL_I2C_ListenCpltCallback(I2C_HANDLER_1);
}
    
void measurements_receiveCallback(I2C_HandleTypeDef *ifi2c_handler, uint8_t *ptr_rx_buf)
{
  if (ifi2c_handler == I2C_HANDLER_1)
  {
    // Copy the data using memcopy
    memcpy(pwr_mgmnt_data_raw, ptr_rx_buf, PWR_MGMNT_PACKET_SIZE);
  }

  // Process and forward the data immediately upon receiving
  measurements_processData();
}
	

void measurements_processData(void)
{
    //Process the received data
    uint16_t mtr_current_reading = (pwr_mgmnt_data_raw[0] << 8) | pwr_mgmnt_data_raw[1];
    pwr_mgmnt_data.motor_current = ((mtr_current_reading * 3.3) / (1024 * 40)) * 200; // AMPS

    uint16_t nuc_current_reading = (pwr_mgmnt_data_raw[2] << 8) | pwr_mgmnt_data_raw[3];
    pwr_mgmnt_data.nuc_current = ((nuc_current_reading * 3.3) / (1024 * 60)) * 100; // AMPS

    uint16_t wall_voltage_reading = (pwr_mgmnt_data_raw[4] << 8) | pwr_mgmnt_data_raw[5];
    pwr_mgmnt_data.wall_voltage = (((wall_voltage_reading * 3.3) / 1024) * 118) / 18; // VOLTS

    uint16_t bat_voltage_reading = (pwr_mgmnt_data_raw[6] << 8) | pwr_mgmnt_data_raw[7];
    pwr_mgmnt_data.bat_voltage = (((bat_voltage_reading * 3.3) / 1024) * 118) / 18; // VOLTS

    pwr_mgmnt_data.bat_pack_voltage = ((pwr_mgmnt_data_raw[8] << 8) | pwr_mgmnt_data_raw[9]) / 1000.0f; // VOLTS

    for (int i = 0; i < 5; i++) {
        pwr_mgmnt_data.bat_cell_voltages[i] = ((pwr_mgmnt_data_raw[10 + i*2] << 8) | pwr_mgmnt_data_raw[11 + i*2]) / 1000.0f; // VOLTS
    }

    pwr_mgmnt_data.bat_cell_temp = ((pwr_mgmnt_data_raw[20] << 8) | pwr_mgmnt_data_raw[21]) / 100.0f; // CELSIUS
    pwr_mgmnt_data.bat_mosfet_temp = ((pwr_mgmnt_data_raw[22] << 8) | pwr_mgmnt_data_raw[23]) / 100.0f; // CELSIUS

    // Debug data
    // for (int i = 0; i < 29; i++) {
    //     // Assuming there's a debug array in MeasurementsType to hold these values
    //     // If not, you can store them elsewhere as needed
    //   pwr_mgmnt_data.dbg[i] = pwr_mgmnt_data_raw[i];
    // }

    // Print the processed data
    printf("BATSTATE: %.3f:%.3f:%.3f:%.3f:",
           pwr_mgmnt_data.motor_current,
           pwr_mgmnt_data.nuc_current,
           pwr_mgmnt_data.wall_voltage,
           MAX(pwr_mgmnt_data.bat_voltage, pwr_mgmnt_data.bat_pack_voltage));
    for (int i = 0; i < 5; i++) {
        printf("%.3f", pwr_mgmnt_data.bat_cell_voltages[i]);
        if (i < 4) {
            printf(":");
        }
    }
    printf(":%.2f:%.2f\n",
           pwr_mgmnt_data.bat_cell_temp, 
           pwr_mgmnt_data.bat_mosfet_temp);
}
    