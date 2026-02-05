/**
 * @file float_format.h
 * @brief Lightweight float-to-string conversion with ZERO dynamic memory allocation
 *        Safe for embedded systems - uses only stack buffers
 */

#ifndef FLOAT_FORMAT_H
#define FLOAT_FORMAT_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Format ODOM message efficiently
 * @param buffer    Output buffer (must be at least 128 bytes)
 * @param buf_size  Size of buffer
 * @param pos_x, pos_y, pos_z    Position values
 * @param vel_x, vel_y, vel_z    Velocity values
 * @return          Number of characters written
 */
int format_odom(char *buffer, size_t buf_size,
                float pos_x, float pos_y, float pos_z,
                float vel_x, float vel_y, float vel_z);

/**
 * @brief Format Joint States message efficiently
 * @param buffer    Output buffer (must be at least 160 bytes)
 * @param buf_size  Size of buffer
 * @param pos_0, pos_1, pos_2    Position values (radians)
 * @param vel_0, vel_1, vel_2    Velocity values (rad/s)
 * @param eff_0, eff_1, eff_2    Effort values (duty cycle %)
 * @return          Number of characters written
 */
int format_joint_states(char *buffer, size_t buf_size,
                        float pos_0, float pos_1, float pos_2,
                        float vel_0, float vel_1, float vel_2,
                        float eff_0, float eff_1, float eff_2);

#ifdef __cplusplus
}
#endif

#endif // FLOAT_FORMAT_H