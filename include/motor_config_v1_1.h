#ifndef MOTORCONFIG_H
#define MOTORCONFIG_H


// Motor configurations for v1.1

struct MotorConfig cfg0 = {
  PC_8,          // pin_dir1
  PC_9,          // pin_dir2
  PC_6,          // pin_pwm
  PC_5,          // pin_enca
  PA_12,         // pin_encb
  PA_6,          // pin_fault
  PA_11,         // pin_feedback
  NC,            // pin_temp
  PID_KP,        // pid_k_p
  PID_TI,        // pid_tau_i
  PID_TD,        // pid_tau_d
  PID_DELTA_T,   // pid_dt
  ENC_CPR,       // enc_cpr
  GEAR_RATIO,    // gear_ratio
  WHEEL_RADIUS,  // wheel_radius
  WHEEL_POS_R,   // wheel_pos_r
  M_PI / 3.0f    // wheel_pos_phi
};

struct MotorConfig cfg1 = {
  PB_15,         // pin_dir1
  PB_1,          // pin_dir2
  PB_14,         // pin_pwm
  PB_4,          // pin_enca
  PB_13,         // pin_encb
  PB_3,          // pin_fault
  PA_10,         // pin_feedback
  NC,            // pin_temp
  PID_KP,        // pid_k_p
  PID_TI,        // pid_tau_i
  PID_TD,        // pid_tau_d
  PID_DELTA_T,   // pid_dt
  ENC_CPR,       // enc_cpr
  GEAR_RATIO,    // gear_ratio
  WHEEL_RADIUS,  // wheel_radius
  WHEEL_POS_R,   // wheel_pos_r
  M_PI           // wheel_pos_phi
};

struct MotorConfig cfg2 = {
  PB_12,              // pin_dir1
  PA_7,               // pin_dir2
  PB_6,               // pin_pwm
  PC_7,               // pin_enca
  PA_9,               // pin_encb
  PB_2,               // pin_fault
  PA_8,               // pin_feedback
  NC,                 // pin_temp
  PID_KP,             // pid_k_p
  PID_TI,             // pid_tau_i
  PID_TD,             // pid_tau_d
  PID_DELTA_T,        // pid_dt
  ENC_CPR,            // enc_cpr
  GEAR_RATIO,         // gear_ratio
  WHEEL_RADIUS,       // wheel_radius
  WHEEL_POS_R,        // wheel_pos_r
  5.0f / 3.0f * M_PI  // wheel_pos_phi
};

#endif
