#ifndef MOTORCONFIG_H
#define MOTORCONFIG_H


// Motor configurations for v0.6

struct MotorConfig cfg0 = {
  PB_8,          // pin_dir1
  PC_8,          // pin_dir2
  PC_6,          // pin_pwm
  PC_5,          // pin_enca
  PB_9,          // pin_encb
  PC_9,          // pin_fault
  NC,            // pin_feedback
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
  PA_6,          // pin_dir1
  PA_12,         // pin_dir2
  PA_11,         // pin_pwm
  PB_12,         // pin_enca
  PA_7,          // pin_encb
  PA_5,          // pin_fault
  NC,            // pin_feedback
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
  PA_8,               // pin_dir1
  PB_2,               // pin_dir2
  PB_1,               // pin_pwm
  PB_15,              // pin_enca
  PB_10,              // pin_encb
  PA_9,               // pin_fault
  NC,                 // pin_feedback
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
