#include "mbed.h"
#include "motor.h"
#include <algorithm>

Motor::Motor(const MotorConfig& cfg)
  : dir1_(cfg.pin_dir1)
  , dir2_(cfg.pin_dir2)
  , pwm_(cfg.pin_pwm)
  , enc_(cfg.pin_enca, cfg.pin_encb, NC, cfg.enc_cpr, QEI::X4_ENCODING)
  , fault_(cfg.pin_fault)
  , pid_(cfg.pid_k_p, cfg.pid_tau_i, cfg.pid_tau_d, cfg.pid_dt)
  , current_feedback_(NULL)
  , status_(STATUS_UNINITIALIZED)
  , current_measured_(100, 0.0f)
//  , speed_limit_(2*M_PI) // 1 revolution per second
  , speed_limit_(0.3) // 30 cm/s
  , effort_limit_(0.25f) // 25% duty cycle
{

  // initialize pid to defaults
//   pid_.setInputLimits(-1.0, 1.0);  // set default limits to 10 cm/s
//   pid_.setOutputLimits(-1.0, 1.0);

  // set input and output limits for pid
  //setSpeedLimit(speed_limit_);
  pid_.setInputLimits(-1.0f, 1.0f);
  pid_.setOutputLimits(-1.0f, 1.0f);
  //setEffortLimit(effort_limit_);

  pid_.setBias(0.0);
  pid_.setMode(1);

  pwm_.period_us(10000);

  // Calculate the relation between an encoder pulse and 
  // linear speed on the wheel where it contacts the ground
  // CCW is positive when looking from the motor towards the wheel
  pulse_to_speed_ratio_ =
      -1.0f / cfg.enc_cpr / cfg.gear_ratio * 2 * M_PI / cfg.pid_dt * cfg.wheel_radius;

  // Check for feedback pin. Be aware that when the pin is initialized with NC or possibly with some other incorrect pinname, the system hangs. Currently we use NC for motor drivers that are not equipped with current sensing hardware.
  if (cfg.pin_feedback != NC)
  {
    current_feedback_ = new InterruptIn(cfg.pin_feedback);
  }

  // Attach the PID ISR ticker
  pidTicker_.attach(callback(this, &Motor::update), cfg.pid_dt);

  // Attach an ISR to count current feedback pulses
  if (current_feedback_)
  {
    current_feedback_->rise(callback(this, &Motor::onCurrentPulse));
  }
  
  fault_.rise(callback(this, &Motor::onFaultPulse));

  // store config in case we need it later.
  config_ = cfg; 
}

Motor::~Motor()
{
}


// Stop the motor
void Motor::stop()
{
  setSpeedSetPoint(0);
  stopped_ = true;
  dir1_ = 0;
  dir2_ = 0;
}

void Motor::setEffortLimit(float effort_limit)
{
  effort_limit_ = std::max(std::min(effort_limit, 1.0f), 0.0f);
}

void Motor::setPIDTunings(float k_p, float tau_i, float tau_d)
{
  pid_.setTunings(k_p, tau_i, tau_d);
}

// Limit angular velocity (in rads/s)
void Motor::setSpeedLimit(float speed_limit)
{
  speed_limit_ = std::max(0.0f, speed_limit);
}

void Motor::setEffort(float effort)
{
  if (stopped_)
  {
    return;
  }

  // limit value to [-1...1]
  // hard limit the effort for testing purposes
  //effort_ = std::max(std::min(effort, 0.2f), -0.2f); 
  //effort_ = std::max(std::min(effort, 1.0f), -1.0f);
  effort_ = std::max(-effort_limit_, std::min(effort_limit_, effort));

  if (effort_ == 0)
  {
    stop();
  }
  else if (effort_ > 0)
  {
    dir1_ = 0;
    dir2_ = 1;
    pwm_ = effort_;
  }
  else
  {
    dir1_ = 1;
    dir2_ = 0;
    pwm_ = -effort_;
  }
}

void Motor::setSpeedSetPoint(float speed)
{
  if (stopped_)
  {
    pid_.reset();
    stopped_ = false;
  }
  // Clamp to speed limits
  speed_setpoint_ = std::max(-speed_limit_, std::min(speed_limit_, speed));
  pid_.setSetPoint(speed_setpoint_);
}


void Motor::update()
{
  // calculate feedback current
  // TODO: use new range [40khz to 1 khz]
  current_measured_.Insert((current_pulse_count_ / config_.pid_dt - 1000)/1000);
  current_pulse_count_ = 0;

  // calculate speed from encoder
  speed_measured_ = enc_.getPulses() * pulse_to_speed_ratio_;
  enc_.reset();

  // calculate new effort
  pid_.setProcessValue(speed_measured_);
  setEffort(pid_.compute());
}

void Motor::onCurrentPulse(void)
{
  current_pulse_count_++;
}

void Motor::onFaultPulse(void)
{
  fault_pulse_count_++;
}
