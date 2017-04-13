#include "mbed.h"
#include "QEI.h"
#include "PID.h"
#include "USBSerial.h"

#define MAXSPEED 100
#define MAXSPEEDINPUT 100
#define KP 2
#define KI 0.25
#define KD 0
#define DELTA_T 0.01

#define M0_PWM P2_3
#define M0_DIR1 P0_21
#define M0_DIR2 P0_20
#define M0_FAULT P0_22
#define M0_ENCA P0_19
#define M0_ENCB P0_18

#define M1_PWM P2_2
#define M1_DIR1 P0_15
#define M1_DIR2 P0_16
#define M1_FAULT P0_17
#define M1_ENCA P2_7
#define M1_ENCB P2_6

#define M2_PWM P2_1
#define M2_DIR1 P0_24
#define M2_DIR2 P0_25
#define M2_FAULT P0_23
#define M2_ENCA P0_26
#define M2_ENCB P0_9

class Motor {

  private:
    volatile int currentSpeed;
    int expectedSpeed;
    QEI enc;
    PID pid;
    DigitalOut dir1, dir2;
    PwmOut pwm;
    DigitalIn fault;

  public:

    Motor(PinName dir1pin, PinName dir2pin, PinName pwmpin, PinName encapin, PinName encbpin, PinName faultpin):
      dir1(dir1pin),
      dir2(dir2pin),
      pwm(pwmpin),
      fault(faultpin),
      enc(encapin, encbpin, NC, 1200, QEI::X4_ENCODING),
      pid(KP, KI, KD, DELTA_T)
    {
      pid.setInputLimits(-1.0, 1.0);
      pid.setOutputLimits(-1.0, 1.0);
      pid.setBias(0.0);
      pid.setMode(1);
    }

    void stop() {
      dir1 = 0;
      dir2 = 0;
    }

    void setSpeed(float spd) {
      if (spd > 1.0) {
        spd = 1.0;
      } else if (spd < -1.0) {
        spd = -1.0;
      }

      if (spd == 0) {
        stop();
      } else if (spd > 0) {
        dir1 = 1;
        dir2 = 0;
        pwm = spd;
      } else {
        dir1 = 0;
        dir2 = 1;
        pwm = -spd;
      }
    }

    void setExpectedSpeed(int speed) {
      expectedSpeed = speed;
    }

    int getSpeed() {
      return currentSpeed;
    }

    void processPID() {
      currentSpeed = enc.getPulses();
      enc.reset();
      pid.setProcessValue((float) currentSpeed / MAXSPEED);
      pid.setSetPoint((float) expectedSpeed / MAXSPEEDINPUT);
      setSpeed(pid.compute());
    }
};

USBSerial serial;
Motor m[3] = {
  {M0_DIR1, M0_DIR2, M0_PWM, M0_ENCA, M0_ENCB, M0_FAULT},
  {M1_DIR1, M1_DIR2, M1_PWM, M1_ENCA, M1_ENCB, M1_FAULT},
  {M2_DIR1, M2_DIR2, M2_PWM, M2_ENCA, M2_ENCB, M2_FAULT}
};


DigitalOut LED1R(P4_28);
DigitalOut LED2R(P0_28);
Ticker pidTicker;

void processPID() {
  for (uint8_t i = 0; i < 3; i++) {
    m[i].processPID();
  }
}

int main() {
  pidTicker.attach(&processPID, DELTA_T);

  int speed[3];

  while (true) {
    serial.printf("%d:%d:%d\n",  m[0].getSpeed(), m[1].getSpeed(), m[2].getSpeed());
    if (serial.available()) {
      serial.scanf("%d:%d:%d", &speed[0], &speed[1], &speed[2]);
      for (uint8_t i = 0; i < 3; i++) {
        m[i].setExpectedSpeed(speed[i]);
      }
    }
    wait(0.01);
  }
}
