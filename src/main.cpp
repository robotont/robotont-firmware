#include "mbed.h"
#include "QEI.h"
#include "PID.h"

#define MAXSPEED 100
#define MAXSPEEDINPUT 100
//#define KP 2
#define KP 0.5
#define KI 0.25
#define KD 0
#define DELTA_T 0.01

#define M0_FAULT PA_9
#define M0_DIR2 PB_2
#define M0_DIR1 PA_8
#define M0_PWM PB_1
#define M0_ENCA PB_10
#define M0_ENCB PB_15

#define M1_FAULT PA_5
#define M1_DIR2 PA_12
#define M1_DIR1 PA_6
#define M1_PWM PA_11
#define M1_ENCA PA_7
#define M1_ENCB PB_12

#define M2_FAULT PC_9
#define M2_DIR2 PC_8
#define M2_DIR1 PB_8
#define M2_PWM PC_6
#define M2_ENCA PB_9
#define M2_ENCB PC_5

#include "DS1820.h"

class Motor
{

  private:
    DigitalOut dir1, dir2;
    PwmOut pwm;
    QEI enc;
    DigitalIn fault;
    PID pid;

    volatile int currentSpeed;
    int expectedSpeed;
    bool stopped;

  public:

    Motor(PinName dir1pin, PinName dir2pin, PinName pwmpin, PinName encapin, PinName encbpin, PinName faultpin):
      dir1(dir1pin),
      dir2(dir2pin),
      pwm(pwmpin),
      enc(encapin, encbpin, NC, 1200, QEI::X4_ENCODING),
      fault(faultpin),
      pid(KP, KI, KD, DELTA_T),
      stopped(true)
  {
      //Initialize motor
      pid.setInputLimits(-1.0, 1.0);
      pid.setOutputLimits(-1.0, 1.0);
      pid.setBias(0.0);
      pid.setMode(1);

      //fault.mode(PullUp);
  }

    void stop()
{
      stopped = true;
      dir1 = 0;
      dir2 = 0;
}

    void setSpeed(float spd)
    {
      if (stopped)
      {
        return;
      }
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

    void setExpectedSpeed(int speed) 
    {
      if (stopped)
      {
        pid.reset();
        stopped = false;
      }
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
}; // Motor

Motor m[3] = {
  {M2_DIR1, M2_DIR2, M2_PWM, M2_ENCA, M2_ENCB, M2_FAULT},
  {M0_DIR1, M0_DIR2, M0_PWM, M0_ENCA, M0_ENCB, M0_FAULT},
  {M1_DIR1, M1_DIR2, M1_PWM, M1_ENCA, M1_ENCB, M1_FAULT}
};
//Motor m(M0_DIR1, M0_DIR2, M0_PWM, M0_ENCA, M0_ENCB, M0_FAULT);
//
//
//DigitalOut LED1R(P4_28);
//DigitalOut LED2R(P0_28);
Ticker pidTicker;
//
void processPID()
{
  for (uint8_t i = 0; i < 3; i++)
  {
    m[i].processPID();
  }
}

Serial serial_pc(USBTX, USBRX); //tx, rx

//DigitalOut led1(LED1);
volatile int speed[3];
volatile float temperatures[3];
char serial_buf[256];
uint8_t serial_arrived = 0;

void pc_rx_callback()
{
  //if(serial_pc.readable() < 6)
  //{
  //      return;
  //}
  //speed[0] = serial_pc.getc();
  //printf("%c", serial_pc.getc());

  //serial_pc.scanf("%d:%d:%d", &speed[0], &speed[1], &speed[2]);
  //led1 = !led1;
  if(serial_pc.readable())
  {
    char c = serial_pc.getc();
    serial_buf[serial_arrived++] = c;
    serial_buf[serial_arrived] = '\0';
    if (serial_arrived > 254)
    {
      serial_arrived = 0;
    }

    if(c == 13) //enter
    {
      sscanf(serial_buf, "%d:%d:%d", &speed[0], &speed[1], &speed[2]);
      for (uint8_t i = 0; i < 3; i++) {
        m[i].setExpectedSpeed(speed[i]);
      }
      serial_buf[0] = '\0';
      serial_arrived = 0;
    }

    if(c == 27) //esc
    {
      for (uint8_t i = 0; i < 3; i++)
      {
        m[i].stop();
        speed[i] = 0;
      }
      serial_buf[0] = '\0';
      serial_arrived = 0;
      //printf("STOP");
    }
  }
}

int main() {
  pidTicker.attach(&processPID, DELTA_T);
  unsigned long iteration = 0;

//  QEI enc(PB_15, PB_10, NC, 1200, QEI::X4_ENCODING);
  serial_pc.baud(115200);
  serial_buf[0] = '\0';
  //serial_arrived = 0;
  serial_pc.attach(&pc_rx_callback);

  // Variables for temperature measurement
  DS1820 ds1820[3] = {DS1820(PC_0), DS1820(PC_1), DS1820(PB_0)};
  float temp = 0;
  int error = 0;

  // Wait for all the temperature sensors
  for (int i = 0; i < 3; i++) {
    while (!ds1820[i].begin()) {
      serial_pc.printf("Couldn't find sensor %d", i);
    }
    ds1820[i].startConversion();
  }

  // MAIN LOOP
  while (true) {
    //serial_pc.printf("%u", iteration++);
    if (iteration++ % 10 == 0) { //Poll temperatures after every 10th cycle
      // Read the temperatures and print to serial
      for (int i = 0; i < 3; i++) {
        if (ds1820[i].isPresent()) {
          while (ds1820[i].read(temp) != 0)
            ;                                // Retry until we got it right.
          temperatures[i] = temp;
          //serial_pc.printf("%3.1f", temp);  // read temperature
          ds1820[i].startConversion();       // start temperature conversion
        }
      }
      serial_pc.printf("Temperatures %3.1f:%3.1f:%3.1f\r\n", temperatures[0],
                       temperatures[1], temperatures[2]);
      // serial_pc.printf("%3.1f", temp);  // read temperature
    }

    // Read the command speed values from serial when arrived
    if (serial_pc.readable())
    {
      char c = serial_pc.getc();
      serial_buf[serial_arrived++] = c;
      serial_buf[serial_arrived] = '\0';
      if (serial_arrived > 254)
      {
        serial_arrived = 0;
      }

      if(c == 13) //enter
      {
        sscanf(serial_buf, "%d:%d:%d", &speed[0], &speed[1], &speed[2]);
        for (uint8_t i = 0; i < 3; i++) {
          m[i].setExpectedSpeed(speed[i]);
        }
      }

      if(c == 27) //esc
      {
        for (uint8_t i = 0; i < 3; i++)
        {
          m[i].stop();
          speed[i] = 0;
        }
        //printf("STOP");
      }
    }
    
    //led1 = !led1;
    //serial_pc.printf("Buf(%d) = ", strlen(serial_buf));
    //for(uint8_t i = 0; i<serial_arrived; i++)
    //{
	  //  serial_pc.printf("%c", serial_buf[i]);
    //}
    //serial_pc.printf("\r\n");
    serial_pc.printf("Setpoints %d:%d:%d\r\n", speed[0], speed[1], speed[2]);
    serial_pc.printf("Encoders %d:%d:%d\r\n\r\n",  m[0].getSpeed(), m[1].getSpeed(), m[2].getSpeed());
    wait(0.02); 
  }
}
