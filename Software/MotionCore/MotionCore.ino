#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_SPIDevice.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  110 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  525 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define DEBUG 0

#define maxlength 93
#define minlength 62

// our servo # counter
uint8_t servonum = 0;

const int LegMapping[6][3] = {{0, 1, -1},
                              {2, 3, -1},
                              {4, 5, 6},
                              {7, 8, 9},
                              {10, 11, 12},
                              {13, 14, 15}};

const int OffsetMapping[6][3] = {{0, 0, -2},
                                {0, 0, -2},
                                {0, 10, 10},
                                {0, 10, 10},
                                {0, 10, 10},
                                {0, 10, 10}};

float ServoPositions[16] = {45,45,45,45,45,45,0,0,180,180,180,180,180,180,0,0};

void setup() {
  Serial.begin(9600);

  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importauint16_t to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}


void setServos() {
  int pwmVal;
  for(int i = 0; i < 16; i ++) {
    
    pwmVal = map(ServoPositions[i], 0, 180, SERVOMIN, SERVOMAX);
    #ifdef DEBUG == 1
      Serial.print(ServoPositions[i]);
      Serial.print(", ");
    #endif
    pwm.setPWM(i, 0, pwmVal);
  }
  Serial.println("");
}

void CalcHeightAngles(float dis, float * ServoPositions, int LegNum) {

  if (dis > maxlength) { dis = maxlength;}
  if (dis < minlength) { dis = minlength;}
  float l1 = 55.25;
  float l2 = 38.4;

  float t1 = acos((pow(dis,2)+pow(l1,2)-pow(l2,2))/(2*dis*l1)) * 180 / PI; //Knee Joint Angle
  float t2 = acos((-pow(dis,2)+pow(l1,2)+pow(l2,2))/(2*l1*l2)) * 180 / PI; //Ankle Joint Angle

  int offset1 = OffsetMapping[LegNum][1];
  int offset2 = OffsetMapping[LegNum][2];
  
  t1 = map(t1, 180, 0, 0 - offset1, 180 - offset1);
  t2 = map(t2, 180, 0, 90 - offset2, 270 - offset2);

  // t1 = 180-t1 -10;
  // t2 = map(t2, 170, 60, 90, 200);
  
  ServoPositions[LegMapping[LegNum][1]] = t1;
  ServoPositions[LegMapping[LegNum][2]] = t2;
  #ifdef DEBUG
  // Serial.print(ServoPositions[0]);
  // Serial.print(",");
  // Serial.print(ServoPositions[1]);
  // Serial.print(",");
  // Serial.print(ServoPositions[2]);
  // Serial.print(",");
  // Serial.print(ServoPositions[3]);
  // Serial.print(",");
  // Serial.print(ServoPositions[4]);
  // Serial.print(",");
  // Serial.print(ServoPositions[5]);
  // Serial.print(",");
  // Serial.print(ServoPositions[6]);
  // Serial.print(",");
  // Serial.print(ServoPositions[7]);
  // Serial.print(",");
  // Serial.print(ServoPositions[8]);
  // Serial.print(",");
  // Serial.print(ServoPositions[9]);
  // Serial.print(",");
  // Serial.print(ServoPositions[10]);
  // Serial.print(",");
  // Serial.print(ServoPositions[11]);
  // Serial.print(",");
  // Serial.print(ServoPositions[12]);
  // Serial.print(",");
  // Serial.print(ServoPositions[13]);
  // Serial.print(",");
  // Serial.print(ServoPositions[14]);
  // Serial.print(",");
  // Serial.println(ServoPositions[15]);
  
  #endif
}

void loop() {
  // for (int i = SERVOMIN; i < SERVOMAX; i++) {
  //   pwm.setPWM(8, 0, i);
  //   Serial.println(i);
  //   delay(100);
  // }

  // coil();
  float dis = 60;

  while(dis < 95) {
    CalcHeightAngles(dis, ServoPositions, 2);
    CalcHeightAngles(dis, ServoPositions, 3);
    CalcHeightAngles(dis, ServoPositions, 4);
    CalcHeightAngles(dis, ServoPositions, 5);
    delay(100);
    setServos();
    dis++;
  }

  Serial.println("Reset");
  
}

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
//void setServoPulse(uint8_t n, double pulse) {
//  double pulselength;
//  
//  pulselength = 1000000;   // 1,000,000 us per second
//  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
//  Serial.print(pulselength); Serial.println(" us per period"); 
//  pulselength /= 4096;  // 12 bits of resolution
//  Serial.print(pulselength); Serial.println(" us per bit"); 
//  pulse *= 1000000;  // convert input seconds to us
//  pulse /= pulselength;
//  Serial.println(pulse);
//  pwm.setPWM(n, 0, pulse);
//}
