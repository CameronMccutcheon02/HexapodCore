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
#define SERVOMIN  125 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  500 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define DEBUG 0

// our servo # counter
uint8_t servonum = 0;

int ServoPositions[16] = {45,45,45,45,45,45,0,0,180,180,180,180,180,180,0,0};

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
  for(int i = 0; i < 14; i ++) {
    
    pwmVal = map(ServoPositions[i], 0, 180, SERVOMIN, SERVOMAX);
    if(DEBUG == 1) {
      Serial.print(ServoPositions[i]);
      Serial.print(",");
      Serial.println(pwmVal);
    }
    pwm.setPWM(i, 0, pwmVal);
  }
}

void stand(){
  Serial.println("Stand");
  for(int i = 8; i < 14; i++) {
    ServoPositions[i] = 155;
  }
  setServos();
}

void coil() {
  Serial.println("coil)");
  for(int i = 8; i < 14; i++) {
    ServoPositions[i] = 0;
  }
  setServos();
}

void step(){

  //Bring up Top 3
  for(int i = 8; i < 12; i = i + 2) {
    ServoPositions[i] = 90;
  }
  ServoPositions[0] = 20;
  ServoPositions[2] = 70;
  setServos();
  delay(2500);

  //Set feet Back Down
  for(int i = 8; i < 12; i = i + 2) {
    ServoPositions[i] = 180;
  }
  setServos();
  delay(2500);
  //Lift the other ones
  for(int i = 9; i < 12; i = i + 2) {
    ServoPositions[i] = 90;
  }

  //Rotate Back
  ServoPositions[0] = 45;
  ServoPositions[2] = 45;
  setServos();
  delay(2500);
}

void loop() {
  // for (int i = SERVOMIN; i < SERVOMAX; i++) {
  //   pwm.setPWM(8, 0, i);
  //   Serial.println(i);
  //   delay(100);
  // }

  // coil();

  delay(5000);
  ServoPositions[0] = 180;
  ServoPositions[1] = 90;
  setServos();
  Serial.println("running");
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
