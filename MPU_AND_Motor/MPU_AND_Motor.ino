//Sensor readings with offsets:  11  -2  16387 2 0 1
//Your offsets: -1112 1603  539 65  52  -30
// Sensor readings with offsets: -7  -2  16389 -1  0 1
// Your offsets: -3829 -1921 856 -6  -13 -1

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//     2012-06-20 - improved FIFO overflow handling and simplified read process
//     2012-06-19 - completely rearranged DMP initialization code and simplification
//     2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//     2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//     2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                - add 3D math helper file to DMP6 example sketch
//                - add Euler output and Yaw/Pitch/Roll output formats
//     2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//     2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//     2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include "pid.h"
#include "Timer.h"
#include "button.h"
#include "led.h"
#include <Servo.h>
#include <SD.h>
#include <RH_ASK.h>

#define _PI 3.141592653589
//  RC
const float __maxRC_angleRange      = sin(30 *_PI/180); //deg
const float __RC_deadzone           = 5; // %
const unsigned int MIN_PULSE_LENGTH = 1000;
const unsigned int MAX_PULSE_LENGTH = 1920;

//  MPU
const float __xAngleOffset      = 0.04;
const float __yAngleOffset      = 0;


//  Motor
const byte __motorEnablePoint   = 18; // %
const byte __motorDisablePoint  = 15; // %
const float Throttle            = 1200;
const float __maxThrottle         = 2000;
const float __minThrottle         = 900;




//#include "MPU6050.h" // not necessary if using MotionApps include file

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */
const int chipSelect = BUILTIN_SDCARD;
bool enableLog = false;
int fileIndex = 0;
Timer logFileTimer;
unsigned int logFileUpdateTime = 10;
bool enableSerialDump = false;

 
const byte led_green_pin  = 23;
const byte led_yellow_pin = 22;
const byte led_red_pin    = 21;
Led led_green(led_green_pin);
Led led_yellow(led_yellow_pin);
Led led_red(led_red_pin);

const byte akku_c1_pin = 35;
const byte akku_c2_pin = 34;
const byte akku_c3_pin = 33;
float voltage_c1;
float voltage_c2;
float voltage_c3;
float voltagePercent;
Timer akkuTimer;


const byte fl_pin = 9;
const byte fr_pin = 10;
const byte bl_pin = 11;
const byte br_pin = 12;

const byte fl_TLM_pin = 14;
const byte fr_TLM_pin = 15;
const byte bl_TLM_pin = 16;
const byte br_TLM_pin = 17;

float totalAngleY = 0;
float totalAngleX = 0;
unsigned long lastTime = 0;
float elapsedTime = 0;
float kp = 0;
float ki = 0;
float kd = 0;
//PID pid(5,0.02,2.05);
PID pidY(kp,ki,kd);
PID pidX(kp,ki,kd);
//Servo servoY;
//Servo servoX;
int servoValueX = 0;
int servoValueY = 0;
Timer timer;
Timer timer2;
Servo motor_fl;
Servo motor_fr;
Servo motor_bl;
Servo motor_br;
float pwm_fl, pwm_fr, pwm_bl, pwm_br;
bool motorEnable = false;

//----------------RC controll
const byte AUX1 = 7;
const byte GEAR = 6;
const byte RUOO = 5;
const byte ELEV = 4;
const byte AILE = 3;
const byte THRO = 2;

float rc_throttle;
float rc_yaw;
float rc_pitch;
float rc_roll;


const unsigned int CR_filterSize = 10;
float rc_throttle_filterBuffer[CR_filterSize];
float rc_yaw_filterBuffer[CR_filterSize];
float rc_pitch_filterBuffer[CR_filterSize];
float rc_roll_filterBuffer[CR_filterSize];

unsigned long rc_throttleStartTime;
bool         rc_throttleState;
bool         rc_throttleLastState;
unsigned long rawTime;

//Button THRO_button(THRO);
//Button RUOO_button(RUOO);
//Button ELEV_button(ELEV);
//Button AILE_button(AILE);
IntervalTimer RC_updateTimer;
void RC_update();

void setTHRO_value(/*unsigned int pulseLength*/);
void ISR_THRO_CHANGE();
volatile bool THRO_pulseState;
volatile unsigned long THRO_pulseStart;
volatile unsigned long THRO_pulseEnd;
void setRUOO_value(/*unsigned int pulseLength*/);
void ISR_RUOO_CHANGE();
volatile bool RUOO_pulseState;
volatile unsigned long RUOO_pulseStart;
volatile unsigned long RUOO_pulseEnd;
void setELEV_value(/*unsigned int pulseLength*/);
void ISR_ELEV_CHANGE();
volatile bool ELEV_pulseState;
volatile unsigned long ELEV_pulseStart;
volatile unsigned long ELEV_pulseEnd;
void setAILE_value(/*unsigned int pulseLength*/);
void ISR_AILE_CHANGE();
volatile bool AILE_pulseState;
volatile unsigned long AILE_pulseStart;
volatile unsigned long AILE_pulseEnd;
//---------------------------

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
Led statusLed(LED_PIN);
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

RH_ASK driver(2000, 24, 23, 0); //RF_receiver
String RF_inputbuffer = "";

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void handleReceiver();
void handleLog();
void getLogData(String &str,String separator = ";");
void getLogDataHeader(String &str,String separator = ";");

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
   // while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    pinMode(8,INPUT); // INT von MPU
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    pinMode(akku_c1_pin,INPUT);
    pinMode(akku_c2_pin,INPUT);
    pinMode(akku_c3_pin,INPUT);

    motor_fl.attach(fl_pin);
    motor_fr.attach(fr_pin);
    motor_bl.attach(bl_pin);
    motor_br.attach(br_pin);

    pinMode(fl_TLM_pin,INPUT);
    pinMode(fr_TLM_pin,INPUT);
    pinMode(bl_TLM_pin,INPUT);
    pinMode(br_TLM_pin,INPUT);

    motor_fl.writeMicroseconds(1000); 
    motor_fr.writeMicroseconds(1000); 
    motor_bl.writeMicroseconds(1000); 
    motor_br.writeMicroseconds(1000); 

    pinMode(AUX1,INPUT);
    pinMode(GEAR,INPUT);
    pinMode(RUOO,INPUT);
    pinMode(ELEV,INPUT);
    pinMode(AILE,INPUT);
    pinMode(THRO,INPUT);

    attachInterrupt(digitalPinToInterrupt(THRO), ISR_THRO_CHANGE, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RUOO), ISR_RUOO_CHANGE, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ELEV), ISR_ELEV_CHANGE, CHANGE);
    attachInterrupt(digitalPinToInterrupt(AILE), ISR_AILE_CHANGE, CHANGE);
    
    
 //   THRO_button.getPulseLength(setTHRO_value);
 //   RUOO_button.getPulseLength(setRUOO_value);
 //   ELEV_button.getPulseLength(setELEV_value);
 //   AILE_button.getPulseLength(setAILE_value);
  
    
    
    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    /*while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again*/

    if (!driver.init())
         Serial.println("init failed");

    if (!SD.begin(chipSelect)) {
      Serial.println("Card failed, or not present");
    // don't do anything more:
      //return;
    }
    logFileTimer.onFinished(handleLog);
    logFileTimer.autoRestart(true);
    logFileTimer.start(logFileUpdateTime);
    
    Timer t1;
    led_green.blinkOn(100);
    while(!t1.start(5000))
    {
      
      led_green.update();
    }
    led_green.blinkOff();

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
   // mpu.setFullScaleAccelRange(2);
   //calibration();
    //Your offsets: -3829 -1921 856 -6  -13 -1
   mpu.setXAccelOffset(-3829);
   mpu.setYAccelOffset(-1921);
   mpu.setZAccelOffset(856);

   mpu.setXGyroOffset(-6);
   mpu.setYGyroOffset(-13);
   mpu.setZGyroOffset(-1);
   
   Serial.println("Xoffset: "+String(mpu.getXAccelOffset()));
   Serial.println("Yoffset: "+String(mpu.getYAccelOffset()));
   Serial.println("Zoffset: "+String(mpu.getZAccelOffset()));
   
   pidX.maximum(500);
   pidX.minimum(-500);
   pidY.maximum(500);
   pidY.minimum(-500);
   pidX.setSpeed(__SPEED_FAST);
   pidY.setSpeed(__SPEED_FAST);
   RC_updateTimer.begin(RC_update,10);

   akkuTimer.onFinished(readAkku);
   akkuTimer.autoRestart(true);
   akkuTimer.start(100);
   Serial.print("]");
   String header;
   getLogDataHeader(header);
   Serial.println(header);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void RC_update()
{
  /*THRO_button.update();
  RUOO_button.update();
  ELEV_button.update();
  AILE_button.update();*/

  handleReceiver();
}


String serialBuffer = "";
char serialChanal = 0;

void loop() {
  akkuTimer.update();
  led_green.update();
  led_yellow.update();
  led_red.update();
  logFileTimer.update();

  setTHRO_value();
  setRUOO_value();
  setELEV_value();
  setAILE_value();
  /*if(THRO_pulseState)
  {
    THRO_pulseStart = pulseIn(THRO,HIGH,2000);
    THRO_pulseState = false;
  }*/

  while (RF_inputbuffer.indexOf("]") != -1)
  {
    String _checksum1 = String(checksum(RF_inputbuffer.substring(0, RF_inputbuffer.indexOf("|"))));
    String _checksum2 = RF_inputbuffer.substring(RF_inputbuffer.indexOf("|") + 1, RF_inputbuffer.indexOf("]"));
    if (_checksum1 != _checksum2)
    {
      //Serial.println("!!! checksum wrong !!!  sender: " + _checksum2 + " teensy: " + _checksum1);
      RF_inputbuffer = RF_inputbuffer.substring(RF_inputbuffer.indexOf("]") + 1);
      break;
    }
   // Serial.println(RF_inputbuffer);

    if (RF_inputbuffer.indexOf("lon") != -1)
    {
      enableLog = true;
      fileIndex = atoi(RF_inputbuffer.substring(3).c_str());
      //Serial.println("fileIndex: " + String(fileIndex));
    } else if (RF_inputbuffer.indexOf("loff") != -1)
    {
      enableLog = false;
    } else if (RF_inputbuffer.indexOf("p") != -1)
    {
      kp = (float)atoi(RF_inputbuffer.substring(1, RF_inputbuffer.indexOf("]")).c_str()) / 10;
      if (kp > 10000 || kp < 0)
      {
        kp = 0;
      }
      pidX.kp(kp);
      pidY.kp(kp);
    } else if (RF_inputbuffer.indexOf("i") != -1)
    {
      ki = (float)atoi(RF_inputbuffer.substring(1, RF_inputbuffer.indexOf("]")).c_str()) / 10000;
      if (ki > 10 || ki < 0)
      {
        ki = 0;
      }
      pidX.ki(ki);
      pidY.ki(ki);
    } else if (RF_inputbuffer.indexOf("d") != -1)
    {
      kd = (float)atoi(RF_inputbuffer.substring(1, RF_inputbuffer.indexOf("]")).c_str()) / 10;
      if (kd > 10000 || kd < 0)
      {
        kd = 0;
      }
      pidX.kd(kd);
      pidY.kd(kd);
    } else if (RF_inputbuffer.indexOf("fi") != -1) //FileInterval
    {
      logFileUpdateTime = atoi(RF_inputbuffer.substring(2, RF_inputbuffer.indexOf("]")).c_str());
      logFileTimer.start(logFileUpdateTime);
    }else if (RF_inputbuffer.indexOf("rstI") != -1) //reset PID.I
    {
      pidX.I(0);
      pidY.I(0);
    }
    RF_inputbuffer = RF_inputbuffer.substring(RF_inputbuffer.indexOf("]") + 1);
  }

  while (Serial.available() != 0)
  {
    char input  = Serial.read();

    // p -> proportional
    // i -> integral
    // d -> derivative
    // s -> serial toggle
    if (input == 'p' || input == 'i' || input == 'd' || input == 's')
    {
      serialChanal = input;
    } else if (input != '\n' && input != ']')
    {
      serialBuffer += input;
    }
    //Serial.println("\""+serialBuffer+"\"");
    if (input == ']')
    {
      switch (serialChanal)
      {
        case 'p':
          {
            kp = (float)atoi(serialBuffer.c_str()) / 10;
            pidX.kp(kp);
            pidY.kp(kp);
            //  Serial.println("new P: "+String(pidX.kp()));
            break;
          }
        case 'i':
          {
            //Serial.println("\""+serialBuffer+"\"");
            ki = (float)atoi(serialBuffer.c_str()) / 1000;
            pidX.ki(ki);
            pidY.ki(ki);
            //  Serial.println("new I: "+String(pidX.ki()));
            break;
          }
        case 'd':
          {
            kd = (float)atoi(serialBuffer.c_str()) / 10;
            pidX.kd(kd);
            pidY.kd(kd);
            // Serial.println("new D: "+String(pidX.kd()));
            break;
          }
        case 's':
          {
            enableSerialDump = !enableSerialDump;
            break;
          }
      }
      Serial.flush();
      serialChanal = 0;
      serialBuffer = "";
    }
  }
  // display real acceleration, adjusted to remove gravity
  elapsedTime = (millis() - lastTime) / 1000.f;
            lastTime = millis();
            //mpu.getRotation(&_rx,&_ry,&_rz);
           /* mpu.getAcceleration(&_ax,&_ay,&_az);
            
            Serial.print(String(_ax));
            Serial.print(";"+String(_ay));
            Serial.print(";"+String(_az));
            Serial.print(";"+String(_rx));
            Serial.print(";"+String(_ry));
            Serial.println(";"+String(_rz));*/

            /*float divisor = 1670.13251783894;
            
            

            float rx = (float)_rx/divisor;
            float ry = (float)_ry/divisor;
            float rz = (float)_rz/divisor;*/
            float ax;
            float ay; 
            float az;

            float rx;
            float ry; 
            float rz;
            getAcceleration(mpu,ax,ay,az);
            getRotationAcceleration(mpu,rx,ry,rz);


            //Serial.println(String(getAngleX())+";"+String(getAngleY()));
            float c1 = 0.1;
            totalAngleX = (1.f-c1)*(totalAngleX - __xAngleOffset + rx * elapsedTime) + c1*ax + __xAngleOffset;
            totalAngleY = (1.f-c1)*(totalAngleY - __yAngleOffset + ry * elapsedTime) + c1*ay + __yAngleOffset;
            

            pidX.input(totalAngleX);
            pidY.input(totalAngleY);

  //------------ only stear if the joystick is out of the deadzone
  if ((50 - __RC_deadzone) > rc_pitch || rc_pitch > (50 + __RC_deadzone))
  {
    pidY.expected(_map<float, float>(rc_pitch, (float)0, (float)100, __maxRC_angleRange, -__maxRC_angleRange) ); //  achse invertiert
  } else
  {
    pidY.expected(0);
  }
  if ((50 - __RC_deadzone) > rc_roll || rc_roll > (50 + __RC_deadzone))
  {
    pidX.expected(_map<float, float>(rc_roll, (float)0, (float)100, -__maxRC_angleRange, __maxRC_angleRange));
  } else
            {
              pidX.expected(0);
            }
  //-----------

            if(sqrt(pow(pidX.error(),2)) < 0.05)
            {
              pidX.enable_I();
            }else if(sqrt(pow(pidX.error(),2)) > 0.1)
            {
              pidX.disable_I();
              pidX.I(0);
            }
            if(sqrt(pow(pidY.error(),2)) < 0.05)
            {
              pidY.enable_I();
            }else if(sqrt(pow(pidY.error(),2)) > 0.1)
            {
              pidY.disable_I();
              pidY.I(0);
            }
        
        
            pidX.update();
            pidY.update();

            float pid_multiplyer = 1;
            if(timer.start(1))
            {
             /* servoValueX -= (int)pidX.output();
              if(servoValueX < 0){servoValueX = 0;}
              else if(servoValueX > 180){servoValueX = 180;}
              
              servoValueY += (int)pidY.output();
              if(servoValueY < 0){servoValueY = 0;}
              else if(servoValueY > 180){servoValueY = 180;}*/
              float rc_throttle_factor  = 0.01;
              float rc_pitch_factor     = 2;
              float rc_yaw_factor       = 2;
              float rc_roll_factor      = 2;

              if(rc_throttle >= __motorEnablePoint && !motorEnable)
              {
                motorEnable = true;
              }else if(rc_throttle <= __motorDisablePoint && motorEnable)
              {
                motorEnable = false;
              }
              if(!motorEnable)//15%
              {
                pwm_fl = 0;
                pwm_fr = 0;
                pwm_bl = 0;
                pwm_br = 0;
              }else
              {
 //     pwm_fl = __minThrottle + (0 + rc_throttle_factor * rc_throttle) * (Throttle + /*rc_roll_factor * (50.f - rc_roll) + rc_pitch_factor * (50.f - rc_pitch)*/ + rc_yaw_factor * (rc_yaw - 50.f) + ( pidX.output() - pidY.output()) * pid_multiplyer);
 //     pwm_fr = __minThrottle + (0 + rc_throttle_factor * rc_throttle) * (Throttle + /*rc_roll_factor * (rc_roll - 50.f) + rc_pitch_factor * (50.f - rc_pitch)*/ + rc_yaw_factor * (50.f - rc_yaw) + (-pidX.output() - pidY.output()) * pid_multiplyer);
 //     pwm_bl = __minThrottle + (0 + rc_throttle_factor * rc_throttle) * (Throttle + /*rc_roll_factor * (50.f - rc_roll) + rc_pitch_factor * (rc_pitch - 50.f)*/ + rc_yaw_factor * (50.f - rc_yaw) + ( pidX.output() + pidY.output()) * pid_multiplyer);
 //     pwm_br = __minThrottle + (0 + rc_throttle_factor * rc_throttle) * (Throttle + /*rc_roll_factor * (rc_roll - 50.f) + rc_pitch_factor * (rc_pitch - 50.f)*/ + rc_yaw_factor * (rc_yaw - 50.f) + (-pidX.output() + pidY.output()) * pid_multiplyer);

      //pwm_fl = _map<float, float>(rc_throttle * (1 +( pidX.output() - pidY.output())), (float)0, (float)100, __minThrottle, __maxThrottle);
  /*    pwm_fl = rc_throttle * (__maxThrottle - __minThrottle + 20*( pidX.output() - pidY.output()))/100  + __minThrottle;
      pwm_fr = rc_throttle * (__maxThrottle - __minThrottle + 20*(-pidX.output() - pidY.output()))/100  + __minThrottle;
      pwm_bl = rc_throttle * (__maxThrottle - __minThrottle + 20*( pidX.output() + pidY.output()))/100  + __minThrottle;
      pwm_br = rc_throttle * (__maxThrottle - __minThrottle + 20*(-pidX.output() + pidY.output()))/100  + __minThrottle;
      */
      float throttleRange = ((__maxThrottle - __minThrottle)*9)/10;
      pwm_fl = rc_throttle * (throttleRange + ( pidX.output() - pidY.output()))/100 + __minThrottle;
      pwm_fr = rc_throttle * (throttleRange + (-pidX.output() - pidY.output()))/100 + __minThrottle;
      pwm_bl = rc_throttle * (throttleRange + ( pidX.output() + pidY.output()))/100 + __minThrottle;
      pwm_br = rc_throttle * (throttleRange + (-pidX.output() + pidY.output()))/100 + __minThrottle;
              }
              if (timer2.start(30))
              {
                /*
                Serial.print(String(totalAngleX)+";"+String(totalAngleY));
                Serial.print("\tpidX: "+String(pidX.output()));
                Serial.print("\tpidY: "+String(pidY.output()));
                Serial.print("\tfl: "+String(pwm_fl-throttle));
                Serial.print("\tfr: "+String(pwm_fr-throttle));
                Serial.print("\tbl: "+String(pwm_bl-throttle));
                Serial.print("\tbr: "+String(pwm_br-throttle));
                Serial.print("\tthrottle: "+String(rc_throttle)+";"+String(rc_yaw)+";"+String(rc_pitch)+";"+String(rc_roll)+";"+String(digitalRead(AUX1))+";"+String(digitalRead(GEAR)));
                */
              /*  const String separator = ";\t";
                Serial.print(String(voltage_c1)+separator+String(voltage_c2)+separator+String(voltage_c3)+";"+separator);
                Serial.print(String(totalAngleX)+separator+String(totalAngleY));
                Serial.print(";"+separator+String(pidX.output()));
                Serial.print(separator+String(pidY.output()));
                Serial.print(";"+separator+String(pwm_fl-__minThrottle));
                Serial.print(separator+String(pwm_fr-__minThrottle));
                Serial.print(separator+String(pwm_bl-__minThrottle));
                Serial.print(separator+String(pwm_br-__minThrottle));
                Serial.print(";"+separator+String(rc_throttle)+separator+String(rc_yaw)+separator+String(rc_pitch)+separator+String(rc_roll)+separator+String(digitalRead(AUX1))+separator+String(digitalRead(GEAR)));
                Serial.print(";"+separator+String(kp)+separator+String(ki)+separator+String(kd)+separator);
                //Serial.println();
                Serial.println(";");*/
                if(enableSerialDump)
                {
                  String dbg;
                  getLogData(dbg,";\t");
                  Serial.println(dbg);
                }

               /* Serial.print(String(0 + rc_throttle_factor * rc_throttle)+";");
                Serial.print(String(throttle + rc_roll_factor * (50.f - rc_roll))+";");
                Serial.print(String(rc_pitch_factor * (50.f - rc_pitch))+";");
                Serial.print(String(rc_yaw_factor * (rc_yaw - 50.f))+";");
                Serial.print(String(( pidX.output() - pidY.output()) * pid_multiplyer)+";;");
                Serial.println(pwm_fl);*/

                //handleLog();
              }

              if(pwm_fl < __minThrottle){pwm_fl = __minThrottle;}else if(pwm_fl > __maxThrottle){pwm_fl = __maxThrottle;}
              if(pwm_fr < __minThrottle){pwm_fr = __minThrottle;}else if(pwm_fr > __maxThrottle){pwm_fr = __maxThrottle;}
              if(pwm_bl < __minThrottle){pwm_bl = __minThrottle;}else if(pwm_bl > __maxThrottle){pwm_bl = __maxThrottle;}
              if(pwm_br < __minThrottle){pwm_br = __minThrottle;}else if(pwm_br > __maxThrottle){pwm_br = __maxThrottle;}
              
              motor_fl.writeMicroseconds(pwm_fl);
              motor_fr.writeMicroseconds(pwm_fr);
              motor_bl.writeMicroseconds(pwm_bl);
              motor_br.writeMicroseconds(pwm_br);
  
             // motor_br.writeMicroseconds(map(analogRead(A0),0,1023,1000,2000));
              
            
            }
            
          
            
          /*  mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);*/
}

void getAcceleration(MPU6050 &accel,float &ax,float &ay, float &az)
{
  int16_t _ax;
  int16_t _ay;
  int16_t _az;

  
  accel.getAcceleration(&_ax,&_ay,&_az);
  
  float divisor = 16384;//1670.13251783894;
  ax = (float)_ax/divisor;
  ay = (float)_ay/divisor;
  az = (float)_az/divisor; 
}
void getRotationAcceleration(MPU6050 &accel,float &rx,float &ry,float &rz)
{
  int16_t _rx;
  int16_t _ry;
  int16_t _rz;
  accel.getRotation(&_rx,&_ry,&_rz);
  float divisor = 16384;//1670.13251783894;
  rx = (float)_rx/divisor;
  ry = (float)_ry/divisor;
  rz = (float)_rz/divisor;
}
float getAngleX()
{
  float ax;
  float ay;
  float az;
  getRotationAcceleration(mpu,ax,ay,az);
  return 114.5915590261648*atan(ax/sqrt(pow(ay,2)+pow(az,2)));
}
float getAngleY()
{
  float ax;
  float ay;
  float az;
  getRotationAcceleration(mpu,ax,ay,az);
  return 114.5915590261648*atan(ay/sqrt(pow(ax,2)+pow(az,2)));
}

void ISR_THRO_CHANGE()
{
  if(digitalRead(THRO))
  {
    THRO_pulseStart = micros();
  }
  else
  {
    THRO_pulseEnd = micros();
    THRO_pulseState = true;
  }
}
void ISR_RUOO_CHANGE()
{
  if(digitalRead(RUOO))
  {
    RUOO_pulseStart = micros();
  }
  else
  {
    RUOO_pulseEnd = micros();
    RUOO_pulseState = true;
  }
}
void ISR_ELEV_CHANGE()
{
  if(digitalRead(ELEV))
  {
    ELEV_pulseStart = micros();
  }
  else
  {
    ELEV_pulseEnd = micros();
    ELEV_pulseState = true;
  }
}
void ISR_AILE_CHANGE()
{
  if(digitalRead(AILE))
  {
    AILE_pulseStart = micros();
  }
  else
  {
    AILE_pulseEnd = micros();
    AILE_pulseState = true;
  }
}
void setTHRO_value(/*unsigned int pulseLength*/)
{
  if(!THRO_pulseState)
    return;

  unsigned int pulseLength = THRO_pulseEnd - THRO_pulseStart;
  if(pulseLength < MIN_PULSE_LENGTH || pulseLength > MAX_PULSE_LENGTH)
  {
    Serial.println("*** THRO pulseLength: "+ String(pulseLength));
    return;
  }

  float filterFactor = 0.02;
  
  float last_rc_throttle = rc_throttle;
  rc_throttle = _map<float,float>((float)pulseLength,(float)1000,(float)1900,0,100);
  filterFactor = filterFactor * sqrt(pow(rc_throttle-last_rc_throttle,2));
    
  rc_throttle = (1-filterFactor)*last_rc_throttle + filterFactor * rc_throttle;
  //Serial.println(String(last_rc_throttle) + "\t" + String(filterFactor) +"\t"+String(rc_throttle));
  /*rc_throttle = 0;
  for(unsigned int i=1; i<CR_filterSize; i++)
  {
    rc_throttle_filterBuffer[i-1] = rc_throttle_filterBuffer[i];
    rc_throttle += rc_throttle_filterBuffer[i-1];
  }
  rc_throttle_filterBuffer[CR_filterSize-1] = (float)map(pulseLength,1000,1900,0,100);
  rc_throttle += rc_throttle_filterBuffer[CR_filterSize-1];
  rc_throttle /= CR_filterSize;*/

  
  //if(debugTimer.start(10))
  {
 // Serial.println(String(pulseLength) +";" +String(rc_throttle));
  }
  THRO_pulseState = false;
}
void setRUOO_value(/*unsigned int pulseLength*/)
{
  if(!RUOO_pulseState)
    return;

  unsigned int pulseLength = RUOO_pulseEnd - RUOO_pulseStart;
  if(pulseLength < MIN_PULSE_LENGTH || pulseLength > MAX_PULSE_LENGTH)
  {
    //Serial.println("*** RUOO pulseLength: "+ String(pulseLength));
    return;
  }
  float filterFactor = 0.01;
  
  float last_rc_yaw = rc_yaw;
  //rc_yaw = ((float)map(pulseLength,1000,1900,0,100) + last_rc_yaw)/2;
  rc_yaw = _map<float,float>((float)pulseLength,(float)1000,(float)1900,0,100);
  filterFactor = filterFactor * sqrt(pow(rc_yaw-last_rc_yaw,2));
  rc_yaw = (1-filterFactor)*last_rc_yaw + filterFactor * rc_yaw;

  RUOO_pulseState = false;
  //Serial.println(String(pulseLength) +" " +String(rc_yaw));
}
void setELEV_value(/*unsigned int pulseLength*/)
{
  if(!ELEV_pulseState)
    return;

  unsigned int pulseLength = ELEV_pulseEnd - ELEV_pulseStart;
  if(pulseLength < MIN_PULSE_LENGTH || pulseLength > MAX_PULSE_LENGTH)
  {
    //Serial.println("*** ELEV pulseLength: "+ String(pulseLength));
    return;
  }
  float filterFactor = 0.01;
  float last_rc_pitch = rc_pitch;
  //rc_pitch = ((float)map(pulseLength,1000,1900,0,100) + last_rc_pitch)/2;
  rc_pitch = _map<float,float>((float)pulseLength,(float)1000,(float)1900,0,100);
  filterFactor = filterFactor * sqrt(pow(rc_pitch-last_rc_pitch,2));
  rc_pitch = (1-filterFactor)*last_rc_pitch + filterFactor * rc_pitch;

  ELEV_pulseState = false;
}
void setAILE_value(/*unsigned int pulseLength*/)
{
  if(!AILE_pulseState)
    return;

  unsigned int pulseLength = AILE_pulseEnd - AILE_pulseStart;
  if(pulseLength < MIN_PULSE_LENGTH || pulseLength > MAX_PULSE_LENGTH)
  {
    //Serial.println("*** AILE pulseLength: "+ String(pulseLength));
    return;
  }
  float filterFactor = 0.01;
  float last_rc_roll = rc_roll;
  //rc_roll = ((float)map(pulseLength,1000,1900,0,100) + last_rc_roll)/2;
  rc_roll = _map<float,float>((float)pulseLength,(float)1000,(float)1900,0,100);
  filterFactor = filterFactor * sqrt(pow(rc_roll-last_rc_roll,2));
  rc_roll = (1-filterFactor)*last_rc_roll + filterFactor * rc_roll;

  AILE_pulseState = false;
}
void readAkku()
{
  float minVoltagePerCell = 3.5;
  float maxVoltagePerCell = 4.2;

  float filterFactor = 0.1;
  
  float lastVoltage_c1 = voltage_c1;
  float lastVoltage_c2 = voltage_c2 + voltage_c1;
  float lastVoltage_c3 = voltage_c3 + voltage_c2 + voltage_c1;
  
   voltage_c1 = ((float)1-filterFactor) * lastVoltage_c1 + filterFactor * ((float)analogRead(akku_c1_pin)*0.007289-0.973043)/*1.4255319*//*1.605486083098023*0.0032258064516129*/; //4.7k/6.7k
   voltage_c2 = ((float)1-filterFactor) * lastVoltage_c2 + filterFactor * ((float)analogRead(akku_c2_pin)*0.014496-1.60205)/*3.35*//*3.352990732940185     *0.0032258064516129*/; //2k/6.7k
   voltage_c3 = ((float)1-filterFactor) * lastVoltage_c3 + filterFactor * ((float)analogRead(akku_c3_pin)*0.0231-4.271)/*4.4*//*4.435783221974759      *0.0032258064516129*/; //2k/8.8k
   //Serial.println("C1: "+String(voltage_c1)+"V\tc2: "+String(voltage_c2)+"V\tc3: "+String(voltage_c3)+"V");
   
   /*if(voltage_c3 <= 0.1)
   {
      voltagePercent = 100*(voltage_c2-2*minVoltagePerCell)/(2*(maxVoltagePerCell-minVoltagePerCell)); //8.4V für 2S
   }else*/
   {
      voltagePercent = 100*(voltage_c3-3*minVoltagePerCell)/(3*(maxVoltagePerCell-minVoltagePerCell)); //12.6V für 3S
      voltage_c3 -= voltage_c2;
   }
   voltage_c2 -= voltage_c1;
  // Serial.println("C1: "+String(voltage_c1)+"V\tc2: "+String(voltage_c2)+"V\tc3: "+String(voltage_c3)+"V\t"+String(voltagePercent)+"%");
   //Serial.println(String(voltage_c1)+";"+String(voltage_c2)+";"+String(voltage_c3)+";"+String(voltagePercent));
   
   
   
   if(voltagePercent < 25)
   {
     led_red.on();
     led_green.off();
     led_yellow.off();
   }else if(voltagePercent < 70)
   {
     led_yellow.on();
     led_green.off();
     led_red.off();
   }else
   {
     led_green.on();
     led_yellow.off();
     led_red.off();
   }
}
void handleReceiver()
{
  uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  uint8_t buflen = sizeof(buf);

  if (driver.recv(buf, &buflen)) // Non-blocking
  {
    statusLed.toggle();
    for (unsigned int a = 0; a < buflen; a++)
    {
      RF_inputbuffer += (char)buf[a];
    }

    
  // Message with a good checksum received, dump it.
  //driver.printBuffer("Got:", buf, buflen);
  }
}
void handleLog()
{
  if(!enableLog)
    return;

  bool printHeader = false;
  String filename = "log"+String(fileIndex)+".csv";
  String logData;
  if(!SD.exists(filename.c_str()))
  {
    printHeader = true;
  }

  File dataFile = SD.open(filename.c_str(), FILE_WRITE);
  if (!dataFile)
  {
    Serial.println("error opening: "+String(filename));
    if (!SD.begin(chipSelect)) {
      Serial.println("Card failed, or not present");
    // don't do anything more:
      //return;
    }
    
  }
  if(printHeader)
  {
    getLogDataHeader(logData);
    dataFile.println(logData);
  }

  
  /*dataFile.print(String(voltage_c1) + ";" + String(voltage_c2) + ";" + String(voltage_c3) + ";;");
  dataFile.print(String(totalAngleX) + ";" + String(totalAngleY));
  dataFile.print(";;" + String(pidX.output()));
  dataFile.print("; " + String(pidY.output()));
  dataFile.print(";;" + String(pwm_fl - __minThrottle));
  dataFile.print(";" + String(pwm_fr - __minThrottle));
  dataFile.print(";" + String(pwm_bl - __minThrottle));
  dataFile.print(";" + String(pwm_br - __minThrottle));
  dataFile.print(";;" + String(rc_throttle) + ";" + String(rc_yaw) + ";" + String(rc_pitch) + ";" + String(rc_roll) + ";" + String(digitalRead(AUX1)) + ";" + String(digitalRead(GEAR)));
  dataFile.print(";;" + String(kp) + ";" + String(ki) + ";" + String(kd) + ";");
  dataFile.println();*/

  
  getLogData(logData);
  dataFile.println(logData);
  dataFile.close();
}
void getLogData(String &str,String separator)
{
  str  = String(pidX.elapsedTime())+ separator+separator;
  str += String(voltage_c1)     + separator+String(voltage_c2)   + separator+String(voltage_c3)+separator+separator;
  str += String(totalAngleX)    + separator+String(totalAngleY)  + separator+separator;
  str += String(pidX.kp())      + separator+String(pidX.ki(),4)    + separator+String(pidX.kd())    + separator+String(pidX.P())    + separator+String(pidX.I())    + separator+String(pidX.D())  + separator+String(pidX.output())    + separator+separator;        // PID x
  str += String(pidY.kp())      + separator+String(pidY.ki())    + separator+String(pidY.kd())    + separator+String(pidY.P())    + separator+String(pidY.I())    + separator+String(pidY.D())  + separator+String(pidY.output())    + separator+separator;        // PID Y
  str += String(pwm_fl-__minThrottle) + separator + String(pwm_fr-__minThrottle) + separator + String(pwm_bl-__minThrottle) + separator + String(pwm_br-__minThrottle) +separator+separator;
  str += String(rc_throttle)    + separator+String(rc_yaw)       + separator+String(rc_pitch)     + separator+String(rc_roll)     + separator+separator;

  
}
void getLogDataHeader(String &str,String separator)
{
  str  = "time"+separator+separator;
  str += "VC1"+separator+"VC2"+separator+"VC3"+separator+separator;//totalAngleX"+separator+"totalAngleY;;pidX.kp"+separator+"pidX.ki;pidX.kd;pidX.P;pidX.I;pidX.D,pidX.output;pidY.output;;fl;fr;bl;br;;rc_throttle;rc_yaw;rc_pitch;rc_roll;rc_Aux1;rc_gear;;kp;ki"+separator+"kd;;";
  str += "totalAngleX"+separator+"totalAngleY;;";
  str += "pidX.kp"+separator+"pidX.ki"+separator+"pidX.kd"+separator+"pidX.P"+separator+"pidX.I"+separator+"pidX.D"+separator+"pidX.output"+separator+separator;
  str += "pidY.kp"+separator+"pidY.ki"+separator+"pidY.kd"+separator+"pidY.P"+separator+"pidY.I"+separator+"pidY.D"+separator+"pidY.output"+separator+separator;
  str += "FL"+separator+"FR"+separator+"BL"+separator+"BR"+separator+separator;
  str += "RC_throttle"+separator+"RC_yaw"+separator+"RC_pitch"+separator+"RC_roll"+separator+separator;

  
}
template<typename T, typename T2>
inline T _map(T2 val, T2 in_min, T2 in_max, T out_min, T out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
int checksum(String text)
{
  int sum = 0;
  //Serial.println("chgecksum: \"");
  for(unsigned int a=0; a<text.length(); a++)
  {
    sum += text.charAt(a);
   // Serial.print(text.charAt(a));
    //s++;
  }
  //Serial.println("\" is: "+String(sum));
  return sum;
}
