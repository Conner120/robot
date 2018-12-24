//lib
#include <Arduino.h>
#include <SPI.h>
#if not defined(_VARIANT_ARDUINO_DUE_X_) && not defined(_VARIANT_ARDUINO_ZERO_)
#endif
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>
#include <EnableInterrupt.h>

//declare sevro for camera
Servo camPan;
//BNO055 declarations and vars
boolean bnoGood = false;
Adafruit_BNO055 bno = Adafruit_BNO055();

// declare RC
#define SERIAL_PORT_SPEED 115200
#define RC_NUM_CHANNELS 6
#define RC_CH1 0
#define RC_CH2 1
#define RC_CH3 2
#define RC_CH4 3
#define RC_CH5 4
#define RC_CH6 5
#define RC_CH1_INPUT  A7
#define RC_CH2_INPUT  A10
#define RC_CH3_INPUT  A9
#define RC_CH4_INPUT  A11
#define RC_CH5_INPUT  A8
#define RC_CH6_INPUT  A5
uint16_t last_update_time;
uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
uint16_t rc_prev[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];
bool radioOFF = true;
//declare light pins
int floodLights[4] = {22, 29, 31, 35};
bool emergancy = true;
int emergancyLight[10] = {23, 24, 25, 26, 27, 28, 30, 32, 33, 34};
//motor controll
#define BRAKEVCC 0
#define CW 1
#define CCW 2
#define BRAKEGND 3
#define CS_THRESHOLD 100
int inApin[2] = {7, 4}; // INA: Clockwise input
int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {5, 6}; // PWM input
int cspin[2] = {2, 3};  // CS: Current sense ANALOG input
int enpin[2] = {0, 1};  // EN: Status of switches output (Analog pin)
//bluetooth declarations
#define FACTORYRESET_ENABLE 0
//#define BUFSIZE                        48   // Size of the read buffer for incoming data
#define MINIMUM_FIRMWARE_VERSION "0.6.6"
#define MODE_LED_BEHAVIOUR "MODE"
#define VERBOSE_MODE true
#define BLUEFRUIT_SWUART_RXD_PIN 9
#define BLUEFRUIT_SWUART_TXD_PIN 10
#define BLUEFRUIT_UART_CTS_PIN 11
#define BLUEFRUIT_UART_RTS_PIN -1
#ifdef Serial1
#define BLUEFRUIT_HWSERIAL_NAME Serial1
#endif
#define BLUEFRUIT_UART_MODE_PIN 12 // Set to -1 if unused
#define BLUEFRUIT_SPI_CS 8
#define BLUEFRUIT_SPI_IRQ 7
#define BLUEFRUIT_SPI_RST 4 // Optional but recommended, set to -1 if unused
#ifdef __arm__
#define bluefruitSS Serial1
#else
//SoftwareSerial bluefruitSS = SoftwareSerial(51, 50);
#endif
Adafruit_BluefruitLE_UART ble(Serial3, 52);
double distanceX = 0;
double distanceY = 0;
double distanceTotal = 0;
/*
  setup function
  using init functions defined later in code to reduce clutter
*/
//communication
void sendMessage(String msg)
{
  Serial.println(msg);
}
void subSystemReport(String name, String status)
{
  sendMessage("| " + name + "\t| " + status + "\t\t\t\t|\n");
}
//Lighting
void frontFloodLights(bool on)
{
  if (on)
  {

    digitalWrite(floodLights[0], HIGH);
    digitalWrite(floodLights[1], HIGH);
  }
  else
  {
    digitalWrite(floodLights[0], LOW);
    digitalWrite(floodLights[1], LOW);
  }
}

void rearFloodLights(bool on)
{
  if (on)
  {
    digitalWrite(floodLights[2], HIGH);
    digitalWrite(floodLights[3], HIGH);
  }
  else
  {
    digitalWrite(floodLights[2], LOW);
    digitalWrite(floodLights[3], LOW);
  }
}
//RC functions
void rc_read_values()
{
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}
bool red = false;
void emer() {
  if (emergancy) {
    if ((millis() % 375) == 0) {
      for (int i = 0; i < 10; i++) {
        int on = random(0, 1);
        if (on == 1)
          digitalWrite(emergancyLight[i], HIGH);
        if (on == 0)
          digitalWrite(emergancyLight[i], LOW);
      }
    }
  } else {
    for (int i = 0; i < 10; i++) {
      digitalWrite(emergancyLight[i], LOW);
    }

  }
}
void calc_input(uint8_t channel, uint8_t input_pin)
{
  radioOFF = false;
  if (digitalRead(input_pin) == HIGH)
  {
    rc_start[channel] = micros();
  }
  else
  {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}
void calc_ch1()
{
  calc_input(RC_CH1, RC_CH1_INPUT);
  last_update_time = millis();
}
void calc_ch2()
{
  calc_input(RC_CH2, RC_CH2_INPUT);
  last_update_time = millis();
}
void calc_ch3()
{
  calc_input(RC_CH3, RC_CH3_INPUT);
  last_update_time = millis();
}
void calc_ch4()
{
  calc_input(RC_CH4, RC_CH4_INPUT);
  last_update_time = millis();
}
void calc_ch5()
{
  calc_input(RC_CH5, RC_CH5_INPUT);
  last_update_time = millis();
}
void calc_ch6()
{
  last_update_time = millis();
  calc_input(RC_CH6, RC_CH6_INPUT);
}

boolean changeInController()
{
  for (int i = 0; i < RC_NUM_CHANNELS; i++)
  {
    if (rc_values[i] != rc_prev[i])
    {
      for (int a = 0; a < RC_NUM_CHANNELS; a++)
      {
        rc_prev[a] = rc_values[a];
      }
      return true;
    }
  }
  return false;
}
void initRCComms() {
  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);
  pinMode(RC_CH5_INPUT, INPUT);
  pinMode(RC_CH6_INPUT, INPUT);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);
  enableInterrupt(RC_CH5_INPUT, calc_ch5, CHANGE);
  enableInterrupt(RC_CH6_INPUT, calc_ch6, CHANGE);
}
//MAIN drive functions for RC Controll
void RCDrive() {

}


//motor operations functions
void motorgo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor <= 1)
  {
    if (direct <= 4)
    {
      // Set inA[motor]
      if (direct <= 1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct == 0) || (direct == 2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}
void motorOff(int motor)
{
  // Initialize braked
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  analogWrite(pwmpin[motor], 0);
}

void stop()
{

  //Serial.println(millis());
  motorgo(0, BRAKEGND, 0);
  motorgo(1, BRAKEGND, 0);
}
//initalizer functions
void initBNO()
{
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    bnoGood = false;
    Serial.println("BNO055.......Failed");
  }
  else
  {
    Serial.println("BNO055.......Good");
    bnoGood = true;
    int8_t temp = bno.getTemp();
    Serial.print("Current Temperature: ");
    Serial.print(temp);
    Serial.println(" C");
    Serial.println("");

    bno.setExtCrystalUse(true);
  }
}
//BNO055 functions
imu::Vector<3> getEuler()
{
  return bno.getVector(Adafruit_BNO055::VECTOR_EULER);
}
imu::Quaternion getQuat()
{
  return bno.getQuat();
}

void avoidance()
{
}
void drive() {
  if (changeInController) {
    //    Serial.print(rc_values[RC_CH1]); Serial.print(",");
    //    Serial.print(rc_values[RC_CH2]); Serial.print(",");
    //    Serial.print(rc_values[RC_CH3]); Serial.print(",");
    //    Serial.print(rc_values[RC_CH4]); Serial.print(",");
    //    Serial.print(rc_values[RC_CH5]); Serial.print(",");
    //    Serial.print(rc_values[RC_CH6]); Serial.println(",");
    int ver = rc_values[RC_CH2];
    int side = rc_values[RC_CH1];
    int maxSp = 135;
    int throt = map(rc_values[RC_CH3], 1004, 1985, 17, 200);
    maxSp = throt;
    ver = map(ver, 990, 1986, -1 * maxSp, maxSp);
    side = map(side, 996, 1988, -1 * maxSp, maxSp);
    int camAng = map(rc_values[RC_CH5], 990, 1988, 30, 160);
    int left = 0;
    int right = 0;
    if (ver > 10) {
      //forward
      if (side > 6) {
        //forward right
        left = int(ver);
        right = int(ver - .75 * side);
        motorgo(1, 1, ver);
        motorgo(0, 2, right);
      } else if (side < -6) {
        //forward left
        left = ver - .75 * side;
        motorgo(1, 1, ver - .75 * side);
        right = ver;
        motorgo(0, 2, ver);
      } else {
        //forward
        left = ver;
        motorgo(0, 2, ver + 10);
        right = ver;
        motorgo(1, 1, ver);
      }
    } else if (ver < -10) {
      //backward
      if (side > 6) {
        //backward left
        left = int(ver);
        motorgo(1, 1, abs(ver));
        right = int(ver + (side * .75));
        motorgo(0, 1, abs(ver - (side * .75)));
      } else if (side < -6) {
        //backward right
        left = ver - .75 * side;
        motorgo(0, 1, abs(ver - 10));
        motorgo(1, 1, abs(ver - (.75 * side)));
        right = ver;
      } else {
        //full reverse
        Serial.println("full reverse");
        left = ver;
        motorgo(0, 1, abs(ver));
        right = ver;
        motorgo(1, 1, abs(ver));
      }
    } else {
      Serial.println(side);
      if (side > 10) {
        //full right turn
        right = -1 * side;
        motorgo(1, 1, side);
        left = side;
        motorgo(0, 2, side);
      } else if (side < -10) {
        //full left turn
        Serial.println("full left");
        motorgo(1, 2, abs(side));
        motorgo(0, 1, abs(side));
      } else {
        //center
        motorgo(0, 0, 0);
        motorgo(1, 0, 0);
      }
    }
    //    Serial.println(ver);
    //    Serial.println(side);
    //    Serial.println(right);
    //    Serial.println(left);
    if (rc_values[RC_CH5] > 1200) {
    } else {
    }
    if (rc_values[RC_CH6] > 1400) {
      digitalWrite(49, HIGH);
    } else {
      digitalWrite(49, LOW);
    }

  }

}
void setup()
{
  uint8_t system, gyro, accel, mag = 0;
  initRCComms();
  Serial.begin(115200);
  Serial.print("Conner Robot System Startup");
  //initalize flood pins
  for (int i = 0; i < 4; i++)
    pinMode(floodLights[i], OUTPUT);
  //initalize emergancy floodLights
  for (int i = 0; i < 10; i++)
    pinMode(emergancyLight[i], OUTPUT);
  //init bno 9dof sensor
  //  initBNO();

}

/*
  main loop
*/
void loop()
{
  if (!radioOFF) {
    drive();
  } else {
    stop();
  }
  emer();
  long startTime = micros();
  // for (int i = 0; i < 10; i++)
  //     digitalWrite(emergancyLights[i], HIGH);
  rearFloodLights(true);
  frontFloodLights(true);
  //  if (bnoGood)
  //  {
  ////    bno.getCalibration(&system, &gyro, &accel, &mag);
  //    Serial.print("CALIBRATION: Sys=");
  //    Serial.print(system, DEC);
  //    Serial.print(" Gyro=");
  //    Serial.print(gyro, DEC);
  //    Serial.print(" Accel=");
  //    Serial.print(accel, DEC);
  //    Serial.print(" Mag=");
  //    Serial.println(mag, DEC);
  //  }
  rc_read_values();
  if (((rc_start[RC_CH1] + 25000) < micros()) == 1) {
    rc_values[RC_CH1] = 0;
    rc_values[RC_CH2] = 0;
    rc_values[RC_CH3] = 0;
    rc_values[RC_CH4] = 0;
    rc_values[RC_CH5] = 0;
    rc_values[RC_CH6] = 0;
    radioOFF = true;
  }
  Serial.println(micros() - startTime);
}
