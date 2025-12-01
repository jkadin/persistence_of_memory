#include <Adafruit_DotStar.h>
#include <SPI.h>
#include "SparkFun_LIS331.h"
#include <Wire.h>
#include <VescUart.h>
#include <AlfredoCRSF.h>
#include <EEPROM.h>
#include <FastCRC.h>

#define NUMPIXELS 25

LIS331 xl;
VescUart UART;
AlfredoCRSF CRSF;

Adafruit_DotStar strip(NUMPIXELS, DOTSTAR_BGR);

const unsigned long EEPROM_MAGIC_NUMBER = 0xDECAFBAD;

const uint32_t red = 0xFF0000;
const uint32_t green = 0x00FF00;
const uint32_t blue = 0x0000FF;
const uint32_t black = 0x000000;
const uint32_t yellow = 0xFFFF00;
const uint32_t purple = 0x800080;
const uint32_t white = 0xFFFFFF;

const int XL_MAX = 400;
const int MAX_BRIGHTNESS = 250;

const float MAX_RADS = 2 * M_PI;

const long animInterval = 1000;

float wheelRPML = 0, wheelRPMR = 0, botRPMFromMotor = 0, botRPMFromXl = 0, wheelVelocityMpS = 0;
const float wheelRadiusMm = 123.825;
unsigned long startMicros = 0, rpmMicros = 0, durationMicros = 0, lastAnim = 0;
float headerOffset = 0;
int16_t x, y, z;
float xAvg = 0, yAvg = 0, zAvg = 0;
float scaledThrottle = 0;
float avgWheelRPM = 0;
float distanceFromCenterOffset = 0;

unsigned long timerMicros = 0;
int loopCounter = 0;
int animIndex = 0;

int numConfigs = 0;
struct accelConfig {
      float g;
      float rpm;
    };

accelConfig accelConfigs[50];

float channelToPercent(int channel, bool reversible=false);

void setup() {
  Serial.begin(9600); // Teensy always uses USB Serial speed, this speed is ignored
  Serial.println("Setup started");

  // Check EEPROM
  unsigned long magicNumCheck;
  FastCRC8 CRC8;
  uint8_t crc, eepromCrc;
  int eepromIndex = 0;
  accelConfig eepromConfigs[50];
  crc = CRC8.smbus((uint8_t*)&accelConfigs, sizeof(accelConfigs));

  EEPROM.get(eepromIndex, magicNumCheck);
  if (magicNumCheck != EEPROM_MAGIC_NUMBER) {
    EEPROM.put(eepromIndex, EEPROM_MAGIC_NUMBER);
    eepromIndex += sizeof(EEPROM_MAGIC_NUMBER);
    EEPROM.put(eepromIndex, 0);
    eepromIndex += sizeof(0);
    EEPROM.put(eepromIndex, accelConfigs);
    eepromIndex += sizeof(accelConfigs);
    EEPROM.put(eepromIndex, crc);
  } else {
    Serial.println("Found correct magic number");
    eepromIndex += sizeof(EEPROM_MAGIC_NUMBER);
    EEPROM.get(eepromIndex, numConfigs);
    eepromIndex += sizeof(numConfigs);
    EEPROM.get(eepromIndex, eepromConfigs);
    eepromIndex += sizeof(eepromConfigs);
    EEPROM.get(eepromIndex, eepromCrc);
    eepromCrc = CRC8.smbus((uint8_t*)&eepromConfigs, sizeof(eepromConfigs));
    if (eepromCrc == crc) {
      memcpy(accelConfigs, eepromConfigs, sizeof(eepromConfigs));
    } else {
    }
  }

  strip.begin(); // Initialize pins for output
  strip.setBrightness(5);
  strip.show();  // Turn all LEDs off ASAP

  Wire.begin();
  xl.setI2CAddr(0x19);    // This MUST be called BEFORE .begin() so 
                          //  .begin() can communicate with the chip
  xl.begin(LIS331::USE_I2C); // Selects the bus to be used and sets
                          //  the power up bit on the accelerometer.
                          //  Also zeroes out all accelerometer
                          //  registers that are user writable.
  xl.setFullScale(xl.HIGH_RANGE); // 400G
  // xl.setLogPort(&Serial);

  Serial1.begin(230400); // TX1/RX1 - UART to VESC
  Serial2.begin(420000); //TX2/RX2 - CRSF to ELRS Beta FPV Lite

  CRSF.begin(Serial2);
  UART.setSerialPort(&Serial1);
  // UART.setLogPort(&Serial);
  // UART.setDebugPort(&Serial);
  UART.setDuty(0);
  UART.setDuty(0, 2);

  // Show LEDs for power
  strip.fill(yellow, 0, 25);
  strip.show();
  delay(500);
  strip.fill(blue, 0, 25);
  strip.show();
  delay(500);
  strip.fill(green, 0, 25);
  strip.show();

  while (initStatus() == false) {
    delay(10);
  }
  strip.fill(0x000000, 0, 25);
  Serial.println("Finished setup");
}



void loop() {
  // if (loopCounter == 0) {
  //   timerMicros = micros();
  // }
  // loopCounter ++;

  CRSF.update(); // fast
  // UART.getVescValues(); // ~3.75ms

  // Do this regardless of melty mode for other telemetry, and to prevent locking the connecting in a bad state due to timeout
  // Can fix async later to re-trigger transmit if it's been longer than timeout
  UART.getVescValuesAsync();

  strip.fill(black, 0, 25);
  
  if (CRSF.isLinkUp()) {
    if (channelToBool(7)) {
      strip.setBrightness(MAX_BRIGHTNESS);
    } else {
      strip.setBrightness(5);
    }
    scaledThrottle = channelToPercent(1) * 0.6; // Scale throttle to 60% max for now
    if (scaledThrottle < .05 ) { // Tank drive if throttle is low
      ledTankDriveAnim();
      float rY = channelToPercent(3, true) * 0.25; // Right stick, Y axis, scale to 25%
      float rX = channelToPercent(4, true) * 0.1; // Right stick, X axis, scale to 10%
      UART.setDuty(rY + rX);
      UART.setDuty(rY - rX, 2);
    }
    else { // do melty things
      durationMicros = micros() - startMicros;
      float rY = channelToPercent(3, true); // Right stick, Y axis
      float rX = channelToPercent(4, true); // Right stick, X axis
      float rAngle = atan2(rY, rX);
      float rMagnitude = sqrt(rX*rX + rY*rY);

      distanceFromCenterOffset = channelToPercent(6, true);

      if (channelToBool(5) == true) { // Transmitter switch to decide between motor eRPM calc and accelerometer calc. Eventually this should be sensor fusion instead?
        // if (true) {
          // xl.readAxes(x, y, z); // ~2.5ms
        if (xl.readAxesAsync(x, y, z)) {
        //   // Get absolute values to ignore orientation and subtract measured offset
        //   // Measured avg offsets - x:0.36,y:0.53,z:0.38
          // float xG = fabs(xl.convertToG(XL_MAX, x)) - 0.36; // x is up/down
          float yG = fabs(xl.convertToG(XL_MAX, y)) - 0.53;
          float zG = fabs(xl.convertToG(XL_MAX, z)) - 0.38;
          botRPMFromXl = rpmFromXlGs((sqrt(zG*zG + yG*yG)), distanceFromCenterOffset * 2);
          rpmMicros = (60000.0 * 1000) / botRPMFromXl;
        }
      } else {
        if (!rMagnitude) { // if using motor eRPM, only update when not translating
          wheelRPML = (fabs(UART.data.rpm)/14.0) / 1.6; // eRPM, divide by motor polls and belt reduction
          avgWheelRPM = wheelRPML;
          wheelVelocityMpS = (avgWheelRPM * (MAX_RADS * ((wheelRadiusMm + (distanceFromCenterOffset * 1)) / 1000.0))) / 60;
          botRPMFromMotor = (wheelVelocityMpS / (M_PI * ((138.0 / 1000) * 2))) * 60;
          rpmMicros = (60000.0 * 1000) / botRPMFromMotor;
        }
      }

      if (durationMicros > rpmMicros) { // We've hit the next rotation
        durationMicros = durationMicros - rpmMicros;
        startMicros = startMicros + rpmMicros;
      }

      float currentAngle = microsToRadians(durationMicros, rpmMicros);
      float lX = channelToPercent(2, false);
      if (fabs(lX) > .2) { // Add deadband
        headerOffset -= lX * .02; // Scale by how fast you want to adjust the header
        if (headerOffset > MAX_RADS) {
          headerOffset = 0;
        } else if (headerOffset < 0) {
          headerOffset = MAX_RADS;
        }
      }

      float offsetAngle = currentAngle + headerOffset;
      if (offsetAngle > MAX_RADS) {
        offsetAngle -= MAX_RADS;
      }

      // Serial.print("headerOffset:");
      // Serial.print(headerOffset * (180/M_PI));
      // Serial.print(",");
      // Serial.print("CurrentAngle:");
      // Serial.print(currentAngle * (180/M_PI));
      // Serial.print(",");
      // Serial.print("OffsetAngle:");
      // Serial.print(offsetAngle * (180/M_PI));
      // Serial.print(",");
      // Serial.print((MAX_RADS - MAX_RADS/360*5) * (180/M_PI));
      // Serial.print(",");
      // Serial.println((MAX_RADS/360*5) * (180/M_PI));


      if (offsetAngle >= MAX_RADS - MAX_RADS/360*5 || offsetAngle <= MAX_RADS/360*5) { // Flash header at +- 5 degrees of 0
        if (UART.data.inpVoltage < (12 * 3.50)) {
          strip.fill(red, 0, 25);
        } else {
          strip.fill(white, 0, 25);
        }
      }

      float translationAngle = offsetAngle + rAngle;
      if (translationAngle > MAX_RADS) {
        translationAngle -= MAX_RADS;
      } else if (translationAngle < 0) {
        translationAngle += MAX_RADS;
      }


      float leftMotorDuty = scaledThrottle + (cos(translationAngle) * rMagnitude * 0.2);
      float rightMotorDuty = scaledThrottle + (-cos(translationAngle) * rMagnitude * 0.2);
      // Serial.print("LeftMotor:");
      // Serial.print(leftMotorDuty);
      // Serial.print(",");
      // Serial.print("RightMotor:");
      // Serial.println(rightMotorDuty);

      UART.setDuty(leftMotorDuty);
      UART.setDuty(-rightMotorDuty, 2);
    }
  } else { // ELRS link is down
    strip.fill(yellow, 0, 25);
    UART.setDuty(0);
    UART.setDuty(0, 2);
  }
  strip.show();

  // if (loopCounter >= 1000) {
  //   Serial.println(((micros() - timerMicros)/ loopCounter));
  //   loopCounter = 0;
  // }
}

void ledTankDriveAnim() {
  // unsigned long now = millis()
  // int start = ;
  // int end = 9;
  // strip.fill(purple, 0, 9);
  // strip.fill(green, 9, 16);
  // strip.fill(purple, 16, 25);
  // if (now - lastAnim > 1000) {
  //   animIndex++;
  //   lastAnim = now;
  // }
  strip.fill(purple, 0, 9);
  strip.fill(green, 9, 16);
  strip.fill(purple, 16, 25);
}
  

bool initStatus () {
  bool xlStatus=false, elrsStatus=false, vescStatus=false;
  xl.readAxes(x, y, z);  // The readAxes() function transfers the
                          //  current axis readings into the three
                          //  parameter variables passed to it.
  CRSF.update(); // Update data from ELRS Receiver


  // LEDs for Accelerometer
  Wire.beginTransmission(0x19);
  byte error = Wire.endTransmission();
  if (error == 0) {
    strip.fill(green, 6, 12);
    xlStatus = true;
  } else {
    strip.fill(blue, 6, 12);
    xlStatus = false;
  }

  // LEDs for ELRS receiver
  if (CRSF.isLinkUp()) {
    strip.fill(green, 12, 18);
    elrsStatus = true;
  } else {
    strip.fill(yellow, 12, 18);
    elrsStatus = false;
  }

  // LEDs for VESC
  if (UART.getVescValues()) {
    strip.fill(green, 18, 25);
    vescStatus = true;
  } else {
    strip.fill(red, 18, 25);
    vescStatus = false;
  }

  strip.show();
  delay(100);
  if (xlStatus && elrsStatus && vescStatus) {
    return true;
  } else {
    return false;
  }
}

float channelToPercent(int channel, bool reversible) {
  if (reversible == true) {
    return fmap(CRSF.getChannel(channel), 1000, 2000, -100, 100) / 100.0;
  } else {
    return fmap(CRSF.getChannel(channel), 1500, 2000, 0, 100) / 100.0;
  }
}

bool channelToBool(int channel) {
  return map(CRSF.getChannel(channel), 1000, 2000, 0, 1);
}

float rpmFromXlGs(float g, float distanceFromCenterOffset) {
  float rpm;
  //calculate RPM from g's - derived from "G = 0.00001118 * r * RPM^2"
  rpm = g * 89445.0f;
  rpm = rpm / ((67.385 + distanceFromCenterOffset) / 10) ; // 67.385mm from CoR  (OLD: 14.4mm Xl distance from CoR)
  rpm = sqrt(rpm);
  return rpm;
}

float microsToRadians(unsigned long durationMicros, unsigned long rpmMicros) {
  return fmap(durationMicros, 0, rpmMicros, 0.0, MAX_RADS);
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}