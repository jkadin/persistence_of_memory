#include <Adafruit_DotStar.h>
#include <SPI.h>
#include "SparkFun_LIS331.h"
#include <Wire.h>
#include <VescUart.h>
#include <AlfredoCRSF.h>

#define NUMPIXELS 25

LIS331 xl;
VescUart UART;
AlfredoCRSF CRSF;

Adafruit_DotStar strip(NUMPIXELS, DOTSTAR_BGR);

const uint32_t red = 0xFF0000;
const uint32_t green = 0x00FF00;
const uint32_t blue = 0x0000FF;
const uint32_t black = 0x000000;
const uint32_t yellow = 0xFFFF00;
const uint32_t purple = 0x800080;
const uint32_t white = 0xFFFFFF;

const int XL_MAX = 400;
const int MAX_BRIGHTNESS = 200;

const float MAX_RADS = 2 * M_PI;

float wheelRPML = 0, wheelRPMR = 0, botRPMFromMotor = 0, botRPMFromXl = 0, wheelVelocityMpS = 0;
const float wheelRadiusMm = 123.825;
unsigned long startMicros = 0, rpmMicros = 0, durationMicros = 0;
float headerOffset = 0;
int16_t x, y, z;
float scaledThrottle = 0;
float avgWheelRPM = 0;
float distanceFromCenterOffset = 0;

unsigned long timerMicros = 0;
int loopCounter = 0;

float channelToPercent(int channel, bool reversible=false);

void setup() {
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

  Serial.begin(9600); // Teensy always uses USB Serial speed, this speed is ignored
  Serial.println("Setup started");

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
  strip.fill(black, 0, 25);
  
  if (CRSF.isLinkUp()) {
    if (channelToBool(7)) {
      strip.setBrightness(MAX_BRIGHTNESS);
    } else {
      strip.setBrightness(5);
    }
    scaledThrottle = channelToPercent(1) * 0.6; // Scale throttle to 60% max for now
    if (scaledThrottle < .05 ) { // Tank drive if throttle is low
      strip.fill(purple, 0, 9);
      strip.fill(green, 9, 16);
      strip.fill(purple, 16, 25);
      float rY = channelToPercent(3, true) * 0.25; // Right stick, Y axis, scale to 25%
      float rX = channelToPercent(4, true) * 0.1; // Right stick, X axis, scale to 10%
      UART.setDuty(rY + rX);
      UART.setDuty(rY - rX, 2);
    }
    else { // do melty things
      durationMicros = micros() - startMicros;
      xl.readAxes(x, y, z); // ~2.8ms
      float rY = channelToPercent(3, true); // Right stick, Y axis
      float rX = channelToPercent(4, true); // Right stick, X axis
      float rAngle = atan2(rY, rX);
      float rMagnitude = sqrt(rX*rX + rY*rY); // Scale to 20% of stick value
      // UART.getVescValues(); // ~3.75ms
      UART.getVescValuesAsync(); // fast
      distanceFromCenterOffset = channelToPercent(6, true) * 2;
      x = xl.convertToG(XL_MAX, x);
      y = xl.convertToG(XL_MAX, y);
      z = xl.convertToG(XL_MAX, z);
      botRPMFromXl = rpmFromXlGs((sqrt(z*z + y*y)), distanceFromCenterOffset);
      if (channelToBool(5) == true) { // Transmitter switch to decide between motor eRPM calc and accelerometer calc. Eventually this should be sensor fusion instead?
        rpmMicros = (60000.0 * 1000) / botRPMFromXl;
      } else {
        if (!rMagnitude) { // if using motor eRPM, only update when not translating
          wheelRPML = (fabs(UART.data.rpm)/14.0) / 1.6; // eRPM, divide by motor polls and belt reduction
          avgWheelRPM = wheelRPML;
          wheelVelocityMpS = (avgWheelRPM * (MAX_RADS * (wheelRadiusMm / 1000.0))) / 60;
          botRPMFromMotor = (wheelVelocityMpS / (M_PI * ((138.0 / 1000) * 2))) * 60;
          rpmMicros = (60000.0 * 1000) / botRPMFromMotor;
        }
      }

      if (durationMicros > rpmMicros) { // We've hit the next rotation
        durationMicros = durationMicros - rpmMicros;
        startMicros = startMicros + rpmMicros;
      }

      float currentAngle = microsToRadians(durationMicros, rpmMicros);
      float lX = channelToPercent(2);
      if (fabs(lX) > .2) { // Add deadband
        headerOffset += lX * .02; // Scale by how fast you want to adjust the header
        if (headerOffset > MAX_RADS) {
          headerOffset = 0;
        } else if (headerOffset < 0) {
          headerOffset = MAX_RADS;
        }
      }

      // Serial.print("headerOffset:");
      // Serial.print(headerOffset * (180/M_PI));
      // Serial.print(",");
      // Serial.print("CurrentAngle:");
      // Serial.print(currentAngle * (180/M_PI));
      // Serial.print(",");
      // Serial.print("OffsetAngle:");
      // Serial.println(offsetAngle * (180/M_PI));

      float offsetAngle = currentAngle + headerOffset;
      if (offsetAngle > MAX_RADS) {
        offsetAngle -= MAX_RADS;
      }

      if (MAX_RADS - MAX_RADS/360*5 <= offsetAngle <= MAX_RADS/360*5) { // Flash header at +- 5 degrees of 0
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


      float leftMotorDuty = scaledThrottle + (cos(translationAngle) * rMagnitude*0.1);
      float rightMotorDuty = (-1 * scaledThrottle) + (-cos(translationAngle) * rMagnitude*0.1);
      // Serial.print("LeftMotor:");
      // Serial.print(leftMotorDuty);
      // Serial.print(",");
      // Serial.print("RightMotor:");
      // Serial.println(rightMotorDuty);

      UART.setDuty(leftMotorDuty);
      UART.setDuty(rightMotorDuty, 2);
    }
  } else { // ELRS link is down
    strip.fill(yellow, 0, 25);
    UART.setDuty(0);
    UART.setDuty(0, 2);
  }
  strip.show();

  // if (loopCounter >= 200) {
  //   Serial.println(((micros() - timerMicros)/ loopCounter)/1000.0);
  //   loopCounter = 0;
  // }
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
    return map(CRSF.getChannel(channel), 1000, 2000, -100, 100) / 100.0;
  } else {
    return map(CRSF.getChannel(channel), 1500, 2000, 0, 100) / 100.0;
  }
}

bool channelToBool(int channel) {
  return map(CRSF.getChannel(channel), 1000, 2000, 0, 1);
}

float rpmFromXlGs(int g, float distanceFromCenterOffset) {
  float rpm;
  //use of absolute makes it so we don't need to worry about accel orientation
  //calculate RPM from g's - derived from "G = 0.00001118 * r * RPM^2"
  rpm = fabs(g) * 89445.0f;
  rpm = rpm / ((67.385 + distanceFromCenterOffset) / 10) ; // 67.385mm from CoR  (OLD: 14.4mm Xl distance from CoR)
  rpm = sqrt(rpm);
  return rpm;
}

float microsToDegrees(long durationMicros, long rpmMicros) {
  return map(durationMicros, 0, rpmMicros, 0.0, 360.0);
}

float microsToRadians(long durationMicros, long rpmMicros) {
  return map(durationMicros, 0, rpmMicros, 0.0, MAX_RADS);
}

float degreesToPercent(int degrees) {
  return map(degrees, 0, 360, 0.0, 100.0);
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}