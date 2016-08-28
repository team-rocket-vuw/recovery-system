/*
  Metis-2 launch onboard sketch
  This sketch contains logic to read sensor values via I2C and SPI and dump data to an SD card
  for further analysis on landing. It also communicates basic data over the 433MHz radio band,
  as well as 2.4GHz WiFi
  This sketch is written for use on a Teensy 3.1/3.2
  The circuit:
  * Components supplying input:
    - 9-Axis motion sensor MPU9250 Shield
      * Connected to the SMT pads on the underside of the Teensy
    - L3G4200D Gyro breakout
      * SCL - pin 14
      * SDA - pin 7
      * SDO - pin 8
      * CS  - as defined in gyroChipSelect field
      * INT1 - pin 2
      * INT2 - pin 3
      * Other pins connected to independent power supplies
  * Components Outputted to:
    - ESP8266-01 WiFi Module
      * TXD - pin 9
      * RXD - pin 10
    - BOB-00544 microSD card SPI breakout
      * MOSI   - pin 11
      * MISO   - pin 12
      * CLK    - pin 13
      * CS     - as defined in sdChipSelect field
    - RFM22B-S2 434MHz radio tranciever
      * SDI    - pin 11
      * SDO    - pin 12
      * CLK    - pin 13
      * CS     - as defined in radioChipSelect field
    - Piezo buzzer
      * +ve - pin 16
      
  Created 28 August 2016
  By Jamie Sanson
  
  Modified 28 August 2016
  By Jamie Sanson
  
*/

// region includes
#include <i2c_t3.h>
#include <SD.h>
#include <SPI.h>
// end region

// region macro definitions
#define espSerial Serial2
// end region

// region pin definitions
int8_t gyroChipSelect = 4;
int8_t radioChipSelect = 5;
int8_t radioIntPin = 15;
int8_t sdChipSelect = 6;
// end region

// region flags
boolean serialDebugMode = true;
// end region

void setup() {
  if (serialDebugMode) {
    // block until serial sent to micro
    Serial.begin(115200);
    
    while(!Serial.available()){}
  } 

  // TODO: Sensor setup, SD setup, ESP handshaking and blocking to wait on commands
}

void loop() {
  // TODO: Read sensors, detect apogee, send radio, activate buzzer
}


