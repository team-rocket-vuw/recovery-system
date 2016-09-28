#include <Arduino.h>

/*
  Metis-2 launch onboard sketch
  This sketch contains logic to read sensor values via I2C and SPI and dump data to an SD card
  for further analysis on landing. It also communicates basic data over the 433MHz radio band,
  as well as 2.4GHz WiFi
  This sketch is written for use on a Teensy 3.1/3.2
  The circuit:
  * Components supplying input:
    - Venus GPS logger
      * TX  - pin 8
      * RX  - pin 7
    - 9-Axis motion sensor MPU9250 Shield
      * Connected to the SMT pads on the underside of the Teensy
    - L3G4200D Gyro breakout
      * SCL - pin 13
      * SDA - pin 11
      * SDO - pin 12
      * CS  - as defined in gyroChipSelect field
      * INT1 - pin 1
      * INT2 - NC
      * Other pins connected to independent power supplies
  * Components Outputted to:
    - ESP8266-01 WiFi Module
      * TXD - pin 10
      * RXD - pin 9
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
      * NIRQ   - pin 0
    - Piezo buzzer
      * +ve - pin 3

  Created 28 August 2016
  By Jamie Sanson

  Modified 28th September 2016
  By Jamie Sanson

*/

// region includes
#include <i2c_t3.h>
#include <SPI.h>
#include <TinyGPS++.h>

#include <Data_module.h>
#include <Sensor_helper.h>
// end region

// region pin definitions
#define radioIntPin 0
#define gyroIntPin 1
#define buzzerPin 3
#define gyroChipSelect 4
#define radioChipSelect 5
#define sdChipSelect 6
// end region

// region macro definitions
#define espSerial Serial2
#define gpsSerial Serial3
#define gpsSerialBaud 9600
#define debugSerialBaud 115200
#define espSerialBaud 115200
#define initFileName "init/init"
#define dataFileName "data/data"
#define initExtension "txt"
#define dataExtension "csv"
// end region

// region IMU configuration
uint8_t OSR = ADC_8192;
int8_t Ascale = AFS_16G;
int8_t Gscale = GFS_250DPS;
int8_t Mscale = MFS_14BITS;  // Choose either 14-bit or 16-bit magnetometer resolution
int8_t L3G4200DScale = L3G4200D_250DPS;
int8_t Mmode = 0x02;         // 2 for 8Hz, 6 for 100Hz continuous

float accelData[3] = {0,0,0}; // 3-axis accelerometer readings defines as x,y,z output in ms^-1
float gyroData[3]  = {0,0,0}; // 3-axis gyro readings defines as x,y,z output in degrees per second
float magData[3]   = {0,0,0}; // 3-axis magnetometer readings defines as x,y,z output in milliGauss

int16_t L3G4200DData[3] = {0,0,0}; // 3-axis high-accuracy gyro readings
// end region

// region flags
boolean serialDebugMode = true;
boolean initialiseOK = true;
boolean L3G4200DReadReady = false;
// end region

// region library instantiation
TinyGPSPlus gps;
Data_module dataModule(sdChipSelect, debugSerialBaud, initFileName, dataFileName);
Sensor_helper helper(Ascale, Gscale, Mscale, Mmode, &dataModule); // Data_module required for sensor debug
// end region

void setup() {
  dataModule.setDebugMode(serialDebugMode); // set debug flag in library instance
  dataModule.initialize();                  // setup data module

  // begin I2C for MPU IMU
  Wire1.begin(I2C_MASTER, 0x000, I2C_PINS_29_30, I2C_PULLUP_EXT, I2C_RATE_400);

  // begin serial used for GPS data
  gpsSerial.begin(gpsSerialBaud);

  // begin SPI for high-accuracy gyro
  SPI.begin();

  // Sensor setup
  delay(500);
  setupIMU();
  setupGyro();

  dataModule.initComplete();
}

void loop() {
  readGPS();

  readIMU();
  readGyro();

  delay(200);
  dataModule.println(getIMULogString(accelData)+getIMULogString(gyroData)+getIMULogString(magData));
  dataModule.println(getGyroLogString(L3G4200DData));
  dataModule.println("Got " + String(gps.charsProcessed(), DEC) + " chars");
}

// ------------------------------------------------------------
//                   Function definitions
// ------------------------------------------------------------

// Function to set up all sensors on IMU board
void setupIMU() {
  // TODO: Check return of each to ensure success
  helper.setupMPU9250();
  helper.setupAK8963();
  helper.setupMS5637();
}

// Function to set up high-accuracy gyro and bind interrupt handler
void setupGyro() {
  helper.setupL3G4200D(L3G4200DScale, gyroChipSelect);
  attachInterrupt(digitalPinToInterrupt(gyroIntPin), handleGyroInterrupt, FALLING);
}

// Gyro interrupt service routine
void handleGyroInterrupt() {
  L3G4200DReadReady = true;
}

// Function which reads each 3-axis sensor on the SMT IMU
void readIMU() {
  helper.getIMUAccelData(accelData);
  helper.getIMUGyroData(gyroData);
  helper.getIMUMagData(magData);
}

// Function for reading and updating information when ready
void readGyro() {
  if (L3G4200DReadReady) {
    helper.getL3G4200DGyroData(L3G4200DData);
    L3G4200DReadReady = false;
  }
}

// function to be called each iteration to feed the GPS instantiation
void readGPS() {
  if (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
}

// Simple function for building a CSV formatted string from a 3 element array
String getIMULogString(float * data) {
  return (String(data[0], DEC) + "," + String(data[1], DEC) + "," + String(data[2], DEC) + ",");
}

// Similar function as above, with different type parameter
String getGyroLogString(int16_t * data) {
  return (String(data[0], DEC) + "," + String(data[1], DEC) + "," + String(data[2], DEC) + ",");
}
