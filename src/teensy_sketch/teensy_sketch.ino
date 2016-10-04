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
#include <RF22.h>

#include <Data_module.h>
#include <Sensor_helper.h>
// end region

// region pin definitions
#define radioIntPin 0
#define buzzerPin 3
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
#define csvLayout "ACCEL_X,ACCEL_Y,ACCEL_Z,GYRO_X,GYRO_Y,GYRO_Z,MAG_X,MAG_Y,MAG_Z,LAT,LNG,"
#define gpsDecimalPoints 6
#define buzzerFrequency 2048
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
RF22 rf22(radioChipSelect, radioIntPin);
// end region

void setup() {
  dataModule.setDebugMode(serialDebugMode); // set debug flag in library instance
  initializeSPI();

  dataModule.initialize();                  // setup data module

  // begin I2C for MPU IMU
  Wire1.begin(I2C_MASTER, 0x000, I2C_PINS_29_30, I2C_PULLUP_EXT, I2C_RATE_400);

  // begin serial used for GPS and ESP
  gpsSerial.begin(gpsSerialBaud);
  espSerial.begin(espSerialBaud);

  // Sensor setup
  setupIMU();
  // Radio setup
  setupRFM();
  // WiFi setup
  setupESP();
  // GPS setup
  setupGPS();

  // Notify dataModule to flush init buffers
  dataModule.initComplete();
  // Print CSV layout to file
  dataModule.println(csvLayout);
}

void loop() {
  // get information from sensors and GPS
  readGPS();
  readIMU();

  // Print all information
  dataModule.print(getIMULogString(accelData) + getIMULogString(gyroData) + getIMULogString(magData));
  dataModule.println(getGPSLogString());
}

// ------------------------------------------------------------
//                   Function definitions
// ------------------------------------------------------------

// Function to handle SPI initialisation
void initializeSPI() {
  // start chipselects
  pinMode(radioChipSelect, OUTPUT);
  pinMode(sdChipSelect, OUTPUT);

  // start SPI
  SPI.begin();
}

// Function to set up all sensors on IMU board
void setupIMU() {
  delay(1000);
  // If statement enters if MPU9250 and MS5637 initialise properly
  if (helper.setupMPU9250() && helper.setupMS5637()) {
    toneIMUSuccess();
  }

  // This sensor is iffy with initialising properly, so don't care about success at this point
  helper.setupAK8963();
}

// Function to set up high-accuracy gyro and bind interrupt handler
void setupGyro() {
  helper.setupL3G4200D(L3G4200DScale, -1);
  attachInterrupt(digitalPinToInterrupt(1), handleGyroInterrupt, FALLING);
}

// Function to set up RFM radio module
void setupRFM() {
  if (!rf22.init()) {
    dataModule.println("\nRFM22B failed to initialise");
  } else {
    rf22.setModeTx(); // Turns off Rx
    rf22.setTxPower(RF22_TXPOW_8DBM);
    rf22.setModemConfig(RF22::UnmodulatedCarrier);
    delay(100);
    dataModule.println("\nRFM22B initialisation success");
  }
}

// Function for setting up ESP8266-01
void setupESP() {
  // TODO setup ESP
}

// Function for waiting for GPS communication and locking
void setupGPS() {
  for (int i = 0; !gps.location.isUpdated(); i++) {
    readGPS();
    delay(20);
    // Only print locking information every 50 iterations
    if (i%50 == 0) {
      dataModule.println("Checksum passed/failed: " + String(gps.passedChecksum(), DEC) + "/" + String(gps.failedChecksum(), DEC));
    }
  }

  toneGPSSuccess();
}

// Gyro interrupt service routine
void handleGyroInterrupt() {
  L3G4200DReadReady = true;
}

// Tones buzzer for successful sensor initialisation
void toneIMUSuccess() {
  tone(buzzerPin, buzzerFrequency, 500);
  delay(500);
  tone(buzzerPin, buzzerFrequency, 500);
}

// Tones buzzer for successful gps lock
void toneGPSSuccess() {
  tone(buzzerPin, buzzerFrequency, 1000);
  delay(1000);
  tone(buzzerPin, buzzerFrequency, 500);
  delay(500);
  tone(buzzerPin, buzzerFrequency, 1000);
}


// --------------------
// Data read functions
// --------------------


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
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
}

// Blocking function to read info from ESP
void readESP() {
  while (espSerial.available()) {
    dataModule.print(espSerial.read());
  }
}


// ------------------
// Utility functions
// ------------------


// Simple function for building a CSV formatted string from a 3 element array
String getIMULogString(float * data) {
  return (String(data[0], DEC) + "," + String(data[1], DEC) + "," + String(data[2], DEC) + ",");
}

// Similar function as above, with different type parameter
String getGyroLogString(int16_t * data) {
  return (String(data[0], DEC) + "," + String(data[1], DEC) + "," + String(data[2], DEC) + ",");
}

// Function for returning formatted GPS string
String getGPSLogString() {
  return (String(gps.location.lat(), gpsDecimalPoints) + "," + String(gps.location.lng(), gpsDecimalPoints) + ",");
}
