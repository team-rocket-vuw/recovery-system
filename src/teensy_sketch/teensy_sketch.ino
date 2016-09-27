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

  Modified 27th September 2016
  By Jamie Sanson

*/

// region includes
#include <i2c_t3.h>

#include <ESP8266.h>
#include <Data_module.h>
#include <Sensor_helper.h>
#include <SPI.h>
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
int8_t Mscale = MFS_16BITS;  // Choose either 14-bit or 16-bit magnetometer resolution
int8_t Mmode = 0x02;         // 2 for 8Hz, 6 for 100Hz continuous

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output



float aRes, gRes, mRes;
float ax,ay,az, gx, gy, gz, mx, my, mz;

int altitudeCount = 0;
float altitudeBuffer = 0;
// end region

// region flags
boolean serialDebugMode = false;
boolean initialiseOK = true;
boolean altInit = false;
// end region

// region library instantiation
Data_module dataModule(sdChipSelect, debugSerialBaud, initFileName, dataFileName);
Sensor_helper helper(Ascale, Gscale, Mscale, Mmode);
// end region

void setup() {
  dataModule.setDebugMode(serialDebugMode); // set debug flag in library instance
  dataModule.initialize(); // setup data module

  // begin I2C for MPU IMU
  Wire1.begin(I2C_MASTER, 0x000, I2C_PINS_29_30, I2C_PULLUP_EXT, I2C_RATE_400);

  // Sensor setup
  setupMPU9250();
  setupAK8963();
  setupMS5637();

  runInitLoop();
}

void loop() {
  runMainLoop();
}

// ------------------------------------------------------------
//                      Loop declarations
// ------------------------------------------------------------

void runMainLoop() {
  // TODO: Read sensors, detect apogee, send radio, activate buzzer
}

void runInitLoop() {
  // TODO: ESP handshaking and blocking to wait on commands
  while (1);
}

// ------------------------------------------------------------
//                   Function definitions
// ------------------------------------------------------------

// REGION DATA COLLECTION FUNCTIONS
float getAltitude(){
  if (!altInit || altitudeCount == 256) {
    D1 = helper.MS5637Read(ADC_D1, OSR);  // get raw pressure value
    D2 = helper.MS5637Read(ADC_D2, OSR);  // get raw temperature value
    dT = D2 - Pcal[5]*pow(2,8);    // calculate temperature difference from reference
    OFFSET = Pcal[2]*pow(2, 17) + dT*Pcal[4]/pow(2,6);
    SENS = Pcal[1]*pow(2,16) + dT*Pcal[3]/pow(2,7);

    Temperature = (2000 + (dT*Pcal[6])/pow(2, 23))/100;   // First-order Temperature in degrees celsius

    // Second order corrections
    if(Temperature > 20)
    {
      T2 = 5*dT*dT/pow(2, 38); // correction for high temperatures
      OFFSET2 = 0;
      SENS2 = 0;
    }
    if(Temperature < 20)       // correction for low temperature
    {
      T2      = 3*dT*dT/pow(2, 33);
      OFFSET2 = 61*(100*Temperature - 2000)*(100*Temperature - 2000)/16;
      SENS2   = 29*(100*Temperature - 2000)*(100*Temperature - 2000)/16;
    }
    if(Temperature < -15)      // correction for very low temperature
    {
      OFFSET2 = OFFSET2 + 17*(100*Temperature + 1500)*(100*Temperature + 1500);
      SENS2 = SENS2 + 9*(100*Temperature + 1500)*(100*Temperature + 1500);
    }
    // End of second order corrections

    Temperature = Temperature - T2/100;
    OFFSET = OFFSET - OFFSET2;
    SENS = SENS - SENS2;

    Pressure = (((D1*SENS)/pow(2, 21) - OFFSET)/pow(2, 15))/100;  // Pressure in mbar or kPa

    altitudeBuffer = ((145366.45*(1.0 - pow((Pressure/1013.25), 0.190284)))/3.2808) - altitudeOffset; // Altitude calculation
    altitudeCount = 0;
  } else {
    altitudeCount++;
  }

  return altitudeBuffer;
}
// END REGION

// REGION SETUP FUNCTIONS
void setupMPU9250() {
  dataModule.print("Reading who-am-i byte of MPU9250\n");
  byte c = helper.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250

  dataModule.print("MPU9250 I AM "); dataModule.print(String(c, HEX)); dataModule.print(", I should be "); dataModule.print(String(0x71, HEX) + "\n");

  if (c == 0x71) {
    dataModule.print("MPU9250 online\n");
    dataModule.print("Calibrating...\n\n");

    helper.calibrateMPU9250(gyroBias, accelBias);

    dataModule.print("Accelerometer bias: (mg)\n");
    dataModule.print("X: " + (String)(1000*accelBias[0]) + " Y: " + (String)(1000*accelBias[1]) + " Z: " + (String)(1000*accelBias[2]) + "\n");

    dataModule.print("Gyro bias: (o/s)\n");
    dataModule.print("X: " + (String)gyroBias[0] + " Y: " + (String)gyroBias[1] + " Z: " + (String)gyroBias[2] + "\n");

    helper.initMPU9250();
    dataModule.print("\nMPU9250 initialized for active data mode....\n\n");
  } else {
    initialiseOK = false;
    dataModule.print("MPU9250 failed to initialise\n");
  }
}

void setupAK8963() {
  dataModule.print("Reading who-am-i byte of magnetometer\n");
  byte d = helper.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for AK8963
  dataModule.print("AK8963 I AM "); dataModule.print(String(d, HEX)); dataModule.print(", I should be "); dataModule.print(String(0x48, HEX) + "\n");

  if (!d == 0x48) {
    initialiseOK = false;
    dataModule.print("AK8963 failed to initialise\n");
  }

  helper.initAK8963(magCalibration);

  dataModule.print("Calibrating...\n");
  dataModule.print("X-Axis sensitivity adjustment value "); dataModule.print(String(magCalibration[0], 2) + "\n");
  dataModule.print("Y-Axis sensitivity adjustment value "); dataModule.print(String(magCalibration[1], 2) + "\n");
  dataModule.print("Z-Axis sensitivity adjustment value "); dataModule.print(String(magCalibration[2], 2) + "\n");
  dataModule.print("\nAK8963 initialized for active data mode....\n");
}

void setupMS5637(){
  helper.resetMS5637();
  delay(100);
  dataModule.print("MS5637 pressure sensor reset...\n");
  // Read PROM data from MS5637 pressure sensor
  helper.readPromMS5637(Pcal);
  dataModule.print("PROM data read:\n");
  dataModule.print("C0 = "); dataModule.print(String(Pcal[0]) + "\n");
  unsigned char refCRC = Pcal[0] >> 12;
  dataModule.print("C1 = "); dataModule.print(String(Pcal[1]) + "\n");
  dataModule.print("C2 = "); dataModule.print(String(Pcal[2]) + "\n");
  dataModule.print("C3 = "); dataModule.print(String(Pcal[3]) + "\n");
  dataModule.print("C4 = "); dataModule.print(String(Pcal[4]) + "\n");
  dataModule.print("C5 = "); dataModule.print(String(Pcal[5]) + "\n");
  dataModule.print("C6 = "); dataModule.print(String(Pcal[6]) + "\n");

  nCRC = helper.checkMS5637CRC(Pcal);  //calculate checksum to ensure integrity of MS5637 calibration data
  dataModule.print("Checksum: " + String(nCRC) + ", should be: " + String(refCRC) + "\n");

  if (nCRC != refCRC) {
    initialiseOK = false;
    dataModule.print("MS5637 checksum integrity failed\n");
  }

  // Calculate offset for relative altitude
  float altitudeTemp = 0;
  for(int i = 0; i < 16; i++) {
    altitudeTemp += getAltitude();
  }

  altitudeOffset = altitudeTemp/16;
  altInit = true;
}
// END REGION
