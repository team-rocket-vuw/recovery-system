#include <Arduino.h>

/*
  This sketch is written for use on a Teensy 3.1/3.2
  The circuit:
  * Components supplying input:
    - Venus GPS logger
      * TX  - pin 8
      * RX  - pin 7
  * Components Outputted to:
    - RFM22B-S2 434MHz radio tranciever
      * SDI    - pin 11
      * SDO    - pin 12
      * CLK    - pin 13
      * CS     - as defined in radioChipSelect field
      * NIRQ   - pin 0

  Created 24 October 2016
  By Marcel van Workum

  Modified 25 October 2016
  By Jamie Sanson

*/

// Region includes
#include <RF22_helper.h>
#include <Data_module.h>
#include <TinyGPS++.h>
// end region

// region pin definitions
#define radioIntPin 0
#define radioChipSelect 5
#define sdChipSelect 4
// end region

// region macro definitions
#define gpsSerial Serial3
#define mockWireless Serial2
#define gpsSerialBaud 9600
#define debugSerialBaud 115200
#define initFileName "init"
#define dataFileName "data"
#define initExtension "txt"
#define dataExtension "csv"
#define gpsDecimalPoints 6
#define rfmBitSpacingMicroseconds 10000
#define rfmMessagePilot "$$$$"
// end region

// region flags
boolean serialDebugMode = true;
// end region

// region library instantiation
TinyGPSPlus gps;
TinyGPSCustom satsInView(gps, "GPGSV", 3); // GPGSV messages contain satellites in view in the third field

Data_module dataModule(sdChipSelect, debugSerialBaud, initFileName, dataFileName);
RF22_helper rf22(radioChipSelect, radioIntPin);

IntervalTimer rf22InterruptTimer;
// end region


void setup() {
  pinMode(radioChipSelect, OUTPUT);
  pinMode(sdChipSelect, OUTPUT);
  gpsSerial.begin(gpsSerialBaud);
  mockWireless.begin(debugSerialBaud);
  
  String commandMessage = waitMockCommand();
  if (commandMessage == "start") {
    dataModule.setDebugMode(serialDebugMode); // set debug flag in library instance
    sendMockSerial(dataModule.initialize() ? "DM_OK" : "DM_FAIL");
  }

  delay(1000);
  
  commandMessage = waitMockCommand();
  if (commandMessage == "cont") {
    sendMockSerial(rf22.initialize() ? "RFM_OK" : "RFM_FAIL");
    rf22InterruptTimer.begin(transmit, rfmBitSpacingMicroseconds);
  }

  
  if (commandMessage == "cont") {
    sendMockSerial("GPS_LOCK");
    
    setupGPS();
    
    sendMockSerial("GPS_OK");
  }

  
  commandMessage = waitMockCommand();

  if (commandMessage == "begin_loop") {
    // Notify dataModule to flush init buffers
    dataModule.initComplete();
  } else {
    while (true);
  }

}

void loop() {
  readGPS();
  rf22.enqueueMessage(getGPSMessageString());
  delay(1000);
}

void transmit() {
  rf22.transmitBuffer();
}

// Function for waiting for GPS communication and locking
void setupGPS() {
  for (int i = 0; !gps.location.isUpdated(); i++) {
    readGPS();
    delay(20);
    // Only print locking information every 50 iterations
    if (i % 50 == 0) {
      rf22.enqueueMessage(getGPSLockingMessage());

      dataModule.println("Checksum passed/failed: " + String(gps.passedChecksum(), DEC) + "/" + String(gps.failedChecksum(), DEC) + " Sats in view: " + satsInView.value());
      // Used for flushing SD card buffer when not in debug mode
      dataModule.flushBuffer();
    }
  }
}

// function to be called each iteration to feed the GPS instantiation
void readGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
}

String getGPSMessageString() {
    return rfmMessagePilot + (String(gps.location.lat(), gpsDecimalPoints) + "," + String(gps.location.lng(), gpsDecimalPoints) + ",");
}

String getGPSLockingMessage() {
  String inView = satsInView.value();
  if (inView.length() < 2) {
    inView = "00";
  }
  return String(rfmMessagePilot) + "SIV=" + String(inView);
}

void sendMockSerial(String message) {
  mockWireless.println(message);
}

String waitMockCommand() {
  // Block until command recieved
  while (!mockWireless.available());

  String recieved;
  boolean stringComplete = false;
  while (!stringComplete) {
    // Only try read a char if it's available
    if (mockWireless.available()) {
       char recChar = mockWireless.read();
       if (String(recChar) == "\n" || String(recChar) == "\r") {
          stringComplete = true;
        } else {
          recieved += String(recChar);
        }
    } 
  }

  // Clear out what's left in the buffer
  flushSerialBuffer();
  
  return recieved;
}

// Stupid hack to clear out any remaining character
void flushSerialBuffer() {
  while (mockWireless.available()) {
    char z = mockWireless.read();
  }
}

