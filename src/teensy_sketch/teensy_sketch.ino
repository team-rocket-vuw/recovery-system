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
#define baseStationConnection Serial2
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
String baseStationBuffer = "";
boolean skipGps = false;

int i = 0;
// end region

// region library instantiation
TinyGPSPlus gps;
TinyGPSCustom satsInView(gps, "GPGSV", 3); // GPGSV messages contain satellites in view in the third field

Data_module dataModule(sdChipSelect, debugSerialBaud, initFileName, dataFileName);
RF22_helper rf22(radioChipSelect, radioIntPin);

IntervalTimer rf22InterruptTimer;
// end region

// --------------------- SETUP ---------------------
void setup() {
  pinMode(radioChipSelect, OUTPUT);
  pinMode(sdChipSelect, OUTPUT);
  gpsSerial.begin(gpsSerialBaud);
  baseStationConnection.begin(debugSerialBaud);

  runInitialisationRoutine();
}

// --------------------- LOOP ---------------------
void loop() {
  i += 1;
  readGPS();
  rf22.enqueueMessage(getGPSMessageString());

  sendStateUpdate("GPSLAT=" + String(-41.288 + (i*0.00005), 5));
  sendStateUpdate("GPSLNG=" + String(174.762 + (i*0.00005), 5));
  sendStateUpdate(getGPSLockingMessage());
  delay(10);
}


// -------- RADIO AND INIT FUNCTIONS ----------
/*
Function used in as the RFM22 interrupt handler for timer driven radio communication
*/
void transmit() {
  rf22.transmitBuffer();
}

/*
Function to be called to begin initialisation flow. Takes commands from base station,
initialises components and reports state updates. More on these state messages can be found in
base-station-sketch.ino
*/
void runInitialisationRoutine() {
  String commandMessage = receiveCommand();

  if (commandMessage == "start") {
    sendAcknowledge();

    // set debug flag in library instance
    dataModule.setDebugMode(serialDebugMode);
    // Send result of initialisation to base station
    sendStateUpdate(dataModule.initialize() ? "DM=OK" : "DM=FAIL");

    sendStateUpdate(rf22.initialize() ? "RFM=OK" : "RFM=FAIL");
    rf22InterruptTimer.begin(transmit, rfmBitSpacingMicroseconds);

    sendStateUpdate("GPS=locking");
    sendStateUpdate(setupGPS() ? "GPS=ready" : "GPS=skipped");

    // Notify dataModule to flush init buffers
    dataModule.initComplete();

    commandMessage = receiveCommand();

    if (commandMessage == "begin") {
      sendAcknowledge();
    } else {
      while (true);
    }
  }
}

// Function for waiting for GPS communication and locking
boolean setupGPS() {
  for (int i = 0; !gps.location.isUpdated(); i++) {
    readGPS();
    readMessageStream();
    delay(20);
    // Only print locking information every 50 iterations
    if (i % 50 == 0) {
      rf22.enqueueMessage(getGPSLockingMessage());

      dataModule.println("Checksum passed/failed: " + String(gps.passedChecksum(), DEC) + "/" + String(gps.failedChecksum(), DEC) + " Sats in view: " + satsInView.value());

      sendStateUpdate(getGPSLockingMessage());
      // Used for flushing SD card buffer when not in debug mode
      dataModule.flushBuffer();
    } else if (skipGps) {
      return false;
    }
  }

  return true;
}


// -------- BASE-STATION COMMUNICATION FUNCTIONS ----------
/*
Function for sending message over base station communication channel
*/
void sendMessage(String message) {
  baseStationConnection.println(message);
}

/*
Function for sending state update message over base station communication channel
as well as printing the state update to the dataModule instance
*/
void sendStateUpdate(String message) {
    baseStationConnection.print(message + ";");
    dataModule.println("State update: " + message);
}

/*
Function for sending acknowledge message over base station communication channel
*/
void sendAcknowledge() {
  sendMessage("ok");
}

/*
Function for reading from the message steam being recieved from the base station.
Takes a command, splits the newline and/or cariage return character from the end
and flushes anything that's left in the inbound serial buffer
*/
void readMessageStream() {
  while (baseStationConnection.available()) {
    char rec = baseStationConnection.read();
    if (String(rec) == "\n" || String(rec) == "\r") {
      if (baseStationBuffer == "skip_gps") {
        skipGps = true;
        sendAcknowledge();
      }

      baseStationBuffer = "";
      flushSerialBuffer();
    } else {
      baseStationBuffer += String(rec);
    }
  }
}

// function to be called each iteration to feed the GPS instantiation
void readGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
}

/*
Blocking function used to wait for receiving commands from the base station. Blocks until entire command is recieved
*/
String receiveCommand() {
  // Block until command recieved
  while (!baseStationConnection.available());

  String recieved;
  boolean stringComplete = false;
  while (!stringComplete) {
    // Only try read a char if it's available
    if (baseStationConnection.available()) {
       char recChar = baseStationConnection.read();
      // Complete the command if nl/cr found, else concat char onto command
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
  while (baseStationConnection.available()) {
    baseStationConnection.read();
  }
}

// ----------------- UTILITY FUNCTIONS ----------------------
String getGPSMessageString() {
    if (gps.location.lat() == 0 || gps.location.lng() == 0) {
      return rfmMessagePilot + getGPSLockingMessage();
    } else {
      return rfmMessagePilot + (String(gps.location.lat(), gpsDecimalPoints) + "," + String(gps.location.lng(), gpsDecimalPoints) + ",");
    }
}

String getGPSLockingMessage() {
  String inView = satsInView.value();
  if (inView.length() < 2) {
    inView = "00";
  }
  return "GPSVIS=" + String(inView);
}
