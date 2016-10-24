#include <Arduino.h>

/*
  This sketch is written for use on a Teensy 3.1/3.2
  The circuit:
  * Components Outputted to:
    - RFM22B-S2 434MHz radio tranciever
      * SDI    - pin 11
      * SDO    - pin 12
      * CLK    - pin 13
      * CS     - as defined in radioChipSelect field
      * NIRQ   - pin 0

  Created 24 October 2016
  By Marcel van Workum

  Created 24 October 2016
  By Marcel van Workum

*/

#include <RF22_helper.h>
// end region

// region pin definitions
#define radioIntPin 0
#define radioChipSelect 5
// end region

// region flags
boolean serialDebugMode = false;
boolean initialiseOK = true;
// end region

RF22_helper rf22(radioChipSelect, radioIntPin);
// end region

IntervalTimer rf22InterruptTimer;

void setup() {
  pinMode(radioChipSelect, OUTPUT);

  delay(1000);

  rf22.initialize();

  rf22InterruptTimer.begin(transmit, 3333);
}

void transmit() {
  rf22.transmitBuffer();
}

void loop() {
  rf22.enqueueMessage("$$$$Message");
  delay(2000);
}

