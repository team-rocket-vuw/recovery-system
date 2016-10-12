#include <RF22.h>
#include "RF22_helper.h"

/*
  Set up RF22 Radio module
*/
RF22_helper::RF22_helper(int radioChipSelect, int radioIntPin) {
  // TODO are these two values needed anywhere else than the initialisation of the
  // rf22 module? if not then we can remove them as fields
  _radioChipSelect = radioChipSelect;
  _radioIntPin = radioIntPin;

  RF22 rf22(radioChipSelect, radioIntPin);
  // TODO again, not sure if this is right? Can't really test it
  _rf22 = rf22;

  // Initially set buffer to be empty so that anything can write to it
  _messageBufferCount = 0;
}

bool RF22_helper::initialize() {
  if (!_rf22.init()) {
    return false;
  } else {
    _rf22.setModeTx(); // Turns off Rx
    _rf22.setTxPower(RF22_TXPOW_8DBM);
    _rf22.setModemConfig(RF22::UnmodulatedCarrier);
    delay(100);
    return true;
  }
}

void RF22_helper::b_broadcastMessage(String message, int broadcastCount) {
  for (int i = 0; i < broadcastCount; i++) {
    // Wait for buffer to be empty
    while(_messageBufferCount > 0) {}

    _rf22.spiWrite(0x07, 0x08); // turn tx on
    RF22_helper::enqueueMessage(message);
    delay(5000);
    _rf22.spiWrite(0x07, 0x01); // turn tx off
  }
}

bool RF22_helper::enqueueMessage(String stringToEnqueue) {
  if (_messageBufferCount == 0) {
    char messageChars[stringToEnqueue.length()];
    stringToEnqueue.toCharArray(messageChars, stringToEnqueue.length());

    // TODO should we actually block interrupts here?
    noInterrupts();

    // First add start bit of 0
    _messageBitBuffer[_messageBufferCount] = 0;

    // then increment the buffer pointers
    _messageBufferCount++;

    // enqueue least significant bit first
    for (unsigned int i = 0; i < (sizeof(messageChars) / sizeof(char)); i++) {
      char c = messageChars[i];

      for (int j = 0; j < 8; j++) {
        // c would be the char from the outer for loop
        if (c & 1) {
          _messageBitBuffer[_messageBufferCount] = 1;
          _messageBufferCount++;
        } else {
          _messageBitBuffer[_messageBufferCount] = 0;
          _messageBufferCount++;
        }
        c = c >> 1;
      }
    }

    // Finally add end bit of 1
    _messageBitBuffer[_messageBufferCount] = 1;

    // then increment the buffer pointers
    _messageBufferCount++;

    interrupts();

    return true;
  } else {
    return false;
  }
}

void RF22_helper::transmitBuffer() {
  if (_messageBufferCount > 0) {
    int bit = _messageBitBuffer[0];

    // tramits the bit

    if (bit) {
      _rf22.setFrequency(HIGH_FREQENCY);
    } else {
      _rf22.setFrequency(LOW_FREQUENCY);
    }


    for (int i = 1; i < _messageBufferCount; i++) {
      _messageBitBuffer[i - 1] = _messageBitBuffer[i];
    }
    _messageBufferCount--;
  }
}
