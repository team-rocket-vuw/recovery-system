/*
    Module to wrap the functionality of the RF22 Radio Module.
*/

#ifndef RF22_helper_h
#define RF22_helper_h

#define HIGH_FREQENCY 434.2010
#define LOW_FREQUENCY 434.2015
#define RF22_CYCLE_TIME 10000
#define BIT_BUFFER_SIZE 1000

#include <RF22.h>
#include <Arduino.h>

class RF22_helper
{
  public:
    RF22_helper(int radioChipSelect, int radioIntPin);

    bool initialize();

    // TODO added the b_ to signify that this is a blocking method
    // not sure if there is a convention on this or if we even
    // need to bother
    void b_broadcastMessage(String message, int broadcastCount);
    bool enqueueMessage(String stringToEnqueue);
    void transmitBuffer();

  private:
    int _radioChipSelect, _radioIntPin;

    // TODO what size buffer do we actually need?
    int _messageBitBuffer[BIT_BUFFER_SIZE];
    int _messageBufferCount = 0;

    RF22 _rf22;
};

#endif
