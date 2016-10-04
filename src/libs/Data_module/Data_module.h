/*
    Module used for data logging specific functions. This includes SD logging
    and serial logging.
*/

#ifndef Data_module_h
#define Data_module_h

#include "Arduino.h"
#include <SD.h>

class Data_module
{
  public:
    Data_module(int chipSelect, int serialBaud, String initFileName, String dataFileName);

    boolean initialize();
    void setDebugMode(boolean isDebugMode);

    void print(String stringToPrint);
    void println(String stringToPrint);

    void initComplete();

    void flushBuffer();

  private:
    String _initFileName, _dataFileName, _dataBuffer;
    int _cs, _writeCount, _serialBaud;
    boolean _isInitState, _serialDebug;
    char _fileNameBuffer[20];

    String getIncrementedFileName(String name, String extension);
};

#endif
