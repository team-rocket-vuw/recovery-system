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
    Data_module(int chipSelect, int serialBaud = 115200, String initFileName = "init/init", String dataFileName = "data/data");

      boolean initialize();
      void setDebugMode(boolean isDebugMode);

      void print(String stringToPrint);
      void println(String stringToPrint);

      void initComplete();

      void flushBuffer();

    private:
      int _cs, _serialBaud;
      String _initFileName, _dataFileName, _dataBuffer;
      int _writeCount;
      boolean _isInitState = true;
      boolean _serialDebug = false;
      char _fileNameBuffer[20];

      String getIncrementedFileName(String name, String extension);
};

#endif
