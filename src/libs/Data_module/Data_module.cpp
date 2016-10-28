#include "Arduino.h"
#include <SD.h>
#include <SPI.h>
#include "Data_module.h"

/*
  Setting up library. Pass in SD card chip select, serial baud rate, and filenames
*/
Data_module::Data_module(int chipSelect, int serialBaud, String initFileName, String dataFileName) :_cs(chipSelect),
                                                                                                    _serialBaud(serialBaud),
                                                                                                    _initFileName(initFileName),
                                                                                                    _dataFileName(dataFileName)
{}


boolean Data_module::initialize()
{
  if (_serialDebug) {
    Serial.begin(_serialBaud);
    while(!Serial.available()){} // block if in debug mode
    return true;
  } else {

    //SD card setup
    if (!SD.begin(_cs)) {
      println("Card failed to initialise");
      return false;
    } else {
      println("Initialising card");
      _initFileName = getIncrementedFileName(_initFileName, "txt");
      _dataFileName = getIncrementedFileName(_dataFileName, "csv");
      return true;
    }
  }

}

// NOTE: Must be called before initialize()
void Data_module::setDebugMode(boolean isDebugMode)
{
  _serialDebug = isDebugMode;
}

void Data_module::initComplete()
{
  _isInitState = false;
  flushBuffer();
}

String Data_module::getIncrementedFileName(String name, String extension)
{
  int fileCount = 1;
  (name + String(fileCount) + "." + extension).toCharArray(_fileNameBuffer, 20);
  while(SD.exists(_fileNameBuffer)) {
    fileCount += 1;
    (name + String(fileCount) + "." + extension).toCharArray(_fileNameBuffer, 20);
  }

  return name + String(fileCount) + "." + extension;
}


void Data_module::flushBuffer()
{
  if (!_serialDebug) {
    // Set up filename buffer
    if (_isInitState) {
      _initFileName.toCharArray(_fileNameBuffer, 20);
    } else {
      _dataFileName.toCharArray(_fileNameBuffer, 20);
    }

    // flushes buffer and writes to SD card
    File _datafile = SD.open(_fileNameBuffer, FILE_WRITE);
    // Check if file opened correctly
    if (_datafile) {
      _datafile.print(_dataBuffer);
      _datafile.close();
    }

    _writeCount = 0;
    _dataBuffer = "";
  }
}

void Data_module::print(String toPrint)
{
  if (_serialDebug) {
    Serial.print(toPrint);
  } else {
    _writeCount++;
    _dataBuffer += toPrint;

    if (_writeCount == 64) {
      flushBuffer();
    }
  }
}

void Data_module::println(String toPrint)
{
  print(String(toPrint + "\n"));
}
