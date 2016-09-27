#include "Arduino.h"
#include "Sensor_helper.h"
#include "Data_module.h"
#include "i2c_t3.h"
#include "SPI.h"

Sensor_helper::Sensor_helper(uint8_t AScale, uint8_t GScale, uint8_t MScale, uint8_t Mmode, Data_module * dataModule){
  _gscale = GScale;
  _ascale = AScale;
  _mscale = MScale;
  _mmode = Mmode;

  _dataModule = dataModule;
}

// REGION SETUP FUNCTIONS

boolean Sensor_helper::setupMPU9250()
{
  _dataModule->println("Reading who-am-i byte of MPU9250");
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250

  _dataModule->println("MPU9250 I AM " + String(c, HEX) + ", I should be " + String(0x71, HEX));

  if (c == 0x71) {
    _dataModule->println("MPU9250 online");
    _dataModule->println("Calibrating...\n\n");

    calibrateMPU9250(_gyroBias, _accelBias);

    _dataModule->println("Accelerometer bias: (mg)");
    _dataModule->println("X: " + String(1000*_accelBias[0]) + " Y: " + String(1000*_accelBias[1]) + " Z: " + String(1000*_accelBias[2]));

    _dataModule->println("Gyro bias: (o/s)");
    _dataModule->println("X: " + String(_gyroBias[0]) + " Y: " + String(_gyroBias[1]) + " Z: " + String(_gyroBias[2]));

    initMPU9250();

    _dataModule->println("\nMPU9250 initialized for active data mode....");
    return true;
  } else {
    _dataModule->println("MPU9250 failed to initialise");
    return false;
  }
}

boolean Sensor_helper::setupAK8963()
{
  _dataModule->println("Reading who-am-i byte of magnetometer");
  byte d = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for AK8963
  _dataModule->println("AK8963 I AM " + String(d, HEX) + ", I should be " + String(0x48, HEX));

  if (d == 0x48) {
    initAK8963(_magCalibration);

    _dataModule->println("Calibrating...");
    _dataModule->println("X-Axis sensitivity adjustment value " + String(_magCalibration[0], 2));
    _dataModule->println("Y-Axis sensitivity adjustment value " + String(_magCalibration[1], 2));
    _dataModule->println("Z-Axis sensitivity adjustment value " + String(_magCalibration[2], 2));
    _dataModule->println("\nAK8963 initialized for active data mode....");
  } else {
    _dataModule->println("AK8963 failed to initialise");
    return false;
  }

  return true;
}

boolean Sensor_helper::setupMS5637()
{
  resetMS5637();
  delay(100);
  _dataModule->println("MS5637 pressure sensor reset...");
  // Read PROM data from MS5637 pressure sensor
  readPromMS5637(_pcal);
  _dataModule->println("PROM data read:");
  _dataModule->println("C0 = " + String(_pcal[0]));
  unsigned char refCRC = _pcal[0] >> 12;
  _dataModule->println("C1 = " + String(_pcal[1]));
  _dataModule->println("C2 = " + String(_pcal[2]));
  _dataModule->println("C3 = " + String(_pcal[3]));
  _dataModule->println("C4 = " + String(_pcal[4]));
  _dataModule->println("C5 = " + String(_pcal[5]));
  _dataModule->println("C6 = " + String(_pcal[6]));

  _nCRC = checkMS5637CRC(_pcal);  //calculate checksum to ensure integrity of MS5637 calibration data
  _dataModule->println("Checksum: " + String(_nCRC) + ", should be: " + String(refCRC));

  if (_nCRC != refCRC) {
    _dataModule->println("MS5637 checksum integrity check failed");
    return false;
  }

  // Calculate offset for relative altitude
  float altitudeTemp = 0;
  for(int i = 0; i < 16; i++) {
    altitudeTemp += getAltitude();
  }

  _altitudeOffset = altitudeTemp/16;

  return true;
}

boolean Sensor_helper::setupL3G4200D(int8_t l3gScale, int8_t chipSelect)
{
  _l3gScale = l3gScale;
  _l3gChipSelect = chipSelect;
  return initL3G4200D(_l3gScale);
}

boolean Sensor_helper::initL3G4200D(int8_t scale)
{
  // The WHO_AM_I register should read 0xD3
  if(readSPIRegister(L3G4200D_WHO_AM_I, _l3gChipSelect)!=0xD3) {
    return false;
  }

  // Enable x, y, z and turn off power down:
  writeSPIRegister(L3G4200D_CTRL_REG1, 0b00001111, _l3gChipSelect);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeSPIRegister(L3G4200D_CTRL_REG2, 0b00000000, _l3gChipSelect);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeSPIRegister(L3G4200D_CTRL_REG3, 0b00001000, _l3gChipSelect);

  // CTRL_REG4 controls the full-scale range, among other things:
  scale &= 0x03;
  writeSPIRegister(L3G4200D_CTRL_REG4, scale<<4, _l3gChipSelect);

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  writeSPIRegister(L3G4200D_CTRL_REG5, 0b00000000, _l3gChipSelect);

  return true;
}

void Sensor_helper::initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128)/256.0 + 1.0;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128)/256.0 + 1.0;
  destination[2] =  (float)(rawData[2] - 128)/256.0 + 1.0;
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, _mscale << 4 | _mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}

void Sensor_helper::initMPU9250()
{
  // wake up device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Wait for all registers to reset

 // get stable time source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200);

 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
                                    // determined inset in CONFIG above

 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value

 // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x02; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | _gscale << 3; // Set full scale range for the gyro
 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

 // Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | _ascale << 3; // Set full scale range for the accelerometer
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
   writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
   delay(100);
}

void Sensor_helper::calibrateMPU9250(float * dest1, float * dest2)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // reset device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

  // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator, bits 2:0 = 001
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
  delay(200);

  // Configure device for bias calculation
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

  }

  accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

  if(accel_bias[2] > 0L)
  {
    accel_bias[2] -= (int32_t) accelsensitivity;
  }  // Remove gravity from the z-axis accelerometer bias calculation
  else
  {
    accel_bias[2] += (int32_t) accelsensitivity;
  }

  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

  // Push gyro biases to hardware registers
  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

  // Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for(ii = 0; ii < 3; ii++) {
    if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Apparently this is not working for the acceleration biases in the MPU-9250
  // Are we handling the temperature correction bit properly?
  // Push accelerometer biases to hardware registers
  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

  // Output scaled accelerometer biases for display in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;

}

// END REGION

// REGION SENSOR VALUE RETRIEVING FUNCTIONS

float Sensor_helper::getAltitude()
{
  _D1 = MS5637Read(ADC_D1, OSR);  // get raw pressure value
  _D2 = MS5637Read(ADC_D2, OSR);  // get raw temperature value
  _dT = _D2 - _pcal[5]*pow(2,8);    // calculate temperature difference from reference
  _OFFSET = _pcal[2]*pow(2, 17) + _dT*_pcal[4]/pow(2,6);
  _SENS = _pcal[1]*pow(2,16) + _dT*_pcal[3]/pow(2,7);

  _temperature = (2000 + (_dT*_pcal[6])/pow(2, 23))/100;   // First-order Temperature in degrees celsius

  // Second order corrections
  if(_temperature > 20)
  {
    _T2      = 5*_dT*_dT/pow(2, 38); // correction for high temperatures
    _OFFSET2 = 0;
    _SENS2   = 0;
  } else if(_temperature < 20)       // correction for low temperature
  {
    _T2      = 3*_dT*_dT/pow(2, 33);
    _OFFSET2 = 61*(100*_temperature - 2000)*(100*_temperature - 2000)/16;
    _SENS2   = 29*(100*_temperature - 2000)*(100*_temperature - 2000)/16;
  } else if(_temperature < -15)      // correction for very low temperature
  {
    _OFFSET2 = _OFFSET2 + 17*(100*_temperature + 1500)*(100*_temperature + 1500);
    _SENS2 = _SENS2 + 9*(100*_temperature + 1500)*(100*_temperature + 1500);
  }
  // End of second order corrections

  _temperature = _temperature - _T2/100;
  _OFFSET = _OFFSET - _OFFSET2;
  _SENS = _SENS - _SENS2;

  _pressure = (((_D1*_SENS)/pow(2, 21) - _OFFSET)/pow(2, 15))/100;  // Pressure in mbar or kPa

  // Altitude calculation from sensor charaterisation on datasheet
  return ((145366.45*(1.0 - pow((_pressure/1013.25), 0.190284)))/3.2808) - _altitudeOffset;
}

void Sensor_helper::getIMUAccelData(float * data)
{
  readAccelData(_accelCount);
  _aRes = getAccelRes();

  // Three axis accelerometer reading calculation
  for (int i = 0; i < 3; i++)
  {
    data[i] = (float)_accelCount[i]*_aRes;
  }
}

void Sensor_helper::getIMUGyroData(float * data)
{
  readGyroData(_gyroCount);
  _gRes = getGyroRes();

  // Three axis gyro reading calculation
  for (int i = 0; i < 3; i++)
  {
    data[i] = (float)_gyroCount[i]*_gRes;
  }
}

void Sensor_helper::getIMUMagData(float * data)
{
  readMagData(_magCount);
  _mRes = getMagRes();

  // Calculate the magnetometer values in milliGauss
  // Include factory calibration per data sheet and user environmental corrections
  for (int i = 0; i < 3; i++)
  {
    data[i] = (float) _magCount[0]*_mRes*_magCalibration[0] - _magbias[0];
  }
}

void Sensor_helper::getL3G4200DGyroData(int16_t * destination)
{
  // To read, we request high byte, then bit-shift and mask with low byte
  // providing a 16 bit integer read out

  // reading gyro x-axis measurements
  destination[0] = (readSPIRegister(L3G4200D_OUT_X_H, _l3gChipSelect)&0xFF)<<8;
  destination[0] |= (readSPIRegister(L3G4200D_OUT_X_L, _l3gChipSelect)&0xFF);

  // reading gyro y-axis measurements
  destination[1] = (readSPIRegister(L3G4200D_OUT_Y_H, _l3gChipSelect)&0xFF)<<8;
  destination[1] |= (readSPIRegister(L3G4200D_OUT_Y_L, _l3gChipSelect)&0xFF);

  // reading gyro z-axis measurements
  destination[2] = (readSPIRegister(L3G4200D_OUT_Z_H, _l3gChipSelect)&0xFF)<<8;
  destination[2] |= (readSPIRegister(L3G4200D_OUT_Z_L, _l3gChipSelect)&0xFF);
}

// END REGION

int8_t Sensor_helper::readSPIRegister(byte address, int8_t chipSelect)
{
  int8_t toRead;

  address |= 0x80; // bitmask read command

  digitalWrite(chipSelect, LOW); // pull chip select LOW for transfer
  SPI.transfer(address);         // Push register address
  toRead = SPI.transfer(0x00);   // Send read request flag
  digitalWrite(chipSelect, HIGH);// pull chip select LOW for transfer

  return toRead;
}

void Sensor_helper::writeSPIRegister(byte address, byte payload, int8_t chipSelect)
{
  address &= 0x7F; // bitmask write command

  digitalWrite(chipSelect, LOW); // pull chip select LOW for transfer
  SPI.transfer(address);         // Push register address
  SPI.transfer(payload);         // Push payload
  digitalWrite(chipSelect, HIGH);// pull chip select LOW for transfer
}

void Sensor_helper::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire1.beginTransmission(address);  // Initialize the Tx buffer
  Wire1.write(subAddress);           // Put slave register address in Tx buffer
  Wire1.write(data);                 // Put data in Tx buffer
  Wire1.endTransmission();           // Send the Tx buffer
}

void Sensor_helper::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
  Wire1.beginTransmission(address);   // Initialize the Tx buffer
  Wire1.write(subAddress);            // Put slave register address in Tx buffer
  Wire1.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive

  uint8_t i = 0;
  Wire1.requestFrom(address, (size_t) count);  // Read bytes from slave register address
  while (Wire1.available()) {
    dest[i++] = Wire1.read(); // Put read results in the Rx buffer
  }
}

uint8_t Sensor_helper::readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data
  Wire1.beginTransmission(address);         // Initialize the Tx buffer
  Wire1.write(subAddress);                  // Put slave register address in Tx buffer
  Wire1.endTransmission(I2C_NOSTOP, 50);        // Send the Tx buffer, but send a restart to keep connection alive
  Wire1.requestFrom(address, (size_t) 1, 50);  // Read one byte from slave register address
  data = Wire1.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void Sensor_helper::readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void Sensor_helper::readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void Sensor_helper::readMagData(int16_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
    readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
    uint8_t c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
      destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
      destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
      destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
    }
  }
}

// Altimeter sensor region
void Sensor_helper::resetMS5637()
{
  Wire1.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
  Wire1.write(MS5637_RESET);                // Put reset command in Tx buffer
  Wire1.endTransmission();                  // Send the Tx buffer
}

void Sensor_helper::readPromMS5637(uint16_t * destination)
{
  uint8_t data[2] = {0,0};
  for (uint8_t ii = 0; ii < 7; ii++) {
    Wire1.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
    Wire1.write(0xA0 | ii << 1);              // Put PROM address in Tx buffer
    Wire1.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire1.requestFrom(MS5637_ADDRESS, 2);   // Read two bytes from slave PROM address
    while (Wire1.available()) {
      data[i++] = Wire1.read(); }               // Put read results in the Rx buffer
      destination[ii] = (uint16_t) (((uint16_t) data[0] << 8) | data[1]); // construct PROM data for return to main program
    }
  }

uint32_t Sensor_helper::MS5637Read(uint8_t CMD, uint8_t OSR)
{
   uint8_t data[3] = {0,0,0};
   Wire1.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
   Wire1.write(CMD | OSR);                  // Put pressure conversion command in Tx buffer
   Wire1.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive

   switch (OSR)
   {
     case ADC_256: delay(1); break;  // delay for conversion to complete
     case ADC_512: delay(3); break;
     case ADC_1024: delay(4); break;
     case ADC_2048: delay(6); break;
     case ADC_4096: delay(10); break;
     case ADC_8192: delay(20); break;
   }

   Wire1.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
   Wire1.write(0x00);                        // Put ADC read command in Tx buffer
   Wire1.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
   uint8_t i = 0;
   Wire1.requestFrom(MS5637_ADDRESS, 3);     // Read three bytes from slave PROM address
   while (Wire1.available()) {
     data[i++] = Wire1.read(); // Put read results in the Rx buffer
   }

   return (uint32_t) (((uint32_t) data[0] << 16) | (uint32_t) data[1] << 8 | data[2]); // construct PROM data for return to main program
}

// Full disclosure: This function was copy-pasted from https://github.com/kriswiner/MPU-9250/blob/master/MPU9250_MS5637_AHRS_t3.ino
// It looks extremely convoluted, but it works so whatever
/*
          /     ^^^^^^^^^            ^^^^^^^^^    \
         /    ^     ^   ^          ^     ^   ^     \
        |             ^       |             ^       |
        |                     |D                    |
         \                                         /
          \        \____________________/         /    J$
*/
unsigned char Sensor_helper::checkMS5637CRC(uint16_t * n_prom)
{
  int cnt;
  unsigned int n_rem = 0;
  unsigned char n_bit;

  n_prom[0] = ((n_prom[0]) & 0x0FFF);  // replace CRC byte by 0 for checksum calculation
  n_prom[7] = 0;
  for(cnt = 0; cnt < 16; cnt++)
  {
    if(cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
    else         n_rem ^= (unsigned short)  (n_prom[cnt>>1]>>8);
    for(n_bit = 8; n_bit > 0; n_bit--)
    {
        if(n_rem & 0x8000)    n_rem = (n_rem<<1) ^ 0x3000;
        else                  n_rem = (n_rem<<1);
    }
  }
  n_rem = ((n_rem>>12) & 0x000F);
  return (n_rem ^ 0x00);
}
// End region

float Sensor_helper::getAccelRes()
{
  switch (_ascale)
  {
   // Possible accelerometer scales (and their register bit settings) are:
   // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
   // Here's a bit of an algorithm to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          return 2.0/32768.0;
    case AFS_4G:
          return 4.0/32768.0;
    case AFS_8G:
          return 8.0/32768.0;
    case AFS_16G:
          return 16.0/32768.0;
    default:
          return 0;
  }
}

float Sensor_helper::getGyroRes()
{
  switch (_gscale)
  {
 	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
  // Here's a bit of an algorithm to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          return 250.0/32768.0;
    case GFS_500DPS:
          return 500.0/32768.0;
    case GFS_1000DPS:
          return 1000.0/32768.0;
    case GFS_2000DPS:
          return 2000.0/32768.0;
    default:
          return 0;
  }
}

float Sensor_helper::getMagRes()
{
  switch (_mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          return 10.*4912./8190.;
    case MFS_16BITS:
          return 10.*4912./32760.0;
    default:
          return 0;
  }
}
