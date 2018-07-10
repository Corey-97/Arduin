#include "Arduino.h"
#include "constants.h"
#include <Wire.h>
#include <SPI.h>
#include "BMP_280.h"

/* Parameter coefficients for the temp sensor */
uint16_t param_T1;
int16_t  param_T2;
int16_t  param_T3;

/* Parameter coefficients for the pressure sensor */
uint16_t param_P1;
int16_t  param_P2;
int16_t  param_P3;
int16_t  param_P4;
int16_t  param_P5;
int16_t  param_P6;
int16_t  param_P7;
int16_t  param_P8;
int16_t  param_P9;

/* Global variables */
int64_t t_fine;
float alt_offset;
bool initialising = true;

bool init_BMP() {
  readCoefficients();
  write8(BMP280_REGISTER_CONTROL, 0x3F);

  for (int i = 0; i < 1000; i++){
    alt_offset += readAltitude();
  }
  alt_offset /= 1000;
  
  initialising = false;
  return true;
}


float readPressure(void) {
  int64_t var1, var2, p;

  // Must be done first to get the t_fine variable set up
  readTemperature();

  int32_t adc_P = read24(BMP280_REGISTER_PRESSUREDATA);
  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)param_P6;
  var2 = var2 + ((var1*(int64_t)param_P5)<<17);
  var2 = var2 + (((int64_t)param_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)param_P3)>>8) +
    ((var1 * (int64_t)param_P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)param_P1)>>33;

  if (var1 == 0) {
    return 0;  // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125) / var1;
  var1 = (((int64_t)param_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)param_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)param_P7)<<4);
  return (float)p/256;
}

float readAltitude() {
  float altitude;

  float pressure = readPressure(); // in Si units for Pascal
  pressure /= 100;

  altitude = 44330 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 0.1903));

  if (!initialising){
    return altitude - alt_offset;
  }else{
    return altitude;
  }
  
}

float readTemperature(void)
{
  int32_t var1, var2;

  int32_t adc_T = read24(BMP280_REGISTER_TEMPDATA);
  adc_T >>= 4;

  var1  = ((((adc_T>>3) - ((int32_t)param_T1 <<1))) * ((int32_t)param_T2)) >> 11;
  var2  = (((((adc_T>>4) - ((int32_t)param_T1)) * ((adc_T>>4) - ((int32_t)param_T1))) >> 12) * ((int32_t)param_T3)) >> 14;

  t_fine = var1 + var2;

  float T  = (t_fine * 5 + 128) >> 8;
  return T/100;
}



void write8(byte reg, byte value){
  
  Wire.beginTransmission((uint8_t)BMP280_R);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
  Wire.endTransmission();
}

uint16_t read16(byte reg){
  uint16_t value;

  Wire.beginTransmission((uint8_t)BMP280_R);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)BMP280_R, (byte)2);
  value = (Wire.read() << 8) | Wire.read();

  return value;
}

uint16_t read16_LE(byte reg) {
  uint16_t temp = read16(reg);
  return (temp >> 8) | (temp << 8);
}

int16_t readS16(byte reg)
{
  return (int16_t)read16(reg);

}

int16_t readS16_LE(byte reg)
{
  return (int16_t)read16_LE(reg);

}


uint32_t read24(byte reg)
{
  uint32_t value;

  Wire.beginTransmission((uint8_t)BMP280_R);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)BMP280_R, (byte)3);
  
  value = Wire.read();
  value <<= 8;
  value |= Wire.read();
  value <<= 8;
  value |= Wire.read();

  return value;
}

/*
 * @brief: gets the 
 */
void readCoefficients(void)
{
    param_T1 = read16_LE(BMP280_REGISTER_DIG_T1);
    param_T2 = readS16_LE(BMP280_REGISTER_DIG_T2);
    param_T3 = readS16_LE(BMP280_REGISTER_DIG_T3);

    param_P1 = read16_LE(BMP280_REGISTER_DIG_P1);
    param_P2 = readS16_LE(BMP280_REGISTER_DIG_P2);
    param_P3 = readS16_LE(BMP280_REGISTER_DIG_P3);
    param_P4 = readS16_LE(BMP280_REGISTER_DIG_P4);
    param_P5 = readS16_LE(BMP280_REGISTER_DIG_P5);
    param_P6 = readS16_LE(BMP280_REGISTER_DIG_P6);
    param_P7 = readS16_LE(BMP280_REGISTER_DIG_P7);
    param_P8 = readS16_LE(BMP280_REGISTER_DIG_P8);
    param_P9 = readS16_LE(BMP280_REGISTER_DIG_P9);
}
