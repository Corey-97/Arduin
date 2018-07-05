/*
 * Datasheet for the BMP280 can be found at https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMP280-DS001-19.pdf
 */


#include <Wire.h>
#include "constants.h"
#include "BMP.h"
#include <math.h>


#define _i2caddr 0x76                   //I2C address for the BMP280
#define byte uint8_t                    //Declartion since Arudino IDE cant parse the standar byte data type

/* Registers for the compensation values for the temperature readings as found in the datasheet */
#define DIG_T1 0x88
#define DIG_T2 0x8A
#define DIG_T3 0x8C

/* Registers for the compensation values for the pressure readings as found in the datasheet */
#define DIG_P1 0x8E
#define DIG_P2 0x90
#define DIG_P3 0x92
#define DIG_P4 0x94
#define DIG_P5 0x96
#define DIG_P6 0x98
#define DIG_P7 0x9A
#define DIG_P8 0x9C
#define DIG_P9 0x9E

#define BMP280_REGISTER_CHIPID 0xD0
#define BMP280_REGISTER_CONTROL 0xF4


/* Variables to hold temperature compensation readings */
uint64_t dig_T1;
uint64_t dig_T2;
uint64_t dig_T3;

/* Variables to hold pressure compensation readings */
uint64_t dig_P1;
uint64_t dig_P2;
uint64_t dig_P3;
uint64_t dig_P4;
uint64_t dig_P5;
uint64_t dig_P6;
uint64_t dig_P7;
uint64_t dig_P8;
uint64_t dig_P9;

/* Globals for the temperature readings */
int32_t t_fine;
float T;


//----------------------------------------------------------------------------------------
//--------------------------Primary functions---------------------------------------------
//----------------------------------------------------------------------------------------


/*
 * Initialise the compensation values for the BMP280
 */
void init_BMP(){
  initCoefficients();
  write8(BMP280_REGISTER_CONTROL, 0x3F);
}

/*
 * brief: Reads the data from the TEMP_DATA_R register and returns the current temperature reading   
 * in degrees C
 * 
 * params: None
 * 
 * returns: Float T/100 as the temperature
 */
float getTemperature(void){
  int32_t var1, var2;

  int32_t adc_T = read24(TEMP_DATA_R);
  adc_T >>= 4;

  var1  = ((((adc_T>>3) - ((int32_t)dig_T1 <<1))) *
     ((int32_t)dig_T2)) >> 11;

  var2  = (((((adc_T>>4) - ((int32_t)dig_T1)) *
       ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) *
     ((int32_t)dig_T3)) >> 14;

  t_fine = var1 + var2;

  T  = (t_fine * 5 + 128) >> 8;
  return (T/100);
}


/*
 * brief: Reads the data from the PRESSURE_DATA_R register and returns the current pressure reading 
 * in Pa
 * 
 * params: None
 * 
 * returns: Float p/256 as the pressure
 */
float getPressure(void) {
  int64_t var1, var2, p;

  // Must be done first to get the t_fine variable set up
  getTemperature();

  int32_t adc_P = read24(PRESSURE_DATA_R);
  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
  var2 = var2 + (((int64_t)dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) +
    ((var1 * (int64_t)dig_P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;

  if (var1 == 0) {
    return 0;  // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125) / var1;
  var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
  return (float)p/256;
}

float getAltitude(void) {
  float altitude;

  float pressure = getPressure(); // in Si units for Pascal
  pressure /= 100;

  altitude = 44330 * (1.0 - pow(pressure / 1013.25, 0.1903));

  return altitude;
}

//----------------------------------------------------------------------------------------
//--------------------------Secondary functions-------------------------------------------
//----------------------------------------------------------------------------------------
void initCoefficients(){
  dig_T1 = read16_LE(DIG_T1);
  dig_T2 = read16_LE(DIG_T2);
  dig_T3 = read16_LE(DIG_T3);

  dig_P1 = read16_LE(DIG_P1);
  dig_P2 = read16_LE(DIG_P2);
  dig_P3 = read16_LE(DIG_P3);
  dig_P4 = read16_LE(DIG_P4);
  dig_P5 = read16_LE(DIG_P5);
  dig_P6 = read16_LE(DIG_P6);
  dig_P7 = read16_LE(DIG_P7);
  dig_P8 = read16_LE(DIG_P8);
  dig_P9 = read16_LE(DIG_P9);
}

void write8(byte reg, byte value){
  
  Wire.beginTransmission((uint8_t)_i2caddr);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
  Wire.endTransmission();
  
}

/*
 * brief: Reads a 16 byte value from the provided register 
 * 
 * param: reg -> register to read from 
 * 
 * returns: value -> value found at the register
*/
uint16_t read16(byte reg){
  uint16_t value;

  Wire.beginTransmission((uint8_t)_i2caddr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)_i2caddr, (byte)2);
  value = (Wire.read() << 8) | Wire.read();


  return value;
}

uint16_t read16_LE(byte reg) {
  uint16_t temp = read16(reg);
  return (temp >> 8) | (temp << 8);
}

/*
 * brief: Reads a 24 byte value from the provided register 
 * 
 * param: reg -> register to read from 
 * 
 * returns: value -> value found at the register
*/
uint32_t read24(byte reg)
{
  uint32_t value;

  Wire.beginTransmission((uint8_t)_i2caddr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)_i2caddr, (byte)3);
  
  value = Wire.read();
  value <<= 8;
  value |= Wire.read();
  value <<= 8;
  value |= Wire.read();


  return value;
}
