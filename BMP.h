#ifndef BMP_H_   /* Include guard */
#define BMP_H_

#include <Wire.h>
#include "constants.h"
#define byte uint8_t

void init_BMP(void);
float getTemperature(void);
float getPressure(void);
float getAltitude(void);
void initCoefficients();
void write8(byte reg, byte value);
uint16_t read16(byte reg);
uint16_t read16_LE(byte reg);
uint32_t read24(byte reg);
int16_t readS16(byte reg);
int16_t readS16_LE(byte reg);

#endif 
