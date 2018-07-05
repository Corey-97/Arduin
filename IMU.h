#ifndef IMU_H_   /* Include guard */
#define IMU_H_

#include "constants.h"

#include <Wire.h>
#include <stdbool.h>
#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include <util/delay.h>




void init_IMU();

void calibrate_Gyro();

void setup_mpu_6050_registers();

float get_roll();

float get_pitch();

void read_mpu_6050_data();


bool Wait(const unsigned long &Time);


#endif // IMU_H_
