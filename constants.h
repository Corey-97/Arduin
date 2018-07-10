#ifndef CONSTANTS_H
#define CONSTANTS_H

#define TIMER_INTERUPT_FREQ 10
#define UNO_CLOCK_SPEED 16*10^6
//IMU Constants ----------------------------------
#define IMU_DATA_R 0x68                   //Register for the MPU6050

//BMP280 Constants -------------------------------
#define BMP280_R 0x76
#define PRESSURE_DATA_R 0xF7               //Regisiter for the Pressure reading
#define TEMP_DATA_R 0xFA                   //Regisiter for the Temperature reading
#define SEA_LEVEL_PRESSURE 1013.25         //Sea Level pressure
#define ALT_STD_DEV 0.12                       //Standard deviation of altitude return


#endif 
