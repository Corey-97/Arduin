#include "IMU.h"
#include "constants.h"
#include "Arduino.h"

#include <Wire.h>
#include <stdbool.h>
#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include <util/delay.h>

/*
 * Declare the global variables
 */
//Declaring some global variables
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
bool set_gyro_roll, set_gyro_pitch;

long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;

int temp;

/*
 * Initialise the MPU-6050
 */
void init_IMU(){
  setup_mpu_6050_registers();
  calibrate_Gyro();
}

/*
 * Prepare the registers for the IMU
 */
void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(IMU_DATA_R);                                        
  Wire.write(0x6B);                                                    
  Wire.write(0x00);                                                   
  Wire.endTransmission();      
                                         
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(IMU_DATA_R);                                        
  Wire.write(0x1C);                                                   
  Wire.write(0x10);                                                   
  Wire.endTransmission();   
                                            
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(IMU_DATA_R);                                       
  Wire.write(0x1B);                                                    
  Wire.write(0x08);                                                 
  Wire.endTransmission();                                             
}

/*
 * Read the raw gyro data 2000 times and take an average of the data to find the calibratio offset
 */
void calibrate_Gyro(){
  int cal_count = 1000;
  for (int cal_int = 0; cal_int < cal_count ; cal_int ++){                  //Read the raw acc and gyro data from the MPU-6050 for 1000 times
    read_mpu_6050_data();                                             
    gyro_x_cal += gyro_x;                                              //Add the gyro x offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                              //Add the gyro y offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                              //Add the gyro z offset to the gyro_z_cal variable

    delay(3);
//    _delay_ms (3);
  }

  // divide by 1000 to get avarage offset
  gyro_x_cal /= cal_count;                                                 
  gyro_y_cal /= cal_count;                                                 
  gyro_z_cal /= cal_count; 
}


/*
 * Get the current roll angle of the quatcopter
 */
float get_roll(){
  read_mpu_6050_data(); 

  //Subtract the offset values from the raw gyro values
  gyro_x -= gyro_x_cal;                                                
  gyro_y -= gyro_y_cal;                                                
  gyro_z -= gyro_z_cal;                                                
         
  //Gyro angle calculations . Note 0.0000611 = 1 / (250Hz x 65.5)
  angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  angle_pitch_acc -= -0.54;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= -4.44;                                               //Accelerometer calibration value for roll

  if(set_gyro_roll){                                                 //If the IMU is already started
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_roll = true;                                            //Set the IMU started flag
  }
  
  //To dampen the roll angles a complementary filter is used
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value

  return angle_roll_output;
}


/*
 * Get the current pitch angle of the quadcopter
 */
float get_pitch(){
  read_mpu_6050_data();   
 //Subtract the offset values from the raw gyro values
  gyro_x -= gyro_x_cal;                                                
  gyro_y -= gyro_y_cal;                                                
  gyro_z -= gyro_z_cal;                                                
         
  //Gyro angle calculations . Note 0.0000611 = 1 / (250Hz x 65.5)
  angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
   
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
   
  angle_pitch_acc -= -0.54;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= -4.44;                                               //Accelerometer calibration value for roll

  if(set_gyro_pitch){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
  }
  else{                                                                //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    set_gyro_pitch = true;                                            //Set the IMU started flag
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value

  return angle_pitch_output;
}



void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(IMU_DATA_R);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(IMU_DATA_R,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  
  acc_y = Wire.read()<<8|Wire.read();                                  
  acc_z = Wire.read()<<8|Wire.read();                                  
  temp = Wire.read()<<8|Wire.read();                                   
  gyro_x = Wire.read()<<8|Wire.read();                                 
  gyro_y = Wire.read()<<8|Wire.read();                                 
  gyro_z = Wire.read()<<8|Wire.read();                                 
}



