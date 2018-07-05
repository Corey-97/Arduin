#include "IMU.h"
#include "BMP.h"
#include <Wire.h>

long loop_timer;
int counter;

void setup() {                                                   
  Serial.begin(115200);
  Serial.println("Initialising I2C devices . . .");
  Wire.begin();  

//  Serial.println("Initialising MPU-6050 . . .");
//  init_IMU();

  Serial.println("Initialising BMP-280 . . .");
  init_BMP();  

  Serial.println("Initialisation Complete . . .");
  loop_timer = micros();      

  Serial.println();
}

void loop(){

//  float temp = getTemperature();
//  float pressure = getPressure();
//  float alt = getAltitude();

  float alt = 0;
  
  for (int i = 0; i <= 100; i++){
    alt += getAltitude();;
  }

  alt /= 100;

  
//  Serial.print("Temperature: "); Serial.println(temp);
//  Serial.print("Pressure: "); Serial.println(pressure);
  Serial.print("Altitude: "); Serial.println(alt/20);

  Serial.println();

   delay(2000);
}

















