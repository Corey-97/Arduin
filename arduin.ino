#include "IMU.h"
#include "BMP_280.h"
#include <Wire.h>
#include <SPI.h>

#define STD_DEV 0.12

int timer_counter;
int counter;
float alt_rolling;

float desired_alt = 1.00;

void setup() {                                                  
  Serial.begin(115200);
  Serial.println("Initialising I2C devices . . .");
  Wire.begin();  

  Serial.println("Initialising MPU-6050 . . .");
  init_IMU();                         //This takes fucking forever 

  Serial.println("Initialising BMP-280 . . .");
  if (!init_BMP()) {Serial.println("Error initialising the BMP-280, check connection and try again"); }  

  Serial.println("Initialisation Complete . . .");  
  Serial.println();

  timer_counter = millis();
}

/*
 * Since this program is timer interupt driven, this is used as the idle task function
 * Might be used later to transmit data to the trasmitter when I get around to building it
 */
void loop(){
  float alt = 0.00;
  float alt_up = 0.00;
  
  for (int i = 0; i < 5; i++){
    alt_up = readAltitude();
    alt += alt_up;
  }

  alt /= 5;

  if (alt > desired_alt + 3 * ALT_STD_DEV){
    Serial.println("Needs to go down");
  }else if (alt < desired_alt - 3 * ALT_STD_DEV){
    Serial.println("Needs to go up");
  }else{
    Serial.println("steady");
  }

  

  
  
  delay(100);
  //Stop until enough time has elapsed so that the code runs at desired rate
//  while(millis() - timer_counter < 1000 / TIMER_INTERUPT_FREQ);                
//  timer_counter = millis();   
}


















