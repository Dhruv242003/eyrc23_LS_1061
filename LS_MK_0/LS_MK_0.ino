#include <SPI.h>
#include<Wire.h>

#include "I2Cdev.h"
//#include "MPU6050.h"
#include "Motors.h"

#include "nrf.h"
//#include "read_angle.h"
#include "angle.h"
#include "Compute_PID.h"
#include "timers.h"


void setup() {
  Serial.begin(9600);
  nrf_setup();
  motors_init();
  mpu_init();
  timerSetup();
}

void loop() {
  mpu.update();
//  readSensor();
  Serial.print(ROLL);
  Serial.print(" ");
  // YAW = getEncoderCount();
  Serial.println(YAW);
  
  // Serial.println(ticks);
  //  delay(10);
}
