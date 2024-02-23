#include <SPI.h>
#include<Wire.h>

#include <Wire.h>
#include "I2Cdev.h"
//#include "MPU6050.h"
#include "nrf.h"
//#include "read_angle.h"
#include "angle.h"
#include "Compute_PID.h"
#include "Indicators.h"
#include "Motors.h"
#include "timers.h"


void setup() {
  Serial.begin(9600);
  nrf_setup();
  motors_init();
  mpu_init();
  timerSetup();
  Hall_init();
  led_init();
  buzzer_init();
}

void loop() {
  mpu.update();

  Serial.print(ROLL);
  Serial.print(" ");
  Serial.println(YAW);
  detect_magnet();
}
