#include <SPI.h>
#include<Wire.h>
#include <Wire.h>
#include "globals.h"
#include "nrf.h"
//#include "read_angle.h"
#include "Motors.h"
#include "angle.h"
#include "Compute_PID.h"
#include "Indicators.h"
#include "timers.h"
#include "controller.h"

void setup() {
  Serial.begin(9600);
  setup_all();
}

void loop() {
  mpu.update();

  // Serial.println(ROLL);
  // Serial.print(" ");
  // Serial.println(YAW);f
  
  // roll_yaw_indicator();
  get_NRF_Gains();
  scheduler();
}

void setup_all(){
  nrf_setup();
  motors_init();
  mpu_init();
  timerSetup();
  Hall_init();
  led_init();
  buzzer_init();
}
