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
#include "controller.h"
#include "timers.h"

void setup() {
  Serial.begin(9600);
  setup_all();
}

void loop() {
  mpu.update();

  // Serial.println(ROLL);
  // Serial.print(" ");
  // Serial.println(YAW);

  // roll_yaw_indicator();   // Using LED's for indicating roll and pitch for making tuninig easy
  get_NRF_Gains();
  detect_magnet();
  runIndicator();
 
  // ERS();  // EMERGENCY RESPONSE SYSTEM for manual interventions
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
