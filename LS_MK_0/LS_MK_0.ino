#include <SPI.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include<Wire.h>

#include "Motors.h"
#include "nrf.h"
#include "read_angle.h"
// #include "Compute_PID.h"


void setup(){
    Serial.begin(9600);
    nrf_setup();
    motors_init();
    mpu_init();
}

void loop(){
    // actuate_DC(-100);
    // Serial.println(myEnc.read()); 
    // get_NRF_Gains();
    // Serial.println(STATIC_KP_ROLL);
    readSensor();
    Serial.println(roll);
    delay(100);
}


