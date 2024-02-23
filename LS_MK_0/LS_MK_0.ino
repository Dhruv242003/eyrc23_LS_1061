#include <SPI.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include<Wire.h>

#include "Motors.h"
#include "nrf.h"
#include "read_angle.h"
#include "Compute_PID.h"
#include "timers.h"


void setup(){
    Serial.begin(9600);
    nrf_setup();
    motors_init();
    // mpu_init();
    timerSetup();
}

void loop(){
    
//    delay(100);
}
