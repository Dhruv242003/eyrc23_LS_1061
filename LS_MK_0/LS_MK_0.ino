#include <SPI.h>
#include <Wire.h>
#include <Encoder.h>
#include <MPU6050_light.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "Motors.h"



void setup(){
    Serial.begin(9600);
    
    // Motor setups
    dc_motor_init();
    bo_motor_init();
}

void loop(){
    actuate_DC(100);
}


