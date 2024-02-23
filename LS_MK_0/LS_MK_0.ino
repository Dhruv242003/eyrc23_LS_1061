#include <SPI.h>
#include <Wire.h>
#include <Encoder.h>
#include <MPU6050_light.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "Motors.h"
#include "nrf.h"


void setup(){
    Serial.begin(9600);
    void nrf_setup();
    // Motor setups
    motors_init();
}

void loop(){
    // actuate_DC(-100);
    // Serial.println(myEnc.read()); 
    void get_NRF_Gains();
    Serial.println(STATIC_KP_ROLL);
}


