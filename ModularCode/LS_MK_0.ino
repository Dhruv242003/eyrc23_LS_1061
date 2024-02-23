#include <SPI.h>
#include <Wire.h>
#include <Encoder.h>
#include <MPU6050_light.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "Motors.h"


// LED PINS
#define LED_GREEN A0
#define LED_RED A1

// BUZZER
#define BUZZER 7

// HALL EFFECT
#define HALL_RIGHT A6
#define HALL_LEFT A7

// IMU
MPU6050 mpu(Wire);

struct DataPacket {
  float array[6];
  int x;
  int y;
  int b1;
  int b2;
};


void setup(){
    Serial.begin(9600);
    radio.begin();
    radio.openReadingPipe(1, address);
    radio.startListening();

    // Motor setups
    dc_motor_init();
    bo_motor_init();

    // MPU6050 setup
    Wire.begin();
    byte status = mpu.begin();
    printMPUStatus(status);

    mpu.setGyroOffsets(-3.35, -3.16, -0.54);
    mpu.setAccOffsets(0.10, 0.00, -0.01);

    Hall_init();
    led_init();
    buzzer_init();

}

// Init Functions
void dc_motor_init() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

void bo_motor_init() {
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void Hall_init() {
  pinMode(HALL_RIGHT, INPUT);
  pinMode(HALL_LEFT, INPUT);
}

void led_init() {
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
}

void buzzer_init() {
  pinMode(BUZZER, OUTPUT);
}

// BO Motor Control
void forwardBO(int speed) {
  analogWrite(enB, speed);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void backwardBO(int speed) {
  analogWrite(enB, speed);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

// Print MPU Status
void printMPUStatus(byte status) {
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {
  }
}
