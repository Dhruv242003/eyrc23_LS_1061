#include <SPI.h>
#include <Wire.h>
#include <Encoder.h>
#include <MPU6050_light.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "functions.h"

/*************************************************************************************
   Function Name: setup
   Input: None
   Output: None
   Logic: Initializes various components and settings at the start of the program
   Example Call: setup();
 *************************************************************************************/

void setup() {
  setupTimer1(timerInterval);
  kpR = 0;
  kiR = 0;
  kdR = 0;

  kpY = 0;
  kiY = 0;
  kdY = 0;

  radio.begin();
  radio.openReadingPipe(1, address);
  radio.startListening();

  vel = 0;
  U = 0.0;

  // Motor setups
  dc_motor_init();
  bo_motor_init();

  // MPU6050 setup
  Wire.begin();
  byte status = mpu.begin();
  printMPUStatus(status);

  // calibrateRobotAngle();
  // mpu.calcOffsets();

  //  mpu.setGyroOffsets(-0.214046 , -1.31, -0.940916);
  //  mpu.setAccOffsets( 0.021778  , -0.02 ,  0.067591);

  // mpu.setGyroOffsets(-0.090294, -1.4525, -1.08892625);
  // mpu.setAccOffsets(0.031191375, -0.0083, 0.0602215);

  mpu.setGyroOffsets(1.208336, -1.65, -1.592856);
  mpu.setAccOffsets(0.037885, 0.01, 0.065807);

  Serial.begin(9600);

  Hall_init();
  led_init();
  buzzer_init();
  forwardBO(0);
  backwardBO(0);
}

void loop() {
  mpu.update();
  roll_yaw_indicator();
  if (radio.available()) 
  {
    DataPacket receivedValues;
    radio.read(&receivedValues, sizeof(receivedValues));


    kpR = receivedValues.array[0];
    kiR = receivedValues.array[1];
    kdR = receivedValues.array[2];
    kpY = receivedValues.array[3];
    kiY = receivedValues.array[4];
    kdY = receivedValues.array[5];

    // uncomment if you want to send values from joystick.
        joyX = receivedValues.x;
        joyY = receivedValues.y;
//        packet_debug();

  }
//  Serial.println(yaw);
  
}

ISR(TIMER1_COMPA_vect) 
{
  sensing();
  actuate();
}

void actuate() {

/////////// TRAVERSING  //////////////////

  if (isTraversing) {

    if (joyX > 550) {
      bo_speed = map(joyX, 455 , 700 , 0 , 255);
      forwardBO(bo_speed);
      // Serial.print("forward");
    }
    else if (joyX < 300) {
      bo_speed = map(joyX, 455 , 0 , 0 , 255);
      backwardBO(bo_speed);
      //      backwardBO(190);
    }
    else {
      forwardBO(0);
      backwardBO(0);
    }

     if (joyY > 550) yaw_joy += 0.38;
     else if (joyY < 300) yaw_joy -= 0.38;
  }

  ////////// PID Controller /////////////
  //////////  CASCADED      ////////

  double rollOffSet = 1.25;

  double error_yaw = yaw_joy - yaw;
  
  outputY = Compute_yaw(error_yaw, kpY, kiY, kdY);
  double error_roll = (rollOffSet - x) - outputY;
  //    Serial.println(x);
  outputR = Compute_roll(error_roll, kpR, kiR, kdR);

  U = outputR;

  //////   PARALLEL   ///////

  // double error_yaw = 0 - yaw;
  // outputY = Compute_yaw(error_yaw, kpY, kiY, kdY);
  // double error_roll = (rollOffSet - x);
  // outputR = Compute_roll(error_roll, kpR, kiR, kdR);

  // U = outputR + outputY;

  // CASCADED apna tarika
  // double error_yaw = 0 - yaw;
  // rollSetpoint = rollOffSet;
  // outputY = Compute_yaw(error_yaw, kpY, kiY, kdY);
  // rollSetpoint += outputY;
  // double error_roll = rollSetpoint - x;
  // // //    Serial.println(x);
  // outputR = Compute_roll(error_roll, kpR, kiR, kdR);

  // U = outputR;
  if (U < 0) 
  {
    vel = constrain(U, -255, -55);
  } 
  else if (U > 0) 
  {
    vel = constrain(U, 55, 255);
  } 
  else 
  {
    vel = 0;
  }
  
  actuate(vel);
}