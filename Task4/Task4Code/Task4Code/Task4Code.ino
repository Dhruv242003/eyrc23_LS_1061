#include <SPI.h>
#include <Wire.h>
#include <MPU6050_light.h>

//Geared Motor
#define enA 6
#define in1 A2
#define in2 A3

//BO Motor
#define enB 5
#define in3 9
#define in4 4

double pi = 3.14159265359;
float vel;

// IMU
MPU6050 mpu(Wire);
unsigned long timer = 0;


// Control Variables
double x, y, z;
double xx, yy, zz;


//double k[4] = {0, 0 , -6.5*10 , -29.95*30 };

double k[4] = {2*26 , 3.5*18.665 , 2*-6.5 , 3.5*-29.95};


double x1, x2, x3, x4, U;
double error_alpha_dot, error_alpha, error_theta_dot, error_theta;


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


// Motor Control Functions
void motor_control(int pwm) {
  if (pwm < 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    pwm = -pwm;
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  analogWrite(enA, pwm);
}



// BO Motor Control
void forwardBO() {
  analogWrite(enB, 100);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}
void backwardBO() {
  analogWrite(enB, 100);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

//Print MPU Status
void printMPUStatus(byte status) {
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) { } // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
}



//Print Roll,Pitch,Yaw
void printMPUData() {
  if ((millis() - timer) > 10) {
    xx = (mpu.getAngleX() * pi) / 180;
    yy = (mpu.getAngleY() * pi) / 180;
    zz = (mpu.getAngleZ() * pi) / 180;

    Serial.print("X : ");
    Serial.print(xx, 6);
    Serial.print("\tY : ");
    Serial.print(yy, 6);
    Serial.print("\tZ : ");
    Serial.print(zz, 6);

    Serial.print("\tVel_X : ");
    Serial.print(((mpu.getGyroX() * pi) / 180), 6);
    Serial.print("\tVel_Y : ");
    Serial.print(((mpu.getGyroY() * pi) / 180), 6);
    Serial.print("\tVel_Z : ");
    Serial.println(((mpu.getGyroZ() * pi) / 180), 6);


    timer = millis();
  }
}

void setup() {
  Serial.begin(9600);
  vel = 0;
  U = 0.0;
  error_alpha_dot = 0;
  error_alpha = 0;
  error_theta_dot = 0;
  error_theta = 0;


  // Motor setups
  dc_motor_init();
  bo_motor_init();


  // MPU6050 setup
  Wire.begin();
  byte status = mpu.begin();
  printMPUStatus(status);

}

void sensing() {
  // MPU6050 data
  mpu.update();
  //printMPUData();

  x = mpu.getAngleX();   // Roll
  y = mpu.getAngleY();   // Pitch
  z = mpu.getAngleZ();   // Yaw

  error_alpha_dot = 0 - mpu.getGyroZ();
  error_alpha = 0 - z;
  error_theta_dot = 0 - mpu.getGyroX();
  error_theta = 0 - x;


  x1 = -error_alpha_dot;
  x2 = -error_alpha;
  x3 = error_theta_dot;
  x4 = error_theta;

  U = k[0] * x1 + k[1] * x2 + k[2] * x3 + k[3] * x4;
  Serial.print(U);
  Serial.print("      ");
  vel = map(U, -1000, 1000, -255, 255);
  Serial.println(-vel);
}



void actuate() {


  
  // DC Motor control
  
  motor_control(-vel);
  //delay(200);
}



void loop() {
  sensing();
  actuate();
}
