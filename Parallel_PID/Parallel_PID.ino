#include <Wire.h>
#include <PID_v1.h>
#include <Encoder.h>
#include <MPU6050_light.h>
#include <nRF24L01.h>
#include <RF24.h>

// MPU6050 sensor object
MPU6050 mpu(Wire);
float pi = 3.14159265359;

// PID controllers for roll and yaw
double rollSetpoint;
double yawSetpoint;

double prev_error = 0;
double error_accumulation = 0;
double angle_increment = 0.0001;
double Imax = 255;

double rollError = 0;
double yawError = 0;

double rollInput, rollOutput;
double yawInput, yawOutput, motorSpeed;

// PID tuning parameters for roll
double kp_roll = 0.0;  // Adjust as needed
double ki_roll = 0.0;  // Adjust as needed
double kd_roll = 0.0;  // Adjust as needed

// PID tuning parameters for yaw
double kp_yaw = 0.0;  // Adjust as needed
double ki_yaw = 0.0;  // Adjust as needed
double kd_yaw = 0.0;  // Adjust as needed

// Geared Motor
#define enA 6
#define in1 A2
#define in2 A3

// BO Motor
#define enB 5
#define in3 9
#define in4 4

Encoder myEnc(2, 3);
const byte switchPin = 2;
double yaw = 0;
double oldYaw = 0;
unsigned long previousMillis = 0;
const unsigned long interval = 100;
double angularVelocity = 0;
long oldPosition = -999;
double x, y, z;
double vel;
double U;


RF24 radio(10, 8);  // CE, CSN
const byte address[6] = "00001";

unsigned long curMillis;
long prevMillis = 0;
unsigned long timePrevious = 0;
double receivedValues[6];
double k[6] = { 0, 0, 0, 0, 0, 0 };
bool vertical = false;

PID rollPID(&rollInput, &rollOutput, &rollSetpoint, kp_roll, ki_roll, kd_roll, DIRECT);
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, kp_yaw, ki_yaw, kd_yaw, DIRECT);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {}  // stop everything if could not connect to MPU6050

  dc_motor_init();

  // Configure PID controllers
  rollPID.SetMode(AUTOMATIC);
  rollPID.SetSampleTime(10);           // Adjust as needed
  rollPID.SetOutputLimits(-255, 255);  // Adjust based on motor range

  yawPID.SetMode(AUTOMATIC);
  yawPID.SetSampleTime(10);           // Adjust as needed
  yawPID.SetOutputLimits(-255, 255);  // Adjust based on motor range

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.startListening();

  timePrevious = millis();
  mpu.setGyroOffsets(0.03, -1.20, -0.82);
  mpu.setAccOffsets(0.05, -0.01, 0.047);
}

void loop() {

  curMillis = millis();
  if (curMillis - prevMillis >= 20) {
    prevMillis = curMillis;
    //Serial.print("Yaw: ");
    Serial.print(yaw * (180 / pi));
    Serial.print(" ");
    Serial.print(yawSetpoint * (180 / pi));
    Serial.println();

    sensing();

    if (radio.available()) {
      radio.read(&receivedValues, sizeof(receivedValues));

      k[0] = receivedValues[0];
      k[1] = receivedValues[1];
      k[2] = receivedValues[2];
      k[3] = receivedValues[3];
      k[4] = receivedValues[4];
      k[5] = receivedValues[5];

      kp_roll = k[0];
      ki_roll = k[1];
      kd_roll = k[2];

      kp_yaw = k[3];
      ki_yaw = k[4];
      kd_yaw = k[5];

      
    }

    if (abs(yaw * (180 / pi)) < 60) {

      ////////// PID Controller /////////////



      

      yawInput = yaw;
      yawSetpoint = 0;
      yawError = yawSetpoint - yawInput;
      yawOutput = computePID(yawError, kp_yaw, ki_yaw, kd_yaw);

      

      U = yawOutput;
      //     vel = constrain(U, -255, 255);

      if (U < 0) {
        vel = constrain(U, -255, -55);
      } else if (U > 0) {
        vel = constrain(U, 55, 255);
      } else {
        vel = 0;
      }
      actuate(vel);

      

    } else {
      actuate(0);
    }

    
  }
}


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

void dc_motor_init() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

void actuate(int pwm) {
  motor_control(-pwm);
}


void sensing() {
  mpu.update();
  x = (mpu.getAngleX()) * (pi / 180);  // Roll
  y = mpu.getAngleY();                 // Pitch
  z = mpu.getAngleZ();                 // Yaw

  /// --------ENCODER YAW -----------
  long newPosition = myEnc.read();
  if (newPosition != oldPosition && newPosition % 2 == 0) {
    oldPosition = newPosition;
    newPosition = newPosition / 2;
    yaw = (newPosition) / 5.824;

    unsigned long currentMillis = millis();
    unsigned long deltaTime = currentMillis - previousMillis;

    if (deltaTime >= interval) {
      angularVelocity = (yaw - oldYaw) / (deltaTime / 1000.0);
      oldYaw = yaw;
      previousMillis = currentMillis;
    }
    yaw = (yaw) * (pi / 180);
  }
}


double computePID(double error, double Kp, double Ki, double Kd) {
  // Compute the error adjustment to the desired angle
  

  // Integrate the error
  error_accumulation += error;

  // Clamp the integrated error
  if (error_accumulation > Imax) {
    error_accumulation = Imax;
  }
  if (error_accumulation < (-Imax)) {
    error_accumulation = -Imax;
  }

  // Approximate the rate of change of the error
  double error_deriv = (error - prev_error);

  // Compute duty cycle with PID controller
  double duty_cycle = (Kp * error) + (Ki * error_accumulation) + (Kd * error_deriv);

  // Update prev_error
  prev_error = error;

  return duty_cycle;
}