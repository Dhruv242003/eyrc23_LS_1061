#include <MPU6050.h>

MPU6050 mpu;

int16_t ax, ay, az, gx, gy, gz;
float a[3] = {0, 0, 0}, g[3] = {0, 0, 0};
const float pi = 3.14159265359, comp_alpha = 0.02, dT = 0.003;
double ROLL = 0;

void mpu_init() ;
void readSensor();
double complimentary_filter_roll();

void mpu_init() {
  Wire.begin();
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}


void readSensor() {
  mpu.getAcceleration(&ax, &ay, &az);
  a[0] = ax / 16384.0;
  a[1] = ay / 16384.0;
  a[2] = az / 16384.0;

  mpu.getRotation(&gx, &gy, &gz);
  g[0] = gx / 131.0;
  g[1] = gy / 131.0;
  g[2] = gz / 131.0;

  ROLL = complimentary_filter_roll();
}

double complimentary_filter_roll() {
 return (1 - comp_alpha) * (ROLL + g[0] * dT) + (comp_alpha) * (atan(a[1] / abs(a[2]))) * (180.0 / pi);
}
