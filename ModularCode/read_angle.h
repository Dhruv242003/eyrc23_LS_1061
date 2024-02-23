#include "I2Cdev.h"
#include "MPU6050.h"
#include<Wire.h>

int16_t ax, ay, az, gx, gy, gz, gnx, gny, gnz;
float a[3] = {0, 0, 0}, g[3] = {0, 0, 0};

const float pi = 3.14159265359, f_cut = 5, dT = 0.003, comp_alpha = 0.02;
float roll = 0;

const float accel_sf = 16384, gyro_sf = 131;
/******************/
MPU6050 mpu;

void readSensor()
{
  sei();
  read_accel();
  read_gyro();
  complimentary_filter_roll();
}

void test()
{
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void set_offsets()
{
  mpu.setXAccelOffset(-5699);
  mpu.setYAccelOffset(-645);
  mpu.setZAccelOffset(1237);

  mpu.setXGyroOffset(-104);
  mpu.setYGyroOffset(15);
  mpu.setZGyroOffset(-10);
}

void read_accel()
{
  mpu.getAcceleration(&ax, &ay, &az);
  a[0] = (ax / accel_sf);
  a[1] = (ay / accel_sf);
  a[2] = (az / accel_sf);
}

void read_gyro()
{
  mpu.getRotation(&gx, &gy, &gz);
  g[0] = (gx / gyro_sf);
  g[1] = (gy / gyro_sf);
  g[2] = (gz / gyro_sf);
}

void complimentary_filter_roll()
{
  roll = (1 - comp_alpha) * (roll_deg + g[1] * dT) + (comp_alpha) * (atan(a[0] / abs(a[2]))) * (180 / 3.14);
}

void mpu_init()
{
  mpu.initialize();
  test();
  mpu.setFullScaleAccelRange(0);
  mpu.setFullScaleGyroRange(0);
  set_offsets();
}
