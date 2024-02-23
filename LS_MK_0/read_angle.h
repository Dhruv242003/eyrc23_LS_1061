
int16_t ax, ay, az, gx, gy, gz, gnx, gny, gnz;
float a[3] = {0, 0, 0}, g[3] = {0, 0, 0};

const float pi = 3.14159265359, f_cut = 5, dT = 0.003, comp_alpha = 0.02;
float roll = 0;

const float accel_sf = 16384, gyro_sf = 131;
/******************/
MPU6050 mpu;

void readSensor();
void test();
void set_offsets();
void read_accel();
void read_gyro();
void complimentary_filter_roll();
void mpu_init();

void readSensor()
{
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
  mpu.setXAccelOffset(-1173);
  mpu.setYAccelOffset(-1221);
  mpu.setZAccelOffset(895);

  mpu.setXGyroOffset(110);
  mpu.setYGyroOffset(103);
  mpu.setZGyroOffset(17);
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
  roll = (1 - comp_alpha) * (roll + g[1] * dT) + (comp_alpha) * (atan(a[0] / abs(a[2]))) * (180 / 3.14);
}

void mpu_init()
{
  mpu.initialize();
  test();
  mpu.setFullScaleAccelRange(0);
  mpu.setFullScaleGyroRange(0);
  set_offsets();
}
