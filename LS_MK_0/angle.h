#include <MPU6050_light.h>

MPU6050 mpu(Wire);
double ROLL = 0;

void mpu_init();
double get_angle();
void printMPUStatus();

void printMPUStatus(byte status) {
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {
  }
}

void mpu_init()
{
  Wire.begin();
  byte status = mpu.begin();
  printMPUStatus(status);

  // mpu.calcOffsets();

  //  mpu.setGyroOffsets(-0.214046 , -1.31, -0.940916);
  //  mpu.setAccOffsets( 0.021778  , -0.02 ,  0.067591);

  // mpu.setGyroOffsets(-0.090294, -1.4525, -1.08892625);
  // mpu.setAccOffsets(0.031191375, -0.0083, 0.0602215);

  mpu.setGyroOffsets(1.208336, -1.65, -1.592856);
  mpu.setAccOffsets(0.037885, 0.01, 0.065807);

}

double get_angle()
{
  ROLL = -mpu.getAngleX();
  return ROLL;
}
