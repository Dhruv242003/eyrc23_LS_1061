#include <MPU6050_light.h>

MPU6050 mpu(Wire);
void mpu_init();
double get_angle();
void printMPUStatus();
void printOffSets();

void printMPUStatus(byte status) {
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {
  }
}

void mpu_init() {
  Wire.begin();
  byte status = mpu.begin();
  printMPUStatus(status);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED, HIGH);
  delay(200);
  mpu.calcOffsets();
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED, HIGH);
  delay(200);
  // printOffSets();

  //  mpu.setGyroOffsets(-0.214046 , -1.31, -0.940916);
  //  mpu.setAccOffsets( 0.021778  , -0.02 ,  0.067591);

  // mpu.setGyroOffsets(-0.090294, -1.4525, -1.08892625);
  // mpu.setAccOffsets(0.031191375, -0.0083, 0.0602215);

  // mpu.setGyroOffsets(1.208336, -1.65, -1.592856);
  // mpu.setAccOffsets(0.037885, 0.01, 0.065807);

  // mpu.setGyroOffsets(-3.35, -3.16, -0.54);
  // mpu.setAccOffsets(0.10, 0.00, -0.01);


  // mpu.setGyroOffsets(-3.387235 , -2.86, -0.050748);
  // mpu.setAccOffsets(0.068582, -0.00, -0.002664);
}

double get_angle() {
  return -mpu.getAngleX();
}

void printOffSets() {
  Serial.print("\tGyroXoffset : ");
  Serial.print(mpu.getGyroXoffset(), 6);
  Serial.print(" \tGyroYoffset : ");
  Serial.print(mpu.getGyroYoffset());
  Serial.print("\tGyroZoffset : ");
  Serial.print(mpu.getGyroZoffset(), 6);
  Serial.print("\tAccXoffset : ");
  Serial.print(mpu.getAccXoffset(), 6);
  Serial.print(" \tAccYoffset : ");
  Serial.print(mpu.getAccYoffset());
  Serial.print("\tAccZoffset : ");
  Serial.println(mpu.getAccZoffset(), 6);

  //GyroXoffset : -3.305861 	GyroYoffset : -2.88	GyroZoffset : -0.004214	AccXoffset : 0.089003 	AccYoffset : -0.00	AccZoffset : -0.002088
  //GyroXoffset : -3.268946 	GyroYoffset : -2.87	GyroZoffset : -0.003969	AccXoffset : 0.071667 	AccYoffset : 0.03	AccZoffset : -0.002107
  //	GyroXoffset : -3.282472 	GyroYoffset : -2.88	GyroZoffset : -0.003206	AccXoffset : 0.071447 	AccYoffset : 0.03	AccZoffset : -0.002184
  //	GyroXoffset : -3.284215 	GyroYoffset : -2.87	GyroZoffset : -0.010443	AccXoffset : 0.071687 	AccYoffset : 0.03	AccZoffset : -0.002006
}
