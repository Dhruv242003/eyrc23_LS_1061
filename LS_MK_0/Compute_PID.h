#include <Arduino.h>

double Compute_roll();
double Compute_yaw();

double Compute_roll(double error_roll) {
  // Turn motors off if robot falls beyond recoverable angle and await human rescue
  if (abs(error_roll) >= 75) {
    outputR = 0;
    return;
  }

  if (!isTraversing) {

    Kp_r = STATIC_KP_ROLL;
    Ki_r = STATIC_KI_ROLL;
    Kd_r = STATIC_KD_ROLL;
  }
  // Aggressive PID gains for |errors| >= 3 degress
  else {
    Kp_r = TRAVERSE_KP_ROLL;
    Ki_r = TRAVERSE_KI_ROLL;
    Kd_r = TRAVERSE_KD_ROLL;
  }
  // Serial.println(Kp_r);

  D_roll = (ROLL - previous_roll);  // curr - prev

  I_roll += Ki_r * (error_roll);
  I_roll = constrain(I_roll, -255, 255);

  outputR = Kp_r * error_roll + I_roll - Kd_r * D_roll;

  outputR = constrain(outputR, -255, 255);

  previous_roll = ROLL;

  return outputR;
}

double Compute_yaw(double error_yaw) {
  if (!isTraversing) {

    Kp_y = STATIC_KP_YAW;
    Ki_y = STATIC_KI_YAW;
    Kd_y = STATIC_KD_YAW;
  }
  // Static balance
  else {
    Kp_y = TRAVERSE_KP_YAW;
    Ki_y = TRAVERSE_KI_YAW;
    Kd_y = TRAVERSE_KD_YAW;
  }

  I_yaw += Ki_y * (error_yaw);
  I_yaw = constrain(I_yaw, -255, 255);

  D_yaw = (YAW - previous_yaw);  // Curr - prev.

  outputY = Kp_y * error_yaw + Ki_y * I_yaw - Kd_y * D_yaw;  // same signs of kp and kd term.
  outputY *= 0.1;
  outputY = constrain(outputY, -255, 255);

  previous_yaw = YAW;

  return outputY;
}