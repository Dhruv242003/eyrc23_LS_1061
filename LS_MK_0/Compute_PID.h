#include <Arduino.h>
#include <read_angle.h>

double roll_offfset = 0;
double error_roll = 0;
double roll_setpoint = 0;
double error_yaw = 0;
double outputR, outputY;
float max_angle_enc = 2;
bool STOP_FLAG = true;
double I_roll, previous_roll, D_roll;
double I_yaw, previous_yaw, D_yaw;

void Compute_roll();

void Compute_yaw();

void Compute_roll()
{

  double Kp = 0, Ki = 0, Kd = 0;
  double roll_angle;

  // Obtain the current tilt angle and make a copy to avoid
  // abrupt changes during current PID computation loop
  roll_angle = roll;

  roll_setpoint = roll_offfset + outputY;

  error_roll = roll_setpoint - roll_angle;

  // Turn motors off if robot falls beyond recoverable angle and await human rescue
  if (abs(error_roll) >= 75)
  {
    outputR = 0;
    return;
  }

  if (abs(error_roll) < 3.0)
  {
    Kp = STATIC_KP_ROLL;
    Ki = STATIC_KI_ROLL;
    Kd = STATIC_KD_ROLL;
  }

  // Aggressive PID gains for |errors| >= 3 degress
  else
  {
    Kp = TRAVERSE_KP_ROLL;
    Ki = TRAVERSE_KI_ROLL;
    Kd = TRAVERSE_KD_ROLL;
  }

  D_roll = (roll_angle - previous_roll); // curr - prev

  I_roll += Ki * (error_roll);
  I_roll = constrain(I_roll, -255, 255);

  outputR = Kp * error_roll + Ki * I_roll - Kd * D_roll;

  outputR = constrain(outputR, -255, 255);

  previous_roll = roll_angle;

  // return outputR;
}

double Compute_yaw()
{

  double Kp = 0, Ki = 0, Kd = 0;
  double yaw_angle;

  yaw_angle = getEncoderCount();

  error_yaw = 0 - yaw_angle;

  if (!STOP_FLAG)
  {
    Kp = STATIC_KP_YAW;
    Ki = STATIC_KI_YAW;
    Kd = STATIC_KD_YAW;
  }
  // Static balance
  else
  {
    Kp = TRAVERSE_KP_YAW;
    Ki = TRAVERSE_KI_YAW;
    Kd = TRAVERSE_KD_YAW;
  }

  I_yaw += Ki * (error_yaw);
  I_yaw = constrain(I_yaw, -255, 255);

  D_yaw = (yaw_angle - previous_yaw); // Curr - prev.

  outputY = Kp * error_yaw + Ki * I_yaw - Kd * D_yaw; // same signs of kp and kd term.

  outputY = constrain(outputY, -max_angle_enc, max_angle_enc);

  previous_yaw = yaw_angle;

  // return outputY;
}