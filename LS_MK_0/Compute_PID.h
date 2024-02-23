#include <Arduino.h>

double roll_offfset = 0, error_roll = 0, roll_setpoint =0;
double error_yaw = 0;
double ITerm1, lastInput1, dInput1, ITerm2, lastInput2, dInput2;
double outputR, outputY;
float max_angle_enc=2;
bool STOP_FLAG = true;

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
    Kp = roll_con_KP;
    Ki = roll_con_KI;
    Kd = roll_con_KD;
  }

  // Aggressive PID gains for |errors| >= 3 degress
  else
  {
    Kp = roll_agr_KP;
    Ki = roll_agr_KI;
    Kd = roll_agr_KD;
  }

  dInput1 = (roll_angle - lastInput1); // curr - prev

  ITerm1 += Ki * (error_roll);
  ITerm1 = constrain(ITerm1, -255, 255);

  outputR = Kp * error_roll + Ki * ITerm1 - Kd * dInput1;

  outputR = constrain(outputR, -255, 255);

  lastInput1 = roll_angle;

  // return outputR;
}

double Compute_yaw()
{

  double Kp = 0, Ki = 0, Kd = 0;
  double yaw_angle;

  yaw_angle = encoder_count();

  error_yaw = 0 - yaw_angle;

  if (!STOP_FLAG)
  {
    Kp = yaw_con_KP;
    Ki = yaw_con_KI;
    Kd = yaw_con_KD;
    max_angle_enc = 2;
  }
  // Static balance
  else
  {
    Kp = yaw_agr_KP;
    Ki = yaw_agr_KI;
    Kd = yaw_agr_KD;
    max_angle_enc = 2;
  }

  ITerm2 += Ki*(error_yaw);
  ITerm2 = constrain(ITerm2, -255, 255);

  dInput2 = (yaw_angle - lastInput2); // Curr - prev.

  outputY = Kp * error_yaw + Ki * ITerm2 - Kd * dInput2; // same signs of kp and kd term.

  outputY = constrain(outputY, -max_angle_enc, max_angle_enc);

  lastInput2 = yaw_angle;
 
  // return outputY;
}