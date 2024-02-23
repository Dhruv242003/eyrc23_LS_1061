#include <Arduino.h>

double Compute_roll() {

  double Kp =0 , Ki = 0, Kd = 0;
  double roll_angle;


    // Obtain the current tilt angle and make a copy to avoid
	// abrupt changes during current PID computation loop
  roll_angle = roll;

  roll_setpoint = roll_offfset + yaw_output;

  error_roll = roll_setpoint - roll_angle;

  if (abs(error_roll) < 3.0)
	{
		Kp = con_KP;
		Ki = con_KI;
		Kd = con_KD;
	}
	
	// Aggressive PID gains for |errors| >= 3 degress
	else
	{
		Kp = agr_KP;
		Ki = agr_KI;
		Kd = agr_KD;
	}

  ITerm1 += (error_roll);
  if (ITerm1 > outMax) ITerm1 = 255;
  else if (ITerm1 < outMin) ITerm1 = -255;

  if (abs(error_roll) < 0.05) ITerm1 = 0;
  dInput1 = (error_roll - lastInput1);  // curr - prev

  output1 = Kp * error_roll + Ki * ITerm1 + Kd * dInput1;

  if (output1 > outMax) output1 = outMax;
  if (output1 < outMin) output1 = outMin;
  //      if(output1 < 0) output1 = abs(output1);

  lastInput1 = error_roll;
  
  return output1;
  // }
}

/***********
   Function Name: Compute_yaw
   Input: error_yaw - double value representing yaw error
          Kp - double value for proportional gain
          Ki - double value for integral gain
          Kd - double value for derivative gain
   Output: double - computed output for yaw control
   Logic: Computes PID output for yaw control
   Example Call: Compute_yaw(error_yaw, Kp, Ki, Kd);
 ***********/

double Compute_yaw(double Kp, double Ki, double Kd) {


    ITerm2 += (error_yaw);
    if (ITerm2 > outMax) ITerm2 = 255;
    else if (ITerm2 < outMin) ITerm2 = -255;

    if (abs(error_yaw) < 0.05) ITerm2 = 0;
    dInput2 = (error_yaw - lastInput2) / timeChange * 100;  //Curr - prev.

    output2 = Kp * error_yaw + Ki * ITerm2 + Kd * dInput2;  // same signs of kp and kd term.

    if (output2 > outMax) output2 = outMax;
    if (output2 < outMin) output2 = outMin;
    //      if(output2 < 0) output2 = abs(output2);

    lastInput2 = error_yaw;
    lastTime2 = now;
    return output2;
}