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

  ITerm1 += Ki*(error_roll);
  
  ITerm1 = constrain(ITerm1, -255, 255);

  
  dInput1 = (roll_angle - lastInput1);  // curr - prev

  output1 = Kp * error_roll + Ki * ITerm1 - Kd * dInput1;

  output1 = constrain(output1, -255, 255);


  lastInput1 = roll_angle;
  
  return output1;
}

double Compute_yaw() {

    double Kp =0 , Ki = 0, Kd = 0;
    double yaw_angle;

    yaw_angle = encoder_count();

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