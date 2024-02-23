double YAW;
double roll_offset = 0;
double error_roll = 0;
double roll_setpoint = 0;
double error_yaw = 0;
double output_roll = 0, output_yaw = 0;

double U;
double vel;

void actuate()
{
    //////////      CASCADED  HEETHESH   ////////
    double YAW_Copy = YAW;
    error_yaw = yaw_joy - YAW_Copy;
    output_yaw = Compute_yaw(error_yaw);

    double roll_angle = ROLL;
    roll_setpoint = roll_offset + output_yaw;
    error_roll = roll_setpoint - roll_angle;
    output_roll = Compute_roll(error_roll);

    U = output_roll;

    ///////// CASCADED EYANTRA ///////////
    double YAW_Copy = YAW;
    error_yaw = yaw_joy - YAW_Copy;
    output_yaw = Compute_yaw(error_yaw);

    double roll_angle = ROLL;
    error_roll =  (roll_offset - roll_angle) - output_yaw;
    output_roll = Compute_roll(error_roll);

    U = output_roll;

    ///////////  PARALLEL  ///////////////
    double YAW_Copy = YAW;
    error_yaw = yaw_joy - YAW_Copy;
    output_yaw = Compute_yaw(error_yaw);

    double roll_angle = ROLL;
    error_roll =  (roll_offset - roll_angle);
    output_roll = Compute_roll(error_roll);

    U = output_roll + output_yaw;

    if (U < 0)
    {
        vel = constrain(U, -255, -55);
    }
    else if (U > 0)
    {
        vel = constrain(U, 55, 255);
    }
    else
    {
        vel = 0;
    }
    motor_control(vel);
}
