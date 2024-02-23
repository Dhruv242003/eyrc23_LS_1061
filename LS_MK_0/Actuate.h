void actuate()
{
    int type = CASCADED2;

    //////////      CASCADED  HEETHESH   ////////
    if(type == CASCADED1){
        double YAW_Copy = YAW;
        error_yaw = STEER_ANGLE - YAW_Copy;
        output_yaw = Compute_yaw(error_yaw);

        double roll_angle = ROLL;
        roll_setpoint = roll_offset + output_yaw;
        error_roll = roll_setpoint - roll_angle;
        output_roll = Compute_roll(error_roll);

        U = output_roll;
    }
    else if(type == CASCADED2){
        ///////// CASCADED EYANTRA ///////////
        double YAW_Copy = YAW;
        error_yaw = STEER_ANGLE - YAW_Copy;
        output_yaw = Compute_yaw(error_yaw);

        double roll_angle = ROLL;
        error_roll =  (roll_offset - roll_angle) - output_yaw;
        output_roll = Compute_roll(error_roll);

        U = output_roll;
    }
    else if(type == PARALLEL){
        ///////////  PARALLEL  ///////////////
        double YAW_Copy = YAW;
        error_yaw = STEER_ANGLE - YAW_Copy;
        output_yaw = Compute_yaw(error_yaw);

        double roll_angle = ROLL;
        error_roll =  (roll_offset - roll_angle);
        output_roll = Compute_roll(error_roll);

        U = output_roll + output_yaw;
    }
    vel = constrain(U, -255, 255);

    actuate_DC(vel);
}
