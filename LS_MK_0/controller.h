void actuate();
void traverse(bool isTraversing);
void steer();
void scheduler();

void scheduler(){
    actuate();
    traverse(isTraversing);
    steer();
}

void actuate()
{   
    if (SW == 1){
        STEER_ANGLE = 0;
        myEnc.write(0);
    }
    //////////      CASCADED1    ////////
    if (type == CASCADED1)
    {
        double alpha_1 = 0.1; 
        double YAW_Copy = YAW;
        static double smoothed_output_yaw = 0;
        
        error_yaw = STEER_ANGLE - YAW_Copy;
        output_yaw = Compute_yaw(error_yaw);
        
        smoothed_output_yaw = alpha_1 * smoothed_output_yaw + (1 - alpha_1) * output_yaw;

        double roll_angle = ROLL;
        static double smoothed_output_roll = 0;
        
        roll_setpoint = roll_offset + smoothed_output_yaw;
        error_roll = roll_setpoint - roll_angle;
        output_roll = Compute_roll(error_roll);
        
        smoothed_output_roll = alpha_1 * smoothed_output_roll + (1 - alpha_1) * output_roll;

        U = smoothed_output_roll;
    }
    else if (type == CASCADED2)
    {
        ///////// CASCADED2 ///////////
        double YAW_Copy = YAW;
        error_yaw = STEER_ANGLE - YAW_Copy;
        output_yaw = Compute_yaw(error_yaw);
      
        double roll_angle = ROLL;
        error_roll = (roll_offset - roll_angle) - output_yaw;
        output_roll = Compute_roll(error_roll);

        U = output_roll;
    }
    else if (type == PARALLEL)
    {
        ///////////  PARALLEL  ///////////////
        double YAW_Copy = YAW;
        error_yaw = STEER_ANGLE - YAW_Copy;
        output_yaw = Compute_yaw(error_yaw);

        double roll_angle = ROLL;
        error_roll = (roll_offset - roll_angle);
        output_roll = Compute_roll(error_roll);

        U = output_roll + output_yaw;
    }
    else if (type == LQR)
    {
        ///////////  LQR  ///////////////
        double YAW_Copy = YAW;
        double yaw_vel = YAW_VEL;
        error_yaw = STEER_ANGLE - YAW_Copy;
        error_yaw_vel = 0 - yaw_vel;
        // Serial.println(YAW_VEL);

        double roll_angle = ROLL;
        double roll_vel = VEL_ROLL;
        
        error_roll = (roll_offset - roll_angle);
        error_roll_vel = 0 - roll_vel;
        // Serial.println(error_yaw_vel);
        U = ((K1*error_roll) + (K2*error_roll_vel)+ (K3*error_yaw) + (K4*error_yaw_vel) );
    }
    vel = constrain(U, -255, 255);
    // Serial.println(U);
    actuate_DC(vel);
}

void traverse(bool isTraversing)
{
    double bo_speed = 0;
    if (isTraversing)
    {
        if (joyY > 10)
        {
            bo_speed = map(joyY, 10, 100, 0, 255);
            BO_Control(FORWARD, bo_speed);
        }
        else if (joyY < -10)
        {
            bo_speed = map(joyY, -10, -100, 0, 255);
            BO_Control(BACKWARD, bo_speed);
        }
        else
        {
            BO_Control(STOP, 0);
        }
    }
}

void steer()
{
    if (joyX > 10)
    {
      // Serial.println("hello");
      STEER_ANGLE -= STEER_FACTOR;
    //   if(YAW - STEER_ANGLE >= STEER_LIMIT){
    //     STEER_ANGLE = -STEER_LIMIT;
    //   }
    }
    else if (joyX < -10)
    {
      STEER_ANGLE += STEER_FACTOR;
    }
    // if(STEER_ANGLE - YAW >= STEER_LIMIT){
    //     STEER_ANGLE = STEER_LIMIT;
    //   }
}

// Some functions for ERS i.e EMERGENCY RESPONSE SYSTEM

void ERS(){
    if(ROLL > 10 || ROLL < -10){
        actuate_DC(0);
        BO_Control(STOP, 0);
        myEnc.write(0);   // makes the YAW 0  
    }      
}
