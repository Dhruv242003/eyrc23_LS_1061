void actuate();
void traverse(bool isTraversing);
void steer();
void scheduler();


void scheduler(){

    detect_magnet();

    if((millis() - lastTime_PID) >= 12){
        lastTime_PID = millis();
        actuate();
        traverse(isTraversing);
        steer();
    }

}

void actuate()
{
    int type = CASCADED1;

    //////////      CASCADED1    ////////
    if (type == CASCADED1)
    {
        double YAW_Copy = YAW;
        error_yaw = STEER_ANGLE - YAW_Copy;
        output_yaw = Compute_yaw(error_yaw);

        double roll_angle = ROLL;
        roll_setpoint = roll_offset + output_yaw;
        error_roll = roll_setpoint - roll_angle;
        output_roll = Compute_roll(error_roll);

        U = output_roll;
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
    vel = constrain(U, -255, 255);

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
      STEER_ANGLE -= 0.35;
    }
    else if (joyX < -10)
    {
      STEER_ANGLE += 0.35;
    }
}