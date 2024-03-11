#include <Arduino.h>
#include <Encoder.h>

Encoder myEnc(2, 3);

void motors_init();
void dc_motor_init();
void bo_motor_init();
void actuate_DC(int pwm);
void set_dc_pwm(int pwm);
void BO_Control(int mode, int speed);
int getEncoderCount();

void motors_init()
{
    dc_motor_init();
    bo_motor_init();
}

// Init Functions
void dc_motor_init()
{
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
}

void bo_motor_init()
{
    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}

void actuate_DC(int pwm)
{
    if (pwm > 0)
    {
        pwm = map(pwm, 0, 255, DC_MIN_PWM, DC_MAX_PWM);
        set_dc_pwm(pwm);
    }
    else if (pwm < 0)
    {
        pwm = map(pwm, 0, -255, DC_MIN_PWM, DC_MAX_PWM);
        set_dc_pwm(-pwm);
    }
    else
    {
        pwm = 0;
        set_dc_pwm(pwm);
    }
}

void set_dc_pwm(int pwm)
{
    if (pwm > 0)
    {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        analogWrite(enA, pwm);
    }
    else if (pwm < 0)
    {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        analogWrite(enA, abs(pwm));
    }
    else
    {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        analogWrite(enA, 0);
    }
}

void BO_Control(int mode, int speed)
{
    speed = map(speed, 0, 255, 0, BO_MAX_PWM);
    if (mode == FORWARD)
    {
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(enB, speed);
    }
    else if (mode == BACKWARD)
    {
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(enB, speed);
    }
    else if (mode == STOP)
    {
        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);
        analogWrite(enB, 0);
    }
}
int getEncoderCount()
{
    return myEnc.read() / 10;
}

double getEncoderVel(double yaw_angle)
{
    newPosition = yaw_angle;
    if (newPosition != oldPosition && newPosition % 2 == 0)
    {
        oldPosition = newPosition;
        newPosition = newPosition / 2;
        cur_yaw = (newPosition) / 5.824;
        angularVelocity = (cur_yaw - oldYaw) / (10);
        oldYaw = cur_yaw;
    }
    return angularVelocity;
}
