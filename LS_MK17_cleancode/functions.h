/////// PID ///////
unsigned long SampleTime = 12;
unsigned long lastTime1;
unsigned long lastTime2;

double outMin = -255.0, outMax = 255.0;
double output1;  //Temp Var for debugging
double output2;
double kpR, kiR, kdR;
double kpY, kiY, kdY;
int joyX = 400;
int joyY = 400;
double outputR = 0;
double outputY = 0;
double ITerm1, lastInput1, dInput1;
double ITerm2, lastInput2, dInput2;


double yawsetpoint = 0;
double rollSetpoint = 0;

bool isTraversing = true;

double yaw_joy = 0;

// LED PINS
#define LED_GREEN A0
#define LED_RED A1

// BUZZER
#define BUZZER 7
unsigned long buzzerStartTime = 0;
unsigned long startTime = 0;     // Variable to store the start time of the buzzer
unsigned long ledStartTime = 0;  // Variable to store the start time of the LED


bool magnetDetected = false;
unsigned long magnetDetectionStartTime = 0;
unsigned long magnetDetectionDuration = 1000;  // 1 second for example
// bool buzzerOn = false;

// HALL EFFECT
#define HALL_RIGHT A7
#define HALL_LEFT A6
double hall_right = 0;
double hall_left = 0;
bool magnet_detected = false;
double hall_sensi = 20;
double base_left = 0;
double base_right = 0;
bool base_done = false;


// Geared Motor
#define enA 6
#define in1 A2
#define in2 A3

// BO Motor
#define enB 5
#define in3 9
#define in4 4
float pi = 3.14159265359;
float vel;
float U;
int bo_speed;

float s1, s2, accum1, accum2;

// IMU
MPU6050 mpu(Wire);
unsigned long timer = 0;

Encoder myEnc(2, 3);

double yaw = 0;
double oldYaw = 0;
unsigned long previousMillis = 0;
const unsigned long interval = 100;
double angularVelocity = 0;
long oldPosition = -999;

// Control Variables
float x, y, z;
float k[6] = { 0, 0, 0, 0, 0, 0 };  // {+,-,-,-}//float k[4] = {0, 0 , 0, 0 };
float receivedValues[6];


struct DataPacket {
  float array[6];
  int x;
  int y;
};

// uncomment if you want to send values from joystick.
//float receivedValues[2];


RF24 radio(10, 8);  // CE, CSN
const byte address[6] = "00001";

unsigned long timerInterval = 12;  // Default timer interval in milliseconds




/*************************************************************************************
   Function Name: roll_yaw_indicator
   Input: None
   Output: None
   Logic: Controls LEDs based on roll and yaw angles
   Example Call: roll_yaw_indicator();
 *************************************************************************************/


void roll_yaw_indicator() {
  if (abs(x) < 1) {
    digitalWrite(LED_RED, HIGH);
  } else {
    digitalWrite(LED_RED, LOW);
  }
  if (yaw > -3 && yaw < 3) {
    digitalWrite(LED_GREEN, HIGH);
  } else {
    digitalWrite(LED_GREEN, LOW);
  }
}

/*************************************************************************************
   Function Name: southDetected
   Input: None
   Output: bool - true if south magnet detected, false otherwise
   Logic: Checks if a south magnet is detected by Hall effect sensors
   Example Call: southDetected();
 *************************************************************************************/

bool southDetected() {
  if (hall_left > base_left + hall_sensi || hall_right < base_right - hall_sensi) return true;
  else return false;
}

/*************************************************************************************
   Function Name: northDetected
   Input: None
   Output: bool - true if north magnet detected, false otherwise
   Logic: Checks if a north magnet is detected by Hall effect sensors
   Example Call: northDetected();
 *************************************************************************************/

bool northDetected() {
  if (hall_right > base_right + hall_sensi || hall_left < base_left - hall_sensi) return true;
  else return false;
}

/*************************************************************************************
   Function Name: buzzerOn
   Input: None
   Output: None
   Logic: Turns on the buzzer
   Example Call: buzzerOn();
 *************************************************************************************/

void buzzerOn() {
  digitalWrite(BUZZER, HIGH);
}

/*************************************************************************************
   Function Name: buzzerOff
   Input: None
   Output: None
   Logic: Turns off the buzzer
   Example Call: buzzerOff();
 *************************************************************************************/

void buzzerOff() {
  digitalWrite(BUZZER, LOW);
}


/*************************************************************************************
   Function Name: detect_magnet
   Input: None
   Output: None
   Logic: Detects magnet presence and controls LEDs accordingly
   Example Call: detect_magnet();
 *************************************************************************************/

void detect_magnet() {

  if (!base_done) {
    base_right = analogRead(HALL_RIGHT);
    base_left = analogRead(HALL_LEFT);
    base_done = true;
  }
  if (!magnet_detected) {
    hall_right = analogRead(HALL_RIGHT);
    hall_left = analogRead(HALL_LEFT);


    if (northDetected()) {
      // Positive Magnet
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_RED, LOW);
      buzzerOn();
      ledStartTime = millis();  // Start LED timer
      magnet_detected = true;

    } else if (southDetected()) {
      // Negative Magnet
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED, HIGH);
      buzzerOn();
      ledStartTime = millis();  // Start LED timer
      magnet_detected = true;
    } else {
      digitalWrite(LED_GREEN, LOW);
      magnet_detected = false;
      buzzerOff();
      digitalWrite(LED_RED, LOW);
    }
  }

  // Turn off LEDs after 3 seconds
  if (millis() - ledStartTime >= 3000) {
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, LOW);
    buzzerOff();
    magnet_detected = false;
  }
}

/*************************************************************************************
   Function Name: dc_motor_init
   Input: None
   Output: None
   Logic: Initializes DC motor pins and configurations
   Example Call: dc_motor_init();
 *************************************************************************************/

// Init Functions
void dc_motor_init() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}


/*************************************************************************************
   Function Name: bo_motor_init
   Input: None
   Output: None
   Logic: Initializes BO motor pins and configurations
   Example Call: bo_motor_init();
 *************************************************************************************/

void bo_motor_init() {
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

/*************************************************************************************
   Function Name: Hall_init
   Input: None
   Output: None
   Logic: Initializes Hall effect sensor pins
   Example Call: Hall_init();
 *************************************************************************************/

void Hall_init() {
  pinMode(HALL_RIGHT, INPUT);
  pinMode(HALL_LEFT, INPUT);
}

/*************************************************************************************
   Function Name: led_init
   Input: None
   Output: None
   Logic: Initializes LED pins
   Example Call: led_init();
 *************************************************************************************/

void led_init() {
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
}

/*************************************************************************************
   Function Name: buzzer_init
   Input: None
   Output: None
   Logic: Initializes buzzer pin
   Example Call: buzzer_init();
 *************************************************************************************/

void buzzer_init() {
  pinMode(BUZZER, OUTPUT);
}

/*************************************************************************************
   Function Name: motor_control
   Input: pwm - int value for PWM control
   Output: None
   Logic: Controls DC motor direction and speed based on PWM value
   Example Call: motor_control(pwm);
 *************************************************************************************/
// Motor Control Functions
void motor_control(int pwm) {
  if (pwm < 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    pwm = -pwm;
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  analogWrite(enA, pwm);
}

/*************************************************************************************
   Function Name: forwardBO
   Input: speed - int value for motor speed
   Output: None
   Logic: Drives BO motor forward at specified speed
   Example Call: forwardBO(speed);
 *************************************************************************************/

// BO Motor Control
void forwardBO(int speed) {
  analogWrite(enB, speed);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

/*************************************************************************************
   Function Name: backwardBO
   Input: speed - int value for motor speed
   Output: None
   Logic: Drives BO motor backward at specified speed
   Example Call: backwardBO(speed);
 *************************************************************************************/

void backwardBO(int speed) {
  analogWrite(enB, speed);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

/*************************************************************************************
   Function Name: printMPUStatus
   Input: status - byte value representing MPU status
   Output: None
   Logic: Prints MPU6050 status information
   Example Call: printMPUStatus(status);
 *************************************************************************************/

// Print MPU Status
void printMPUStatus(byte status) {
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {
  }
}

/*************************************************************************************
   Function Name: printMPUData
   Input: None
   Output: None
   Logic: Prints MPU6050 sensor data
   Example Call: printMPUData();
 *************************************************************************************/

void printMPUData() {
  if ((millis() - timer) > 10) {
    Serial.print("Roll : ");
    Serial.print(x, 6);
    Serial.print("\tYaw : ");
    Serial.println(yaw);

    timer = millis();
  }
}

/*************************************************************************************
   Function Name: sensing
   Input: None
   Output: None
   Logic: Reads sensor data from MPU6050 and updates global variables
   Example Call: sensing();
 *************************************************************************************/

void sensing() {

  //  x = (mpu.getAngleX()) * (pi / 180); // Roll
  x = -mpu.getAngleX();  // Roll

  /// --------ENCODER YAW -----------
  long newPosition = myEnc.read();
  if (newPosition != oldPosition && newPosition % 2 == 0) {
    oldPosition = newPosition;
    newPosition = newPosition / 2;
    yaw = (newPosition) / 5.824;
  }
}

/*************************************************************************************
   Function Name: actuate
   Input: pwm - int value for PWM control
   Output: None
   Logic: Actuates motor based on PWM value
   Example Call: actuate(pwm);
 *************************************************************************************/


void actuate(int pwm) {
  motor_control(-pwm);
}

/*************************************************************************************
   Function Name: setupTimer1
   Input: timerInterval - unsigned long interval for timer
   Output: None
   Logic: Initializes Timer1 for executing control loop at regular intervals
   Example Call: setupTimer1(timerInterval);
 *************************************************************************************/

void setupTimer1(unsigned long timerInterval) {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // Calculate the value for OCR1A based on the desired interval
  OCR1A = (timerInterval * 1000) / 4;  // Convert milliseconds to microseconds and divide by 4µs per tick

  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11) | (1 << CS10);

  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}


/*************************************************************************************
   Function Name: Compute_roll
   Input: error_roll - double value representing roll error
          Kp - double value for proportional gain
          Ki - double value for integral gain
          Kd - double value for derivative gain
   Output: double - computed output for roll control
   Logic: Computes PID output for roll control
   Example Call: Compute_roll(error_roll, Kp, Ki, Kd);
 *************************************************************************************/

double Compute_roll(double error_roll, double Kp, double Ki, double Kd) {

  unsigned long now = millis();
  unsigned long timeChange = (now - lastTime1);

  if (timeChange >= SampleTime) {

    ITerm1 += (error_roll * timeChange / 10);
    if (ITerm1 > outMax) ITerm1 = 255;
    else if (ITerm1 < outMin) ITerm1 = -255;

    if (abs(error_roll) < 0.05) ITerm1 = 0;
    dInput1 = (error_roll - lastInput1) / timeChange * 100;  // curr - prev

    output1 = Kp * error_roll + Ki * ITerm1 + Kd * dInput1;

    if (output1 > outMax) output1 = outMax;
    if (output1 < outMin) output1 = outMin;
    //      if(output1 < 0) output1 = abs(output1);

    lastInput1 = error_roll;
    lastTime1 = now;
    return output1;
  }
}

/*************************************************************************************
   Function Name: Compute_yaw
   Input: error_yaw - double value representing yaw error
          Kp - double value for proportional gain
          Ki - double value for integral gain
          Kd - double value for derivative gain
   Output: double - computed output for yaw control
   Logic: Computes PID output for yaw control
   Example Call: Compute_yaw(error_yaw, Kp, Ki, Kd);
 *************************************************************************************/

double Compute_yaw(double error_yaw, double Kp, double Ki, double Kd) {

  unsigned long now = millis();
  unsigned long timeChange = (now - lastTime2);

  if (timeChange >= SampleTime) {

    ITerm2 += (error_yaw * timeChange / 10);
    if (ITerm2 > outMax) ITerm2 = 255;
    else if (ITerm2 < outMin) ITerm2 = -255;

    if (abs(error_yaw) < 0.05) ITerm2 = 0;
    dInput2 = (error_yaw - lastInput2) / timeChange * 100; //Curr - prev.

    output2 = Kp * error_yaw + Ki * ITerm2 + Kd * dInput2;  // same signs of kp and kd term.

    if (output2 > outMax) output2 = outMax;
    if (output2 < outMin) output2 = outMin;
    //      if(output2 < 0) output2 = abs(output2);

    lastInput2 = error_yaw;
    lastTime2 = now;
    return output2;
  }
}

void packet_debug()
{
  Serial.println("Received Values: ");
  Serial.print("Roll_P : ");
  Serial.print(kpR, 4);
  Serial.println(" ");
  Serial.print("Roll_I : ");
  Serial.print(kiR, 4);
  Serial.println(" ");
  Serial.print("Roll_D : ");
  Serial.print(kdR, 4);
  Serial.println(" ");
  Serial.print("Yaw_P : ");
  Serial.print(kpY, 4);
  Serial.println(" ");
  Serial.print("Yaw_I : ");
  Serial.print(kiY, 4);
  Serial.println(" ");
  Serial.print("Yaw_D : ");
  Serial.print(kdY, 4);
  Serial.println(" ");
  Serial.print("JoyX : "); Serial.print(joyX); Serial.println(" ");
  Serial.print("JoyY : "); Serial.print(joyY); Serial.println(" ");
  delay(200);
}