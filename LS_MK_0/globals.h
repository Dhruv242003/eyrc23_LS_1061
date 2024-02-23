/////////////// Actuate.h   //////////// 
double YAW;
double roll_offset = 0;
double error_roll = 0;
double roll_setpoint = 0;
double error_yaw = 0;
double output_roll = 0, output_yaw = 0;

double U;
double vel;

#define CASCADED1 1   // HEETHESH
#define CASCADED2 2   // CASCADED EYANTRA
#define PARALLEL 3

/////////////// angle.h   //////////// 
double ROLL = 0;

/////////////// Compute_PID.h  ////////////

double outputR, outputY;
float max_angle_enc = 2;
bool STOP_FLAG = true;
double I_roll, previous_roll, D_roll;
double I_yaw, previous_yaw, D_yaw;

double Kp_r = 0;
double Ki_r = 0;
double Kd_r = 0; 
double Kp_y = 0;
double Ki_y = 0;
double Kd_y = 0; 

//////////// Indicators.h  ////////////

#define LED_GREEN A0
#define LED_RED A1
#define BUZZER 7
#define HALL_RIGHT A6
#define HALL_LEFT A7

double base_left = 0;
double base_right = 0;
bool base_done = false;

double hall_left = 0;
double hall_right = 0;
double hall_sensi = 20;
bool magnet_detected = false;

bool flag = false;
unsigned long startTime = 0;
unsigned long ledStartTime = 0;  
unsigned long buzzerStartTime = 0;

bool magnetDetected = false;
unsigned long magnetDetectionStartTime = 0;
unsigned long magnetDetectionDuration = 1000;  

float pi = 3.14159265359;

//////////// Motors.h  ////////////
// Geared Motor
#define enA 6
#define in1 A2
#define in2 A3

// BO Motor
#define enB 5
#define in3 9
#define in4 4
#define FORWARD	1
#define BACKWARD 2
#define STOP 3

int STEER_ANGLE = 0;
#define BO_MIN_PWM 100
#define DC_MIN_PWM 55


////////////  nrf.h  ////////////
// Static state gains
double STATIC_KP_ROLL = 0;
double STATIC_KI_ROLL = 0;
double STATIC_KD_ROLL = 0;
double STATIC_KP_YAW = 0;
double STATIC_KI_YAW = 0;
double STATIC_KD_YAW = 0;

// Traversing Gains

double TRAVERSE_KP_ROLL = 0;
double TRAVERSE_KI_ROLL = 0;
double TRAVERSE_KD_ROLL = 0;
double TRAVERSE_KP_YAW = 0;
double TRAVERSE_KI_YAW = 0;
double TRAVERSE_KD_YAW = 0;

// JoyStick variables
int joyX = 0;
int joyY = 0;

// isTraversing ?
bool isTraversing = true;

// Run Indicators
int RUN_END = 0;
int RUN_START = 0;

void nrf_setup();
void get_NRF_Gains();

struct DataPacket
{
    float array[6];
    bool static_tuning;
    int x;
    int y;
    int b1;
    int b2;
} receivedValues;


///// read_angle.h ////

// int16_t ax, ay, az, gx, gy, gz;
// float a[3] = {0, 0, 0}, g[3] = {0, 0, 0};
// const float pi = 3.14159265359, comp_alpha = 0.02, dT = 0.003;
// double ROLL = 0;

/////timers.h  ///

