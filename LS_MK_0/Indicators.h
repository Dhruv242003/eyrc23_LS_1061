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

void Hall_init();
void led_init();
void buzzer_init();
void buzzerOn();
void buzzerOff();
bool northDetected();
bool southDetected();
void roll_yaw_indicator();
void detect_magnet();
void runIndicator();


void Hall_init() {
  pinMode(HALL_RIGHT, INPUT);
  pinMode(HALL_LEFT, INPUT);
}


void led_init() {
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
}


void buzzer_init() {
  pinMode(BUZZER, OUTPUT);
}


void buzzerOn() {
  digitalWrite(BUZZER, HIGH);
}


void buzzerOff() {
  digitalWrite(BUZZER, LOW);
}


bool northDetected() {
  if (hall_right > base_right + hall_sensi || hall_left < base_left - hall_sensi) return true;
  else return false;
}


bool southDetected() {
  if (hall_left > base_left + hall_sensi || hall_right < base_right - hall_sensi) return true;
  else return false;
}


void roll_yaw_indicator() {
  if (abs(error_roll) < 0.5) {
    digitalWrite(LED_RED, HIGH);
  } else {
    digitalWrite(LED_RED, LOW);
  }
  if (error_yaw > -3 * (pi / 180) && error_yaw < 3 * (pi / 180)) {
    digitalWrite(LED_GREEN, HIGH);
  } else {
    digitalWrite(LED_GREEN, LOW);
  }
}



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
      magnet_detected = true;
      ledStartTime = millis();     // Start LED timer
      buzzerStartTime = millis();  // Start buzzer timer
    } else if (southDetected()) {
      // Negative Magnet
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED, HIGH);
      magnet_detected = true;
      ledStartTime = millis();     // Start LED timer
      buzzerStartTime = millis();  // Start buzzer timer
    } else {
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED, LOW);
      magnet_detected = false;
    }
  }

  // Turn off LEDs after 3 seconds
  if (millis() - ledStartTime >= 3000) {
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, LOW);
    magnet_detected = false;
  }

  // Beep the buzzer twice with a one-second stop between beeps for 3 seconds
  if (magnet_detected && millis() - buzzerStartTime < 3000) {
    unsigned long beepDuration = (millis() - buzzerStartTime) % 2000;
    if (beepDuration < 1000) {
      buzzerOn();
    } else {
      buzzerOff();
    }
  } else {
    buzzerOff();
  }
}

void runIndicator(){
  if (RUN_START == 1 || RUN_END == 1) {
    buzzerOn();
  } else if(RUN_START == 0 || RUN_END == 0) {
    buzzerOff();
  }
}

