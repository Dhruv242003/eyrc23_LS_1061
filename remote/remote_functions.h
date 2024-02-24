#include <SPI.h>
#include <RF24.h>

const int VRx = A1;
const int VRy = A0;
int x_value, y_value;
int sw_state = 0;

unsigned long time = 0;

RF24 radio(7, 8);

bool flag = false;


#define joyXOffset 8
#define joyYOffset 1

int joyX = 0;
int joyY = 0;

#define BUTTON1 A3
#define BUTTON2 A4
#define SW A2
const byte address[6] = "00001";

void printTransmitted();
void runIndicators();
void serialInput();
void handleButtons();
void joySW();
void handleJoyStick();
void remoteScheduler();
void remote_init();


struct DataPacket {
  float array[6] = { 100, 0, 80, 0, 0, 0 };  // Use float instead of double
  bool isTraversing;
  int x;          // Use int8_t instead of int
  int y;          // Use int8_t instead of int
  int8_t sw = 0;  // Use int8_t instead of int
  int8_t b1 = 0;  // Use int8_t instead of int
  int8_t b2 = 0;  // Use int8_t instead of int
} data;

void remote_init() {
  pinMode(SW, INPUT_PULLUP);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);

  radio.begin();
  radio.openWritingPipe(address);
  radio.stopListening();
}

void printTransmitted() {
  Serial.print(data.array[0], 4);
  Serial.print(" ");
  Serial.print(data.array[1], 4);
  Serial.print(" ");
  Serial.print(data.array[2], 4);
  Serial.print(" ");
  Serial.print(data.array[3], 4);
  Serial.print(" ");
  Serial.print(data.array[4], 4);
  Serial.print(" ");
  Serial.print(data.array[5], 4);
  Serial.println();
}

void runIndicators() {
  data.b1 = !digitalRead(BUTTON1);
  data.b2 = !digitalRead(BUTTON2);
  if (data.b1 == 1) {
    time = millis();
    while (millis() - time <= 1000) {
      radio.write(&data, sizeof(data));
    }
  } else if (data.b2 == 1) {
    time = millis();
    while (millis() - time <= 5000) {
      radio.write(&data, sizeof(data));
    }
  } else {
    data.b1 = 0;
    data.b2 = 0;
    radio.write(&data, sizeof(data));
  }
}

void serialInput() {
  if (Serial.available() > 0) {
    double tempArr[6] = { 0 };
    String inputString = Serial.readStringUntil('\n');
    int index = 0;
    char *token = strtok(const_cast<char *>(inputString.c_str()), " \t");
    while (token != NULL && index < 6) {
      tempArr[index++] = atof(token);
      token = strtok(NULL, " \t");
    }
    for (int i = 0; i < 6; i++) {
      data.array[i] = tempArr[i];
    }
  }
}


void handleButtons() {
  joySW();
  runIndicators();
}

void joySW() {
  if (!digitalRead(SW)) {
    sw_state = !sw_state;
    data.sw = sw_state;
  }
}

void handleJoyStick() {
  joyX = analogRead(VRx);
  joyY = analogRead(VRy);

  joyX += joyXOffset;
  joyY += joyYOffset;

  data.x = map(joyX, 0, 889, -100, 100);
  data.y = map(joyY, 0, 889, -100, 100);

  // data.x += joyXOffset;
  // data.y += joyYOffset;
}

void remoteScheduler() {
  serialInput();
  handleJoyStick();
  handleButtons();
  // printTransmitted();
}