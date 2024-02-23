#include <SPI.h>
#include <RF24.h>

const int VRx = A1; // analog pin for x-axis
const int VRy = A0; // analog pin for y-axis
const int SW = 7;   // Digital pin for the button
int x_value, y_value, button_state;

unsigned long time = 0;
int joyX = 0;
int joyY = 0;
RF24 radio(7, 8);

bool flag = false;

#define joyYOffset 2
#define joyXOffset 4

// #define BUTTON1 3
// #define BUTTON2 4

const byte address[6] = "00001";

struct DataPacket
{
  double array[6] = {100, 0, 80, 0, 0, 0};
  bool static_tuning;
  int x;
  int y;
  int b1 = 0;
  int b2 = 0;
} data;

void setup()
{
  Serial.begin(9600);
  pinMode(SW, INPUT_PULLUP);
  // pinMode(BUTTON1, INPUT);
  // pinMode(BUTTON2, INPUT);

  radio.begin();
  radio.openWritingPipe(address);
  radio.stopListening();
}

void loop()
{
  data.static_tuning = true;
  if (Serial.available() > 0)
  {
    double tempArr[6] = {0};
    String inputString = Serial.readStringUntil('\n');

    int index = 0;
    char *token = strtok(const_cast<char *>(inputString.c_str()), " \t");
    while (token != NULL && index < 8)
    {
      if (index == 6)
      {
        data.b1 = atof(token);
        index++;
      }
      else if (index == 7)
      {
        data.b2 = atof(token);
        index++;
      }
      else
      {
        tempArr[index++] = atof(token);
      }

      token = strtok(NULL, " \t");
    }
    for (int i = 0; i < 6; i++)
    {
      data.array[i] = tempArr[i];
    }
  }

  joyX = analogRead(VRx);
  joyY = analogRead(VRy);

  data.x = map(joyX, 0, 889, -100, 100);
  data.y = map(joyY, 0, 889, -100, 100);

  data.x -=joyXOffset;
  data.y -=joyYOffset;

  // Serial.println(data.x);
  // data.b1 = digitalRead(BUTTON1);
  // data.b2 = digitalRead(BUTTON2);
  if (data.b1 == 1)
  {
    if (!flag)
    {
      time = millis();
      flag = true;
    }
    data.b1 = 1;
    radio.write(&data, sizeof(data));
    if (millis() - time >= 1000)
    {
      data.b1 = 0;
      flag = false;
    }
  }
  else if (data.b2 == 1)
  {
    if (!flag)
    {
      time = millis();
      flag = true;
    }

    radio.write(&data, sizeof(data));
    if (millis() - time >= 5000)
    {
      data.b2 = 0;
      flag = false;
    }
  }
  else
  {
    data.b1 = 0;
    data.b2 = 0;
    radio.write(&data, sizeof(data));
  }

  // data.b1 = 0;
  // data.b2 =0;

 
  printTransmitted();
  delay(200); // Introduce a small delay after write
}

void printTransmitted(){
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
