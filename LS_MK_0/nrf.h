#include <nRF24L01.h>
#include <RF24.h>

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
bool isTraversing = false;

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

RF24 radio(10, 8); // CE, CSN
const byte address[6] = "00001";

void nrf_setup()
{
    radio.begin();
    radio.openReadingPipe(1, address);
    radio.startListening();
}

void get_NRF_Gains()
{
    if (radio.available())
    {

        radio.read(&receivedValues, sizeof(receivedValues));
        isTraversing = receivedValues.static_tuning;
        if (isTraversing)
        {
            STATIC_KP_ROLL = receivedValues.array[0];
            STATIC_KI_ROLL = receivedValues.array[1];
            STATIC_KD_ROLL = receivedValues.array[2];
            STATIC_KP_YAW = receivedValues.array[3];
            STATIC_KI_YAW = receivedValues.array[4];
            STATIC_KD_YAW = receivedValues.array[5];
        }
        else
        {
            TRAVERSE_KP_ROLL = receivedValues.array[0];
            TRAVERSE_KI_ROLL = receivedValues.array[1];
            TRAVERSE_KD_ROLL = receivedValues.array[2];
            TRAVERSE_KP_YAW = receivedValues.array[3];
            TRAVERSE_KI_YAW = receivedValues.array[4];
            TRAVERSE_KD_YAW = receivedValues.array[5];
        }

        joyX = receivedValues.x;
        joyY = receivedValues.y;

        RUN_START = receivedValues.b1;
        RUN_END = receivedValues.b2;
    }
}
