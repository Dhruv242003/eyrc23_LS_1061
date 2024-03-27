#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(10, 8);  // CE, CSN
const byte address[6] = "00001";

void nrf_setup() {
  radio.begin();
  radio.openReadingPipe(1, address);
  radio.startListening();
}

void get_NRF_Gains() {
  if (radio.available()) {

    radio.read(&receivedValues, sizeof(receivedValues));
    isTraversing = receivedValues.isTraversing;
    SW = receivedValues.sw;
    if (type == CASCADED1 || type == CASCADED2 || type == PARALLEL) {
      if (!isTraversing) {
        STATIC_KP_ROLL = receivedValues.array[0];
        STATIC_KI_ROLL = receivedValues.array[1];
        STATIC_KD_ROLL = receivedValues.array[2];
        STATIC_KP_YAW = receivedValues.array[3];
        STATIC_KI_YAW = receivedValues.array[4];
        STATIC_KD_YAW = receivedValues.array[5];
      } else {
        TRAVERSE_KP_ROLL = receivedValues.array[0];
        TRAVERSE_KI_ROLL = receivedValues.array[1];
        TRAVERSE_KD_ROLL = receivedValues.array[2];
        TRAVERSE_KP_YAW = receivedValues.array[3];
        TRAVERSE_KI_YAW = receivedValues.array[4];
        TRAVERSE_KD_YAW = receivedValues.array[5];
      }
    }

    else if (type == LQR) {
      K1 = receivedValues.array[0];
      K2 = receivedValues.array[1];
      K3 = receivedValues.array[2];
      K4 = receivedValues.array[3];
    }
    

    // Serial.println(TRAVERSE_KP_ROLL);

    joyX = receivedValues.x;
    joyY = receivedValues.y;

    RUN_START = receivedValues.b1;
    RUN_END = receivedValues.b2;

    // Serial.println("Received Values: ");
    // Serial.print("Roll_P : ");
    // Serial.print(TRAVERSE_KP_ROLL, 4);
    // Serial.println(" ");
    // Serial.print("Roll_I : ");
    // Serial.print(TRAVERSE_KI_ROLL, 4);
    // Serial.println(" ");
    // Serial.print("Roll_D : ");
    // Serial.print(TRAVERSE_KD_ROLL, 4);
    // Serial.println(" ");
    // Serial.print("Yaw_P : ");
    // Serial.print(TRAVERSE_KP_YAW, 4);
    // Serial.println(" ");
    // Serial.print("Yaw_I : ");
    // Serial.print(TRAVERSE_KI_YAW, 4);
    // Serial.println(" ");
    // Serial.print("Yaw_D : ");
    // Serial.print(TRAVERSE_KD_YAW, 4);
    // Serial.println(" ");
    // Serial.print("JoyX : ");
    // Serial.print(joyX);
    // Serial.println(" ");
    // Serial.print("JoyY : ");
    // Serial.print(joyY);
    // Serial.println(" ");
    // delay(200);

    
  }
}