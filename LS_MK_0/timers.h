#include "read_angle.h"
#include "Motors.h"

void setupTimer1();
void setupTimer2();
void timerSetup();

ISR(TIMER1_COMPA_vect) {
  // 100 Hz Timer for reading Roll
    readSensor(); // MPU Reading roll
}

ISR(TIMER2_COMPA_vect) {
  // 50 Hz Timer for reading Encoders
  YAW = getEncoderCount();
}

void setupTimer1() {
  TCCR1A = 0; 
  TCCR1B = 0;
  TCNT1 = 0;  // Initialize counter value to 0
  OCR1A = 15999; // Set the value that Timer1 will count up to (100Hz frequency)
  TCCR1B |= (1 << WGM12); // Set WGM12 bit for CTC mode
  TCCR1B |= (1 << CS11) | (1 << CS10); // Set CS10 and CS11 bits for 64 prescaler
  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 compare interrupt
}

void setupTimer2() {
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0; // Initialize counter value to 0
  OCR2A = 79; // Set the value that Timer2 will count up to (50Hz frequency)
  TCCR2A |= (1 << WGM21); // Set WGM21 bit for CTC mode
  TCCR2B |= (1 << CS22); // Set CS22 bit for 64 prescaler
  TIMSK2 |= (1 << OCIE2A); // Enable Timer2 compare interrupt
}

void timerSetup(){
    setupTimer1();
    setupTimer2();
}