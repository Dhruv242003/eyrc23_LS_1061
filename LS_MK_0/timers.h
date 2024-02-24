void setupTimer1();
void setupTimer2();
void timerSetup();

ISR(TIMER2_COMPA_vect) {
  // 10 ms timer
  OCR2A += 156;
  ROLL = get_angle();
}
ISR(TIMER1_COMPA_vect) {
  // 20 ms timer
  OCR1A += 34000;  // Advance The COMPA Register
  YAW = -getEncoderCount();
}

void setupTimer2() {
  TCCR2A = 0;           // Init Timer2A
  TCCR2B = 0;           // Init Timer2B
  TCCR2B |= B00000111;  // Prescaler = 1024
  OCR2A = 156;          // Timer Compare2A Register
  TIMSK2 |= B00000010;  // Enable Timer COMPA Interrupt
}

void setupTimer1() {
  TCCR1A = 0;           // Init Timer1A
  TCCR1B = 0;           // Init Timer1B
  TCCR1B |= B00000010;  // Prescaler = 8
  OCR1A = 34000;        // Timer Compare1A Register
  TIMSK1 |= B00000010;  // Enable Timer COMPA Interrupt
}

void timerSetup() {
  setupTimer1();
  setupTimer2();
}
