#include "remote_functions.h"

void setup()
{
  Serial.begin(9600);
  remote_init();
}
void loop()
{
  data.isTraversing = true;
  remoteScheduler();
  delay(200); // Introduce a small delay after write
}
