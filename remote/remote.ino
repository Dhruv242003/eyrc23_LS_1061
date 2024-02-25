#include "remote_functions.h"

void setup()
{
  Serial.begin(9600);
  remote_init();
}
void loop()
{
  //80.0000 8.0000 90.0000 0.5000 0.0000 12.0000

  data.isTraversing = true;
  remoteScheduler();
  delay(200); // Introduce a small delay after write
  // Serial.print(data.sw);
  // Serial.print(" ");
  // Serial.print(data.b1);
  // Serial.print(" ");
  // Serial.println(data.x);
}
