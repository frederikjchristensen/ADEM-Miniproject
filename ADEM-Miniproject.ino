#include "library.h"

Dynamixel_p2 MOTOR(13); // Constructs a instance of the class.

void setup() {
  Serial.begin(57600); // Begin Serial port comms.
  MOTOR.begin(57600); // Constructs a class of Dynamixel servo called "MOTOR"

  delay(1000);
  Serial.print(MOTOR.PingServo(0xFE));
}

void loop() {
  
}
