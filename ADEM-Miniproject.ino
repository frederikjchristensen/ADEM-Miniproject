#include "library.h"

unsigned char ID = 253; // ID can't be 253, so this way we know something went wrong in setup if this is still 253.
unsigned char temp = 0;
bool shutoff = false;
int frequency = 10;
Dynamixel_p2 MOTOR(13); // Constructs a instance of the class.
unsigned long samplingT = micros();


void setup() {
  Serial.begin(57600); // Begin Serial port comms.
  MOTOR.begin(57600); // Constructs a class of Dynamixel servo called "MOTOR"
  MOTOR.Reboot(0xFE);
  delay(2000);
  MOTOR.setTorqueEnable(0xFE, false); // Disable torque so that EEPROM where ID is stored can be accessed.

}
void loop() {

  while (!shutoff) {
    if (ID == 253) {
      for (int i = 0; i < 253; i++) {
        if (MOTOR.getID(i) != 0xFF) {
          ID = MOTOR.getID(i);
          Serial.print("found "); Serial.println(ID);
          MOTOR.setPositionGainP(ID, 100);
          MOTOR.setTorqueEnable(ID, true);
        }
        else {
          //Serial.print("no motor on "); Serial.println(i);
        }
      }
    }

    if (micros() - samplingT > 1000000 / frequency) {

      if (ID != 253) {
        MOTOR.setGoalPosition(ID, 2048);
        Serial.print("ID: ");
        Serial.print(ID);
        Serial.print(" Temp: ");
        temp = MOTOR.getTemperature(ID);
        Serial.println(temp);
        MOTOR.setGoalPosition(ID, 0);
      }

      if (temp > 40) {
        MOTOR.Reboot(0xFE);
        shutoff = true;
      }


      samplingT = micros();
    }
  }
}
