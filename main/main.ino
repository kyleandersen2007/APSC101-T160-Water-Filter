#include "state_machine.h"

StateMachine machine;

void setup() {
  Serial.begin(9600);
  machine.begin();
}

void loop() {
  machine.update();
}
