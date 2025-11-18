#include "motor.h"

Motor::Motor(int channel) : motor(channel) {}

void Motor::forward() {
  motor.run(FORWARD);
}

void Motor::backward() {
  motor.run(BACKWARD);
}

void Motor::setSpeed(int speed) {
  motor.setSpeed(speed);
}

void Motor::stop() {
  motor.setSpeed(0);
}

void Motor::kill() {
  motor.setSpeed(0);
  motor.run(RELEASE);
}
