#pragma once
#include <AFMotor.h>

class Motor {
public:
  Motor(int channel);
  void forward();
  void backward();
  void setSpeed(int speed);
  void stop();
  void kill();

private:
  AF_DCMotor motor;
};
