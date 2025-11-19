#pragma once
#include "motor.h"

class StateMachine {
public:
  enum State {
    STATE_A,
    STATE_B,
    STATE_B_2,
    STATE_C,
    STATE_D,
    STATE_E,
    STATE_F,
    STATE_G,
    STATE_H,
    STATE_COUNT
  };

  StateMachine();
  void begin();
  void update();

private:
  State currentState, previousState;
  unsigned long stateStart;
  unsigned long duration[STATE_COUNT];

  bool started;
  bool hasRun;
  bool paused;
  bool lastStopPressed;

  unsigned long pauseStart;

  Motor c_pump;
  Motor d_pump;
  Motor motor_mixer;
  Motor motor_press;

  int sensor1;
  int sensor2;

  float read_sensor_1();
  float read_sensor_2();
  void kill();
  void onEnter(State s);
  void runState(State s);
  void onExit(State s);
};
