#include "state_machine.h"
#include <Arduino.h>

#define START_BUTTON_PIN 50
#define STOP_BUTTON_PIN 51
#define SENSOR1 A0
#define SENSOR2 A1
#define C_PUMP_CH 3
#define D_PUMP_CH 4
#define MIXER_CH 1
#define PRESS_CH 2
#define RELAY_PIN 53

StateMachine::StateMachine()
  : currentState(STATE_A), previousState(STATE_COUNT), stateStart(0), started(false), hasRun(false),
    emergencyStopActive(false), c_pump(C_PUMP_CH), d_pump(D_PUMP_CH), motor_mixer(MIXER_CH),
    motor_press(PRESS_CH), sensor1(0), sensor2(0) {
  duration[STATE_A] = 5000;
  duration[STATE_B] = 20000;
  duration[STATE_B_2] = 10000;
  duration[STATE_C] = 10000;
  duration[STATE_D] = 10000;
  duration[STATE_E] = 10000;
  duration[STATE_F] = 14000;
  duration[STATE_G] = 20000;
  duration[STATE_H] = 10000;
}

void StateMachine::begin() {
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  c_pump.kill();
  d_pump.kill();
  motor_mixer.kill();
  motor_press.kill();
}

void StateMachine::update() {
  unsigned long now = millis();
  bool startPressed = (digitalRead(START_BUTTON_PIN) == LOW);
  bool stopPressed = (digitalRead(STOP_BUTTON_PIN) == LOW);

  if (emergencyStopActive) return;

  if (started && stopPressed) {
    onExit(currentState);
    kill();
    started = false;
    emergencyStopActive = true;
    return;
  }

  if (!started && !hasRun && startPressed) {
    started = true;
    hasRun = true;
    currentState = STATE_A;
    previousState = STATE_COUNT;  // “no previous state”
    stateStart = now;
    return;  // let the state-change logic handle onEnter
  }


  if (!started) return;

  if (currentState != previousState) {
    onExit(previousState);
    onEnter(currentState);
    previousState = currentState;
    stateStart = now;
  }

  if (now - stateStart >= duration[currentState]) {
    switch (currentState) {
      case STATE_A: currentState = STATE_B; break;
      case STATE_B: currentState = STATE_B_2; break;
      case STATE_B_2: currentState = STATE_C; break;
      case STATE_C: currentState = STATE_D; break;
      case STATE_D: currentState = STATE_E; break;
      case STATE_E: currentState = STATE_F; break;
      case STATE_F: currentState = STATE_G; break;
      case STATE_G: currentState = STATE_H; break;
      case STATE_H:
        onExit(STATE_H);
        kill();
        started = false;
        return;
      default: break;
    }
  }


  runState(currentState);
}

float StateMachine::read_sensor_1() {
  sensor1 = analogRead(SENSOR1);
  return sensor1 * (5.0 / 1024.0);
}

float StateMachine::read_sensor_2() {
  sensor2 = analogRead(SENSOR2);
  return sensor2 * (5.0 / 1024.0);
}

void StateMachine::kill() {
  c_pump.kill();
  d_pump.kill();
  motor_mixer.kill();
  motor_press.kill();
  digitalWrite(RELAY_PIN, LOW);
}

void StateMachine::onEnter(State s) {
  Serial.println("Entering new state");
  switch (s) {
    case STATE_A: break;
    case STATE_B: c_pump.forward(); break;
    case STATE_B_2: digitalWrite(RELAY_PIN, HIGH); break;
    case STATE_C: motor_mixer.forward(); break;
    case STATE_D: break;
    case STATE_E: break;
    case STATE_F: motor_press.backward(); break;
    case STATE_G: d_pump.forward(); break;
    case STATE_H: break;
    default: break;
  }
}

void StateMachine::runState(State s) {
  switch (s) {
    case STATE_A: Serial.println(read_sensor_1()); break;
    case STATE_B: c_pump.setSpeed(255); break;
    case STATE_B_2: break;
    case STATE_C: motor_mixer.setSpeed(255); break;
    case STATE_D: motor_mixer.setSpeed(150); break;
    case STATE_E: break;
    case STATE_F: motor_press.setSpeed(255); break;
    case STATE_G: d_pump.setSpeed(255); break;
    case STATE_H: Serial.println(read_sensor_2()); break;
    default: break;
  }
}

void StateMachine::onExit(State s) {
  Serial.println("Exiting state");
  switch (s) {
    case STATE_B: c_pump.kill(); break;
    case STATE_B_2: digitalWrite(RELAY_PIN, LOW); break;
    case STATE_D: motor_mixer.kill(); break;
    case STATE_E: break;
    case STATE_F: motor_press.kill(); break;
    case STATE_G: d_pump.kill(); break;
    case STATE_H: break;
    default: break;
  }
}
