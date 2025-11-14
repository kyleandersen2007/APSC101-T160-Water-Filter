#include <AFMotor.h>

#define BUTTON_PIN 2
#define SENSOR1 A0
#define SENSOR2 A1

#define P_PUMP_CH 4
#define C_PUMP_CH 3
#define D_PUMP_CH 2

#define MIXER_CH 1
#define PRESS_CH 2

enum State {
  STATE_A,
  STATE_B,
  STATE_C,
  STATE_D,
  STATE_E,
  STATE_F,
  STATE_COUNT
};

State currentState = STATE_A;
State previousState = STATE_COUNT;

unsigned long stateStart = 0;

unsigned long duration[STATE_COUNT] = {
  5000,
  2000,
  30000,
  5000,
  5000,
  5000
};

const float CLEAN_THRESHOLD = 1.0;

bool started = false;

AF_DCMotor p_pump(P_PUMP_CH);
AF_DCMotor c_pump(C_PUMP_CH);
AF_DCMotor d_pump(D_PUMP_CH);

AF_DCMotor motor_mixer(MIXER_CH);
AF_DCMotor motor_press(PRESS_CH);

int sensor1;
int sensor2;

float read_sensor_1() {
  sensor1 = analogRead(SENSOR1);
  return sensor1 * (5.0 / 1024.0);
}

float read_sensor_2() {
  sensor2 = analogRead(SENSOR2);
  return sensor2 * (5.0 / 1024.0);
}

void onEnter(State s) {
  switch (s) {
    case STATE_A:
      Serial.println("ENTER A: Reading Sensor 1 (dirty water level)...");
      break;

    case STATE_B:
      Serial.println("ENTER B: Pumping in dirty water + Alum...");
      c_pump.setSpeed(255);
      p_pump.setSpeed(255);
      break;

    case STATE_C:
      Serial.println("ENTER C: Mixing...");
      motor_mixer.setSpeed(255);
      break;

    case STATE_D:
      Serial.println("ENTER D: Pressing...");
      motor_press.setSpeed(255);
      break;

    case STATE_E:
      Serial.println("ENTER E: Pumping out clean water...");
      d_pump.setSpeed(255);
      break;

    case STATE_F:
      Serial.println("ENTER F: Reading Sensor 2 (cleanliness)...");
      break;

    default:
      break;
  }
}

void runState(State s) {
  switch (s) {
    case STATE_A:
      Serial.print("Sensor 1 (V): ");
      Serial.println(read_sensor_1());
      break;

    case STATE_B:
      c_pump.run(FORWARD);
      p_pump.run(FORWARD);
      break;

    case STATE_C:
      motor_mixer.run(FORWARD);
      break;

    case STATE_D:
      motor_press.run(FORWARD);
      break;

    case STATE_E:
      d_pump.run(FORWARD);
      break;

    case STATE_F: {
      float v = read_sensor_2();
      Serial.print("Sensor 2 (V): ");
      Serial.println(v);
      break;
    }

    default:
      break;
  }
}

void onExit(State s) {
  if (s == STATE_COUNT) return;

  switch (s) {
    case STATE_A:
      Serial.println("EXIT A");
      break;

    case STATE_B:
      Serial.println("EXIT B: Stopping dirty/coagulant pumps");
      c_pump.setSpeed(0);
      p_pump.setSpeed(0);
      c_pump.run(RELEASE);
      p_pump.run(RELEASE);
      break;

    case STATE_C:
      Serial.println("EXIT C: Stopping mixer");
      motor_mixer.setSpeed(0);
      motor_mixer.run(RELEASE);
      break;

    case STATE_D:
      Serial.println("EXIT D: Stopping press");
      motor_press.setSpeed(0);
      motor_press.run(RELEASE);
      break;

    case STATE_E:
      Serial.println("EXIT E: Stopping clean pump");
      d_pump.setSpeed(0);
      d_pump.run(RELEASE);
      break;

    case STATE_F:
      Serial.println("EXIT F");
      break;

    default:
      break;
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  p_pump.run(RELEASE);
  c_pump.run(RELEASE);
  d_pump.run(RELEASE);
  motor_mixer.run(RELEASE);
  motor_press.run(RELEASE);
}

void loop() {
  unsigned long now = millis();

  if (!started) {
    if (digitalRead(BUTTON_PIN) == LOW) {
      started = true;
      currentState = STATE_A;
      previousState = STATE_COUNT;
      stateStart = now;
      onEnter(currentState);
    }
    return;
  }

  if (currentState != previousState) {
    onExit(previousState);
    onEnter(currentState);
    previousState = currentState;
    stateStart = now;
  }

  if (now - stateStart >= duration[currentState]) {
    switch (currentState) {
      case STATE_A:
        currentState = STATE_B;
        break;

      case STATE_B:
        currentState = STATE_C;
        break;

      case STATE_C:
        currentState = STATE_D;
        break;

      case STATE_D:
        currentState = STATE_E;
        break;

      case STATE_E:
        currentState = STATE_F;
        break;

      case STATE_F: {
        break;
      }

      default:
        break;
    }
  }

  runState(currentState);
}
