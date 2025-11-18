#include <AFMotor.h>

#define START_BUTTON_PIN 50
#define STOP_BUTTON_PIN 51

#define SENSOR1 A0
#define SENSOR2 A1

#define C_PUMP_CH 3
#define D_PUMP_CH 4

#define MIXER_CH 1
#define PRESS_CH 2

#define RELAY_PIN 53

enum State {
  STATE_A,
  STATE_B,
  STATE_B_2,
  STATE_C,
  STATE_D,
  STATE_E,
  STATE_F,
  STATE_G,
  STATE_COUNT
};

State currentState = STATE_A;
State previousState = STATE_COUNT;

unsigned long stateStart = 0;

// Durations for each state
unsigned long duration[STATE_COUNT] = {
  10000,  // STATE_A: read value
  10000,  // STATE_B: pump clean / coagulant
  10000,  // STATE_B_2: relay / alum etc.
  10000,  // STATE_C: mix part 1
  10000,  // STATE_D: mix part 2
  10000,  // STATE_E: waiting phase
  12000,  // STATE_F: press plate
  30000   // STATE_G: pump out clean water
};

const float CLEAN_THRESHOLD = 1.0f;  // currently unused, kept for future logic

bool started = false;              // sequence currently running
bool hasRun = false;               // has the sequence ever been started?
bool emergencyStopActive = false;  // true after emergency stop

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

void stopAllOutputs() {
  // Generic "everything off" safety function
  c_pump.setSpeed(0);
  c_pump.run(RELEASE);

  d_pump.setSpeed(0);
  d_pump.run(RELEASE);

  motor_mixer.setSpeed(0);
  motor_mixer.run(RELEASE);

  motor_press.setSpeed(0);
  motor_press.run(RELEASE);

  digitalWrite(RELAY_PIN, LOW);
}

void onEnter(State s) {
  switch (s) {
    case STATE_A:
      Serial.println("ENTER A: Reading sensor...");
      break;

    case STATE_B:
      Serial.println("ENTER B: Coagulant / clean pump...");
      c_pump.run(FORWARD);
      break;

    case STATE_B_2:
      Serial.println("ENTER B_2: Relay ON...");
      digitalWrite(RELAY_PIN, HIGH);
      break;

    case STATE_C:
      Serial.println("ENTER C: Mixing Part 1...");
      motor_mixer.run(FORWARD);
      break;

    case STATE_D:
      Serial.println("ENTER D: Mixing Part 2...");
      // mixer continues from STATE_C
      break;

    case STATE_E:
      Serial.println("ENTER E: Waiting Phase...");
      break;

    case STATE_F:
      Serial.println("ENTER F: Pressing Phase...");
      motor_press.run(BACKWARD);
      break;

    case STATE_G:
      Serial.println("ENTER G: Pumping out clean water...");
      d_pump.run(FORWARD);
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
      c_pump.setSpeed(255);
      break;

    case STATE_B_2:
      // relay already HIGH in onEnter
      break;

    case STATE_C:
      motor_mixer.setSpeed(255);
      break;

    case STATE_D:
      motor_mixer.setSpeed(150);
      break;

    case STATE_E:
      // Waiting phase
      break;

    case STATE_F:
      motor_press.setSpeed(255);
      break;

    case STATE_G:
      d_pump.setSpeed(255);
      break;

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
      Serial.println("EXIT B: Stopping coagulant pump");
      c_pump.setSpeed(0);
      c_pump.run(RELEASE);
      break;

    case STATE_B_2:
      Serial.println("EXIT B_2: Relay OFF");
      digitalWrite(RELAY_PIN, LOW);
      break;

    case STATE_C:
      break;

    case STATE_D:
      Serial.println("EXIT D: Stopping mixer");
      motor_mixer.setSpeed(0);
      motor_mixer.run(RELEASE);
      break;

    case STATE_E:
      Serial.println("EXIT E");
      break;

    case STATE_F:
      Serial.println("EXIT F: Releasing press");
      motor_press.setSpeed(0);
      motor_press.run(RELEASE);
      break;

    case STATE_G:
      Serial.println("EXIT G: Stopping clean water pump");
      d_pump.setSpeed(0);
      d_pump.run(RELEASE);
      break;

    default:
      break;
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  c_pump.run(RELEASE);
  d_pump.run(RELEASE);
  motor_mixer.run(RELEASE);
  motor_press.run(RELEASE);
}

void loop() {
  unsigned long now = millis();

  bool startPressed = (digitalRead(START_BUTTON_PIN) == LOW);
  bool stopPressed = (digitalRead(STOP_BUTTON_PIN) == LOW);

  if (emergencyStopActive) {
    return;
  }

  if (started && stopPressed) {
    Serial.println("EMERGENCY STOP (STOP BUTTON) TRIGGERED!");
    onExit(currentState);
    stopAllOutputs();
    started = false;
    emergencyStopActive = true;
    return;
  }

  if (!started && !hasRun && startPressed) {
    Serial.println("START BUTTON PRESSED: Beginning sequence");
    started = true;
    hasRun = true;
    currentState = STATE_A;
    previousState = STATE_COUNT;
    stateStart = now;
    onEnter(currentState);
    return;
  }

  if (!started) {
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
        currentState = STATE_B_2;
        break;

      case STATE_B_2:
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

      case STATE_F:
        currentState = STATE_G;
        break;

      case STATE_G:

        onExit(STATE_G);
        stopAllOutputs();

        started = false;
        return;

      default:
        break;
    }
  }
  runState(currentState);
}
