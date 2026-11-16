// Standalone BENCH test: 28BYJ-48 + ULN2003 board, IN1..IN4 = D8..D11.
// No position switches or obstruction detection. Test without an animal.
// r = confirm manually positioned OPEN, c = one cycle, x = stop/release.
const uint8_t COIL_PINS[4] = {8, 9, 10, 11};

// ===== EDIT THESE SETTINGS =====
const long DOOR_TRAVEL_HALF_STEPS = 0; // TODO: calibrate. 0 disables motion.
// Roughly 4096 half-steps/output revolution; actual gearbox ratio varies.
const int CLOSE_DIRECTION = 1; // change to -1 if reversed
const unsigned long CLOSE_STEP_INTERVAL_US = 5000; // larger = slower
const unsigned long OPEN_FAST_INTERVAL_US = 1500;  // tune under actual load
const unsigned long OPEN_START_INTERVAL_US = 5000; // gentle acceleration
const long OPEN_RAMP_STEPS = 100;
const unsigned long CLOSED_PAUSE_MS = 2000; // TODO: time with slit closed
const unsigned long AFTER_OPEN_PAUSE_MS = 1000;
// These intervals are between half-steps, not separate full-motion delays.
// 1500 us is a starting target, NOT a guaranteed maximum motor speed.

const uint8_t PHASES[8][4] = {
  {1,0,0,0},{1,1,0,0},{0,1,0,0},{0,1,1,0},
  {0,0,1,0},{0,0,1,1},{0,0,0,1},{1,0,0,1}
};
enum DoorState { UNREFERENCED, READY, CLOSING, CLOSED_WAIT, OPENING, OPEN_WAIT };
DoorState state = UNREFERENCED;
int phaseIndex = 0;
long stepsDone = 0;
unsigned long lastStepUs = 0, stateSinceMs = 0;

void releaseCoils() {
  for (uint8_t i = 0; i < 4; ++i) digitalWrite(COIL_PINS[i], LOW);
}

void halfStep(int direction) {
  phaseIndex = (phaseIndex + direction + 8) % 8;
  for (uint8_t i = 0; i < 4; ++i) digitalWrite(COIL_PINS[i], PHASES[phaseIndex][i]);
}

unsigned long openingInterval() {
  long ramp = OPEN_RAMP_STEPS;
  if (ramp > DOOR_TRAVEL_HALF_STEPS / 2) ramp = DOOR_TRAVEL_HALF_STEPS / 2;
  if (ramp <= 0 || OPEN_START_INTERVAL_US <= OPEN_FAST_INTERVAL_US) return OPEN_FAST_INTERVAL_US;
  long edge = stepsDone;
  long remaining = DOOR_TRAVEL_HALF_STEPS - 1 - stepsDone;
  if (remaining < edge) edge = remaining;
  if (edge > ramp) edge = ramp;
  return OPEN_START_INTERVAL_US -
    (OPEN_START_INTERVAL_US - OPEN_FAST_INTERVAL_US) * (unsigned long)edge / ramp;
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < 4; ++i) pinMode(COIL_PINS[i], OUTPUT);
  releaseCoils();
  Serial.println(F("Position OPEN manually, then r. c=cycle; x=stop/release."));
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'x') {
      releaseCoils(); state = UNREFERENCED;
      Serial.println(F("Stopped. Position unknown: manually open and send r."));
    } else if (c == 'r' && (state == UNREFERENCED || state == READY)) {
      state = READY; Serial.println(F("Manual open reference accepted."));
    } else if (c == 'c' && state == READY) {
      if (DOOR_TRAVEL_HALF_STEPS <= 0) Serial.println(F("Set DOOR_TRAVEL_HALF_STEPS first."));
      else {
        state = CLOSING; stepsDone = 0; lastStepUs = micros();
        Serial.println(F("Closing."));
      }
    }
  }
  if (state == CLOSING || state == OPENING) {
    unsigned long interval = state == CLOSING ? CLOSE_STEP_INTERVAL_US : openingInterval();
    unsigned long now = micros();
    if (now - lastStepUs >= interval) {
      lastStepUs = now;
      halfStep(state == CLOSING ? CLOSE_DIRECTION : -CLOSE_DIRECTION);
      if (++stepsDone >= DOOR_TRAVEL_HALF_STEPS) {
        state = state == CLOSING ? CLOSED_WAIT : OPEN_WAIT;
        stateSinceMs = millis();
        Serial.println(state == CLOSED_WAIT ? F("Closed pause.") : F("Open pause."));
      }
    }
  } else if (state == CLOSED_WAIT && millis() - stateSinceMs >= CLOSED_PAUSE_MS) {
    state = OPENING; stepsDone = 0; lastStepUs = micros();
    Serial.println(F("Opening."));
  } else if (state == OPEN_WAIT && millis() - stateSinceMs >= AFTER_OPEN_PAUSE_MS) {
    releaseCoils(); state = UNREFERENCED;
    Serial.println(F("Done; coils released. Check OPEN position, then r for next test."));
  }
}
