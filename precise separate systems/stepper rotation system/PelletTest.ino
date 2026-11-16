// Standalone Uno + EasyDriver. STEP=D2; DIR=D3; common GND.
// Serial p = rotate once. No movement command on startup.
// EasyDriver can energize coils whenever motor power is on, including at rest.
// Power off before connecting/disconnecting motor wires.
const uint8_t STEP_PIN = 2, DIR_PIN = 3;

// ===== EDIT THESE SETTINGS =====
const float DISPENSE_DEGREES = 0.0f; // TODO: e.g. 30.0. Zero disables motion.
const uint16_t FULL_STEPS_PER_REV = 200; // 1.8-degree motor: verify label
const uint8_t MICROSTEPS = 8; // MUST match actual MS1/MS2 hardware setting
const float MOTOR_REVS_PER_DISPENSER_REV = 1.0f; // direct drive = 1
const bool DISPENSE_DIRECTION = HIGH; // LOW reverses
const unsigned int STEP_INTERVAL_US = 2000; // higher = slower
const unsigned int STEP_HIGH_US = 5;
const unsigned long AFTER_DISPENSE_DELAY_MS = 1000;

void dispenseOnce() {
  if (DISPENSE_DEGREES <= 0 || STEP_INTERVAL_US <= STEP_HIGH_US ||
      MOTOR_REVS_PER_DISPENSER_REV <= 0) {
    Serial.println(F("Set positive DISPENSE_DEGREES and valid timing/gearing first."));
    return;
  }
  long pulses = (long)(DISPENSE_DEGREES / 360.0f * FULL_STEPS_PER_REV *
                       MICROSTEPS * MOTOR_REVS_PER_DISPENSER_REV + 0.5f);
  if (pulses <= 0) { Serial.println(F("Angle smaller than one pulse.")); return; }
  digitalWrite(DIR_PIN, DISPENSE_DIRECTION);
  delayMicroseconds(10);
  for (long i = 0; i < pulses; ++i) {
    digitalWrite(STEP_PIN, HIGH); delayMicroseconds(STEP_HIGH_US);
    digitalWrite(STEP_PIN, LOW); delayMicroseconds(STEP_INTERVAL_US - STEP_HIGH_US);
  }
  Serial.print(F("Commanded pulses: ")); Serial.println(pulses);
  delay(AFTER_DISPENSE_DELAY_MS);
  // Discard queued commands so holding p does not cause back-to-back deliveries.
  while (Serial.available()) Serial.read();
  Serial.println(F("Ready: p = one more rotation."));
}

void setup() {
  digitalWrite(STEP_PIN, LOW); digitalWrite(DIR_PIN, LOW);
  pinMode(STEP_PIN, OUTPUT); pinMode(DIR_PIN, OUTPUT);
  Serial.begin(115200);
  Serial.println(F("Set angle/current limit; p = one rotation. No pellet sensing yet."));
}

void loop() {
  if (Serial.available() && Serial.read() == 'p') dispenseOnce();
}
