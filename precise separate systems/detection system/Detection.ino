// Standalone Arduino Uno test. Install "VL53L1X" by Pololu.
// ToF: SDA=A4, SCL=A5. TSL1401: SI=D4, CLK=D5, AO=A0.
// Only reports events; it does NOT operate either motor.
#include <Wire.h>
#include <VL53L1X.h>

const uint8_t SI_PIN = 4, CLK_PIN = 5, AO_PIN = A0;

// ===== EDIT THESE SETTINGS =====
const uint16_t BACK_MIN_MM = 0; // TODO: nearest distance in rear trigger zone
const uint16_t BACK_MAX_MM = 0; // TODO: farthest distance; 0 disables triggers
const unsigned long PRESENT_DWELL_MS = 500;
const unsigned long ABSENT_DWELL_MS = 500; // must leave before another event
const bool REQUIRE_CAMERA_CHANGE = false; // start with ToF only
const uint16_t PIXEL_CHANGE_THRESHOLD = 80; // ADC counts, 0..1023; calibrate
const uint8_t MIN_CHANGED_PIXELS = 12;     // calibrate
const uint8_t ROI_FIRST = 0, ROI_LAST = 127; // inclusive pixel window
const unsigned int CAMERA_EXTRA_EXPOSURE_MS = 5;
// Exposure also includes most of the preceding ~15 ms scan, NOT just this delay.
// Keep total integration below 100 ms. Lower illumination if pixels saturate.

VL53L1X tof;
uint16_t pixels[128], background[128];
bool haveBackground = false, armed = true, timingPresent = false, timingAbsent = false;
unsigned long presentSince = 0, absentSince = 0;

void scanLine() {
  digitalWrite(CLK_PIN, LOW);
  digitalWrite(SI_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(CLK_PIN, HIGH); // rising edge 1: pixel 0
  delayMicroseconds(2);
  digitalWrite(SI_PIN, LOW);
  for (uint8_t i = 0; i < 128; ++i) {
    delayMicroseconds(2);
    pixels[i] = analogRead(AO_PIN);
    digitalWrite(CLK_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(CLK_PIN, HIGH); // last iteration produces rising edge 129
  }
  delayMicroseconds(2);
  digitalWrite(CLK_PIN, LOW);
  delayMicroseconds(20); // charge transfer recovery
}

void readCamera() {
  // Discard accumulated exposure from Serial/ToF delays, then acquire a fresh line.
  scanLine();
  delay(CAMERA_EXTRA_EXPOSURE_MS);
  scanLine();
}

void updateDetection(bool valid, bool present) {
  unsigned long now = millis();
  if (!valid) {
    timingPresent = timingAbsent = false;
    return; // invalid measurements cannot trigger OR rearm
  }
  if (present) {
    timingAbsent = false;
    if (!timingPresent) { timingPresent = true; presentSince = now; }
    if (armed && now - presentSince >= PRESENT_DWELL_MS) {
      Serial.println(F("DELIVERY_REQUEST (detection only)"));
      armed = false;
    }
  } else {
    timingPresent = false;
    if (!timingAbsent) { timingAbsent = true; absentSince = now; }
    if (now - absentSince >= ABSENT_DWELL_MS) armed = true;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(SI_PIN, OUTPUT); pinMode(CLK_PIN, OUTPUT);
  digitalWrite(SI_PIN, LOW); digitalWrite(CLK_PIN, LOW);
  Wire.begin(); Wire.setClock(100000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000, true);
#endif
  tof.setTimeout(200);
  if (!tof.init()) {
    Serial.println(F("ToF initialization failed: check power and I2C wiring."));
    while (true) { delay(100); }
  }
  tof.setDistanceMode(VL53L1X::Short);
  tof.setMeasurementTimingBudget(50000);
  tof.startContinuous(60);
  Serial.println(F("b = capture EMPTY scene background; l = dump 128 pixels."));
  Serial.println(F("Set BACK_MIN_MM and BACK_MAX_MM to enable events."));
}

void loop() {
  uint16_t mm = tof.read();
  bool valid = !tof.timeoutOccurred() && tof.last_status == 0 &&
               tof.ranging_data.range_status == VL53L1X::RangeValid;
  readCamera();
  char command = Serial.available() ? Serial.read() : 0;
  if (command == 'b') {
    for (uint8_t i = 0; i < 128; ++i) background[i] = pixels[i];
    haveBackground = true;
    timingPresent = timingAbsent = false;
    Serial.println(F("Empty-scene baseline captured."));
  }
  uint16_t changed = 0, low = 1023, high = 0;
  for (uint8_t i = 0; i < 128; ++i) {
    if (pixels[i] < low) low = pixels[i];
    if (pixels[i] > high) high = pixels[i];
    if (haveBackground && i >= ROI_FIRST && i <= ROI_LAST &&
        abs((int)pixels[i] - (int)background[i]) >= PIXEL_CHANGE_THRESHOLD) ++changed;
    if (command == 'l') { Serial.print(pixels[i]); Serial.print(i == 127 ? '\n' : ','); }
  }
  bool configured = BACK_MAX_MM > BACK_MIN_MM && BACK_MAX_MM != 0;
  bool inZone = mm >= BACK_MIN_MM && mm <= BACK_MAX_MM;
  bool cameraPresent = haveBackground && changed >= MIN_CHANGED_PIXELS;
  updateDetection(valid && configured && (!REQUIRE_CAMERA_CHANGE || haveBackground),
                  inZone && (!REQUIRE_CAMERA_CHANGE || cameraPresent));
  Serial.print(F("mm=")); Serial.print(mm);
  Serial.print(F(" valid=")); Serial.print(valid);
  Serial.print(F(" changedPixels=")); Serial.print(changed);
  Serial.print(F(" pixelMin=")); Serial.print(low);
  Serial.print(F(" pixelMax=")); Serial.println(high);
}
