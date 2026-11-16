#include <Wire.h>
#include "VL53L1X.h"
#include <AccelStepper.h>
#include <math.h>


// Trigger distance
const uint16_t TRIGGER_DISTANCE_MM = 400; //0.4m

// door - 28BYJ-48 - change for new motor
const unsigned long MOTOR_FORWARD_MS = 3000;
const unsigned long MOTOR_PAUSE_MS   = 1000;
const unsigned long MOTOR_REVERSE_MS = 3000;

// stepper movement - NEMA 17 motor, should work with NEMA 14 but start in configuration
const unsigned long NEMA_DELAY_MS = 3000; // door activation delay
const float NEMA_MOVE_DEGREES = 90.0;      // angle change

// full system delay
const unsigned long COOLDOWN_MS = 2000;

// 28BYJ-48 speed in 1/2 steps/s
// change the sign to reverse direction
const float MOTOR_FORWARD_SPEED = 500.0;

// NEMA configuration, check correct numbers per each motor
// 200 full steps/rev for NEMA 17
// microstepping is set on HW135, change if using different driver
const int NEMA_FULL_STEPS_PER_REV = 200;
const int NEMA_MICROSTEPS = 1;

// NEMA speed and acceleration are in driver pulses/s and pulses/s^2.
const float NEMA_MAX_SPEED = 400.0;
const float NEMA_ACCELERATION = 800.0;

// current pins

const uint8_t NEMA_STEP_PIN = 2;
const uint8_t NEMA_DIR_PIN  = 3;

const uint8_t MOTOR_IN1_PIN = 8;
const uint8_t MOTOR_IN2_PIN = 9;
const uint8_t MOTOR_IN3_PIN = 10;
const uint8_t MOTOR_IN4_PIN = 11;



// DEVICES

VL53L1X sensor;

AccelStepper nemaMotor(
  AccelStepper::DRIVER,
  NEMA_STEP_PIN,
  NEMA_DIR_PIN
);

//  28BYJ-48 + ULN2003  - Replace with James' system
// stepper pin order is IN1, IN3, IN2, IN4.
AccelStepper smallMotor(
  AccelStepper::HALF4WIRE,
  MOTOR_IN1_PIN,
  MOTOR_IN3_PIN,
  MOTOR_IN2_PIN,
  MOTOR_IN4_PIN
);

enum Phase {
  IDLE,
  FORWARD,
  PAUSE,
  REVERSE,
  COOLDOWN
};

Phase phase = IDLE;

unsigned long activationTime = 0;
unsigned long phaseStartTime = 0;

bool nemaMoveStarted = false;
long nemaMoveSteps = 0;

// START ONE CYCLE

void startCycle(unsigned long now) {
  activationTime = now;
  phaseStartTime = now;
  nemaMoveStarted = false;

  smallMotor.enableOutputs();
  smallMotor.setSpeed(MOTOR_FORWARD_SPEED);

  phase = FORWARD;
}



void setup() {
  Serial.begin(115200);
  Wire.begin();

  smallMotor.setMaxSpeed(fabs(MOTOR_FORWARD_SPEED));
  smallMotor.setSpeed(0);
  smallMotor.disableOutputs();

  nemaMotor.setMaxSpeed(NEMA_MAX_SPEED);
  nemaMotor.setAcceleration(NEMA_ACCELERATION);
  nemaMotor.setMinPulseWidth(3); // Microseconds

  nemaMoveSteps = lround(
    (NEMA_MOVE_DEGREES / 360.0) *
    NEMA_FULL_STEPS_PER_REV *
    NEMA_MICROSTEPS
  );

  sensor.setTimeout(500);

  if (!sensor.init()) {
    Serial.println(F("VL53L1X not detected"));
    while (true) {
    }
  }

  sensor.setDistanceMode(VL53L1X::Short);
  sensor.setMeasurementTimingBudget(50000); // Microseconds
  sensor.startContinuous(50);               // Milliseconds

  Serial.println(F("start"));
}


void loop() {
  unsigned long now = millis();

  // check delay
  if (phase != IDLE &&
      !nemaMoveStarted &&
      now - activationTime >= NEMA_DELAY_MS) {

    // set current position 0, increment 90 degrees. Fernando positioning system will go here
    nemaMotor.setCurrentPosition(0);
    nemaMotor.moveTo(nemaMoveSteps);
    nemaMoveStarted = true;
  }

  //28BYJ-48
  switch (phase) {
    case IDLE:
      break;

    case FORWARD:
      if (now - phaseStartTime >= MOTOR_FORWARD_MS) {
        smallMotor.setSpeed(0);
        phase = PAUSE;
        phaseStartTime = now;
      }
      break;

    case PAUSE:
      if (now - phaseStartTime >= MOTOR_PAUSE_MS) {
        smallMotor.setSpeed(-MOTOR_FORWARD_SPEED);
        phase = REVERSE;
        phaseStartTime = now;
      }
      break;

    case REVERSE:
      if (now - phaseStartTime >= MOTOR_REVERSE_MS) {
        smallMotor.setSpeed(0);

        // disable after first movement [closing]
        smallMotor.disableOutputs();

        phase = COOLDOWN;
        phaseStartTime = now;
      }
      break;

    case COOLDOWN:
      // check full cooldown in case something gets skipped
      if (now - phaseStartTime >= COOLDOWN_MS &&
          nemaMoveStarted &&
          nemaMotor.distanceToGo() == 0) {
        phase = IDLE;
      }
      break;
  }

  // status
  nemaMotor.run();

  if (phase == FORWARD || phase == REVERSE) {
    smallMotor.runSpeed();
  }

  //prevent cycle triggering if spinning or cooldown phase
  if (sensor.dataReady()) {
    uint16_t distanceMM = sensor.read(false);

    bool validReading =
      !sensor.timeoutOccurred() &&
      sensor.ranging_data.range_status == VL53L1X::RangeValid;

    if (phase == IDLE &&
        validReading &&
        distanceMM > 0 &&
        distanceMM <= TRIGGER_DISTANCE_MM) {
      startCycle(millis());
    }
  }
}
