#include "StepperMotor.h"

// Public: create a stepper motor driver instance and initialize pins.
StepperMotor::StepperMotor(int dirPin, int pulPin, int enaPin) {
  _dirPin = dirPin;
  _pulPin = pulPin;
  _enaPin = enaPin;
  _speed = 1000;
  _pulseWidth = 2;

  pinMode(_dirPin, OUTPUT);
  pinMode(_pulPin, OUTPUT);
  pinMode(_enaPin, OUTPUT);

  disable();
}

// Public: set the delay between step pulses (lower = faster).
void StepperMotor::setSpeed(int speed) {
  _speed = speed;
}

// Public: set the pulse width for a single STEP pulse.
void StepperMotor::setPulseWidth(unsigned int pulseWidthMicroseconds) {
  _pulseWidth = pulseWidthMicroseconds < 1 ? 1 : pulseWidthMicroseconds;
}

// Internal helper: emit exactly one step pulse for non-blocking schedulers.
void StepperMotor::stepOnce(int direction) {
  digitalWrite(_dirPin, direction);
  digitalWrite(_pulPin, HIGH);
  delayMicroseconds(_pulseWidth);
  digitalWrite(_pulPin, LOW);
}

// Public: execute a blocking multi-step movement.
void StepperMotor::step(int steps) {
  int direction = (steps >= 0) ? HIGH : LOW;
  digitalWrite(_dirPin, direction);
  steps = abs(steps);

  for (int i = 0; i < steps; i++) {
    digitalWrite(_pulPin, HIGH);
    delayMicroseconds(_pulseWidth);
    digitalWrite(_pulPin, LOW);
    delayMicroseconds(_pulseWidth);
    delayMicroseconds(_speed);
  }
}

// Public: enable the motor driver output.
void StepperMotor::enable() {
  digitalWrite(_enaPin, LOW);
}

// Public: disable the motor driver output.
void StepperMotor::disable() {
  digitalWrite(_enaPin, HIGH);
}


