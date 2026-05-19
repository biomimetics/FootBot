#include "MultiAxisStepper.h"

static const unsigned long NEVER_STEP_TIME = 0xFFFFFFFFUL;

// Public: create the multi-axis controller using existing motor objects.
MultiAxisStepper::MultiAxisStepper(StepperMotor& xMotor, StepperMotor& yMotor, StepperMotor& zMotor)
  : _motors{ &xMotor, &yMotor, &zMotor }
{
  _isBusy = false;
  for (int i = 0; i < AXIS_COUNT; i++) {
    _position[i] = 0;
    _remaining[i] = 0;
    _direction[i] = HIGH;
    _speed[i] = 1000;
    _nextStepTime[i] = NEVER_STEP_TIME;
    _active[i] = false;
  }
}

// Public: set the speed for a single axis.
void MultiAxisStepper::setSpeed(Axis axis, unsigned int speedMicroseconds) {
  if (axis < 0 || axis >= AXIS_COUNT) {
    return;
  }
  _speed[axis] = speedMicroseconds < 1 ? 1 : speedMicroseconds;
}

// Public: set speeds for all axes at once.
void MultiAxisStepper::setSpeeds(unsigned int speedX, unsigned int speedY, unsigned int speedZ) {
  setSpeed(X_AXIS, speedX);
  setSpeed(Y_AXIS, speedY);
  setSpeed(Z_AXIS, speedZ);
}

void MultiAxisStepper::setPulseWidth(Axis axis, unsigned int pulseWidthMicroseconds) {
  if (axis < 0 || axis >= AXIS_COUNT) {
    return;
  }
  _motors[axis]->setPulseWidth(pulseWidthMicroseconds);
}

void MultiAxisStepper::setPulseWidths(unsigned int pulseWidthX, unsigned int pulseWidthY, unsigned int pulseWidthZ) {
  setPulseWidth(X_AXIS, pulseWidthX);
  setPulseWidth(Y_AXIS, pulseWidthY);
  setPulseWidth(Z_AXIS, pulseWidthZ);
}

// Public: begin a non-blocking relative move.
void MultiAxisStepper::moveBy(long xSteps, long ySteps, long zSteps) {
  long steps[AXIS_COUNT] = { xSteps, ySteps, zSteps };
  startMoveInternal(steps);
}

// Public: begin a non-blocking move to an absolute position.
void MultiAxisStepper::moveTo(long xPosition, long yPosition, long zPosition) {
  long steps[AXIS_COUNT] = {
    xPosition - _position[X_AXIS],
    yPosition - _position[Y_AXIS],
    zPosition - _position[Z_AXIS]
  };
  startMoveInternal(steps);
}

// Public: begin a blocking relative move until completion.
void MultiAxisStepper::moveByBlocking(long xSteps, long ySteps, long zSteps) {
  moveBy(xSteps, ySteps, zSteps);
  while (busy()) {
    update();
  }
}

// Public: begin a blocking absolute move until completion.
void MultiAxisStepper::moveToBlocking(long xPosition, long yPosition, long zPosition) {
  moveTo(xPosition, yPosition, zPosition);
  while (busy()) {
    update();
  }
}

// Public: update the motion scheduler; call frequently from loop().
void MultiAxisStepper::update() {
  if (!_isBusy) {
    return;
  }

  unsigned long now = micros();
  _isBusy = false;

  for (int i = 0; i < AXIS_COUNT; i++) {
    if (!_active[i]) {
      continue;
    }

    if ((long)(now - _nextStepTime[i]) >= 0) {
      _motors[i]->stepOnce(_direction[i]);
      _position[i] += (_direction[i] == HIGH) ? 1 : -1;
      _remaining[i]--;
      if (_remaining[i] == 0) {
        _active[i] = false;
        _nextStepTime[i] = NEVER_STEP_TIME;
      } else {
        _nextStepTime[i] = now + (unsigned long)_speed[i] * 2UL;
      }
    }

    if (_active[i]) {
      _isBusy = true;
    }
  }
}

// Public: check whether a move is still in progress.
bool MultiAxisStepper::busy() const {
  return _isBusy;
}

// Public: cancel any active move immediately.
void MultiAxisStepper::stop() {
  for (int i = 0; i < AXIS_COUNT; i++) {
    _remaining[i] = 0;
    _active[i] = false;
    _nextStepTime[i] = NEVER_STEP_TIME;
  }
  _isBusy = false;
}

// Public: enable all motors in the controller.
void MultiAxisStepper::enable() {
  for (int i = 0; i < AXIS_COUNT; i++) {
    _motors[i]->enable();
  }
}

// Public: disable all motors in the controller.
void MultiAxisStepper::disable() {
  for (int i = 0; i < AXIS_COUNT; i++) {
    _motors[i]->disable();
  }
}

// Public: read the last-known position for an axis.
long MultiAxisStepper::getPosition(Axis axis) const {
  if (axis < 0 || axis >= AXIS_COUNT) {
    return 0;
  }
  return _position[axis];
}

// Internal helper: initialize move state for the scheduler.
void MultiAxisStepper::startMoveInternal(long steps[AXIS_COUNT]) {
  _isBusy = false;

  for (int i = 0; i < AXIS_COUNT; i++) {
    _remaining[i] = absolute(steps[i]);
    _direction[i] = (steps[i] >= 0) ? HIGH : LOW;
    _active[i] = (_remaining[i] > 0);
    _nextStepTime[i] = _active[i] ? micros() : NEVER_STEP_TIME;
    if (_active[i]) {
      _isBusy = true;
    }
  }
}

long MultiAxisStepper::absolute(long value) {
  return value < 0 ? -value : value;
}
