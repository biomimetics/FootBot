#include "StageController.h"

const StageController::Axis StageController::X_AXIS = static_cast<StageController::Axis>(MultiAxisStepper::X_AXIS);
const StageController::Axis StageController::Y_AXIS = static_cast<StageController::Axis>(MultiAxisStepper::Y_AXIS);
const StageController::Axis StageController::Z_AXIS = static_cast<StageController::Axis>(MultiAxisStepper::Z_AXIS);

StageController::StageController(MultiAxisStepper& stepper,
                                 const float stepsPerRev[AXIS_COUNT],
                                 const float mmPerRev[AXIS_COUNT])
  : _stepper(stepper)
{
  for (int i = 0; i < AXIS_COUNT; i++) {
    _stepsPerRev[i] = stepsPerRev[i];
    _mmPerRev[i] = mmPerRev[i];
    updateCalibration(static_cast<Axis>(i));
  }
}

void StageController::setCalibration(Axis axis, float stepsPerRev, float mmPerRev) {
  if (axis < 0 || axis >= AXIS_COUNT) {
    return;
  }
  _stepsPerRev[axis] = stepsPerRev;
  _mmPerRev[axis] = mmPerRev;
  updateCalibration(axis);
}

void StageController::moveByMm(float xMm, float yMm, float zMm) {
  long xSteps = mmToSteps(xMm, static_cast<Axis>(0));
  long ySteps = mmToSteps(yMm, static_cast<Axis>(1));
  long zSteps = mmToSteps(zMm, static_cast<Axis>(2));
  _stepper.moveBy(xSteps, ySteps, zSteps);
}

void StageController::moveToMm(float xMm, float yMm, float zMm) {
  long xSteps = mmToSteps(xMm, static_cast<Axis>(0));
  long ySteps = mmToSteps(yMm, static_cast<Axis>(1));
  long zSteps = mmToSteps(zMm, static_cast<Axis>(2));
  _stepper.moveTo(xSteps, ySteps, zSteps);
}

void StageController::moveByMmBlocking(float xMm, float yMm, float zMm) {
  moveByMm(xMm, yMm, zMm);
  while (busy()) {
    update();
  }
}

void StageController::moveToMmBlocking(float xMm, float yMm, float zMm) {
  moveToMm(xMm, yMm, zMm);
  while (busy()) {
    update();
  }
}

void StageController::setSpeed(Axis axis, unsigned int speedMicroseconds) {
  _stepper.setSpeed(static_cast<MultiAxisStepper::Axis>(axis), speedMicroseconds);
}

void StageController::setSpeeds(unsigned int speedX, unsigned int speedY, unsigned int speedZ) {
  _stepper.setSpeeds(speedX, speedY, speedZ);
}

void StageController::setPulseWidth(Axis axis, unsigned int pulseWidthMicroseconds) {
  _stepper.setPulseWidth(static_cast<MultiAxisStepper::Axis>(axis), pulseWidthMicroseconds);
}

void StageController::setPulseWidths(unsigned int pulseWidthX, unsigned int pulseWidthY, unsigned int pulseWidthZ) {
  _stepper.setPulseWidths(pulseWidthX, pulseWidthY, pulseWidthZ);
}

void StageController::update() {
  _stepper.update();
}

bool StageController::busy() const {
  return _stepper.busy();
}

void StageController::stop() {
  _stepper.stop();
}

void StageController::enable() {
  _stepper.enable();
}

void StageController::disable() {
  _stepper.disable();
}

float StageController::getPositionMm(Axis axis) const {
  return stepsToMm(_stepper.getPosition(static_cast<MultiAxisStepper::Axis>(axis)), axis);
}

long StageController::getPositionSteps(Axis axis) const {
  return _stepper.getPosition(static_cast<MultiAxisStepper::Axis>(axis));
}

void StageController::updateCalibration(Axis axis) {
  if (_mmPerRev[axis] == 0.0f) {
    _stepsPerMm[axis] = 0.0f;
  } else {
    _stepsPerMm[axis] = _stepsPerRev[axis] / _mmPerRev[axis];
  }
}

long StageController::mmToSteps(float mm, Axis axis) const {
  if (axis < 0 || axis >= AXIS_COUNT) {
    return 0;
  }
  return long(round(mm * _stepsPerMm[axis]));
}

float StageController::stepsToMm(long steps, Axis axis) const {
  if (axis < 0 || axis >= AXIS_COUNT) {
    return 0.0f;
  }
  if (_stepsPerMm[axis] == 0.0f) {
    return 0.0f;
  }
  return float(steps) / _stepsPerMm[axis];
}
