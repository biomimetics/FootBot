#ifndef StageController_h
#define StageController_h

#include <Arduino.h>
#include "MultiAxisStepper.h"

class StageController {
  public:
    using Axis = MultiAxisStepper::Axis;
    static const Axis X_AXIS;
    static const Axis Y_AXIS;
    static const Axis Z_AXIS;
    static const int AXIS_COUNT = MultiAxisStepper::AXIS_COUNT;

    StageController(MultiAxisStepper& stepper,
                    const float stepsPerRev[AXIS_COUNT],
                    const float mmPerRev[AXIS_COUNT]);

    void setCalibration(Axis axis, float stepsPerRev, float mmPerRev);

    void moveByMm(float xMm, float yMm, float zMm);
    void moveToMm(float xMm, float yMm, float zMm);
    void moveByMmBlocking(float xMm, float yMm, float zMm);
    void moveToMmBlocking(float xMm, float yMm, float zMm);

    void setSpeed(Axis axis, unsigned int speedMicroseconds);
    void setSpeeds(unsigned int speedX, unsigned int speedY, unsigned int speedZ);
    void setPulseWidth(Axis axis, unsigned int pulseWidthMicroseconds);
    void setPulseWidths(unsigned int pulseWidthX, unsigned int pulseWidthY, unsigned int pulseWidthZ);

    void update();
    bool busy() const;
    void stop();

    void enable();
    void disable();

    float getPositionMm(Axis axis) const;
    long getPositionSteps(Axis axis) const;

  private:
    MultiAxisStepper& _stepper;
    float _stepsPerRev[AXIS_COUNT];
    float _mmPerRev[AXIS_COUNT];
    float _stepsPerMm[AXIS_COUNT];

    void updateCalibration(Axis axis);
    long mmToSteps(float mm, Axis axis) const;
    float stepsToMm(long steps, Axis axis) const;
};

#endif
