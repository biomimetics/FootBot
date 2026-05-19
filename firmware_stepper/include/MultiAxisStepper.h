#ifndef MultiAxisStepper_h
#define MultiAxisStepper_h

#include <Arduino.h>
#include "StepperMotor.h"

class MultiAxisStepper {
  public:
    enum Axis {
      X_AXIS = 0,
      Y_AXIS = 1,
      Z_AXIS = 2,
      AXIS_COUNT = 3
    };

    MultiAxisStepper(StepperMotor& xMotor, StepperMotor& yMotor, StepperMotor& zMotor);

    void setSpeed(Axis axis, unsigned int speedMicroseconds);
    void setSpeeds(unsigned int speedX, unsigned int speedY, unsigned int speedZ);
    void setPulseWidth(Axis axis, unsigned int pulseWidthMicroseconds);
    void setPulseWidths(unsigned int pulseWidthX, unsigned int pulseWidthY, unsigned int pulseWidthZ);

    void moveBy(long xSteps, long ySteps, long zSteps);
    void moveTo(long xPosition, long yPosition, long zPosition);
    void moveByBlocking(long xSteps, long ySteps, long zSteps);
    void moveToBlocking(long xPosition, long yPosition, long zPosition);

    void update();
    bool busy() const;
    void stop();

    void enable();
    void disable();

    long getPosition(Axis axis) const;

  private:
    StepperMotor* _motors[AXIS_COUNT];
    long _position[AXIS_COUNT];
    long _remaining[AXIS_COUNT];
    int _direction[AXIS_COUNT];
    unsigned int _speed[AXIS_COUNT];
    unsigned long _nextStepTime[AXIS_COUNT];
    bool _active[AXIS_COUNT];
    bool _isBusy;

    void startMoveInternal(long steps[AXIS_COUNT]);
    static long absolute(long value);
};

#endif
