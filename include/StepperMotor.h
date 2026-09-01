#ifndef StepperMotor_h
#define StepperMotor_h

#include <Arduino.h>

class StepperMotor {
  public:
    StepperMotor(int dirPin, int pulPin, int enaPin);
    void setSpeed(int speed);
    void setPulseWidth(unsigned int pulseWidthMicroseconds);
    void step(int steps);
    void stepOnce(int direction);
    void enable();
    void disable();

  private:
    int _dirPin;
    int _pulPin;
    int _enaPin;
    int _speed;
    unsigned int _pulseWidth;
};

#endif