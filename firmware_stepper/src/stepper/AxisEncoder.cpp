#include "AxisEncoder.h"

AxisEncoder::AxisEncoder()
  : _encoder(nullptr), _mmPerCount(0.0f)
{
}

void AxisEncoder::begin(bool enabled, int pinA, int pinB,
                        float pulsesPerRev, float countsPerPulse, float mmPerRev) {
  if (!enabled || pinA < 0 || pinB < 0) {
    return;
  }

  float countsPerRev = pulsesPerRev * countsPerPulse;
  if (countsPerRev <= 0.0f) {
    return;
  }

  _mmPerCount = mmPerRev / countsPerRev;
  _encoder = new Encoder(pinA, pinB);
}

bool AxisEncoder::isEnabled() const {
  return _encoder != nullptr;
}

long AxisEncoder::getCounts() const {
  return _encoder == nullptr ? 0L : _encoder->read();
}

float AxisEncoder::getPositionMm() const {
  return _encoder == nullptr ? 0.0f : (float)_encoder->read() * _mmPerCount;
}

void AxisEncoder::zero() {
  if (_encoder != nullptr) {
    _encoder->write(0);
  }
}
