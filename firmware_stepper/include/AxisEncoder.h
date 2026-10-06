#ifndef AxisEncoder_h
#define AxisEncoder_h

#include <Arduino.h>
#include <Encoder.h>

// Read-only quadrature encoder for one axis.
//
// Measures where the axis actually is, independently of the commanded step
// count held by MultiAxisStepper. Nothing here feeds back into motion control.
//
// An axis whose encoder is not fitted is constructed disabled: begin() then
// allocates nothing and claims no pins or interrupts, and the accessors report
// zero. Enable axes in common_constants.h, not here.
class AxisEncoder {
  public:
    AxisEncoder();

    // Configure one axis. Call from setup() before any read.
    // pulsesPerRev is the encoder's own PPR figure; countsPerPulse is the
    // quadrature multiplier (4 for a full 4x decoder).
    void begin(bool enabled, int pinA, int pinB,
               float pulsesPerRev, float countsPerPulse, float mmPerRev);

    bool isEnabled() const;

    // Raw quadrature count since the last zero(). Zero when disabled.
    long getCounts() const;

    // Measured travel in mm since the last zero(). Zero when disabled.
    float getPositionMm() const;

    // Define the current physical location as zero. Call this alongside any
    // reset of the commanded step count so the two stay comparable.
    void zero();

  private:
    // Encoder attaches interrupts in its constructor, so it is only built for
    // an axis that is actually fitted. Heap-allocated once in begin() and never
    // released - an encoder lives for the lifetime of the sketch.
    Encoder* _encoder;
    float _mmPerCount;
};

#endif
