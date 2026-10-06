/* // common.h
* Header file that contains pin definitions and constants for the stepper motor controller
*/

#define BAUD_RATE 115200

#define AXIS_COUNT 3
#define COMMON_SPEED 500 // microseconds between steps (lower = faster)
#define COMMON_PULSE_WIDTH 10 // microseconds for pulse width (controls max speed)

// x direction stepper motor pins
#define MOTOR_X_PIN_PULSE 27 //define Pulse pin (GREEN)
#define MOTOR_X_PIN_DIR 25 //define Direction pin (BLUE)
#define MOTOR_X_PIN_ENABLE 23 //define Enable Pin (WHITE)
#define MOTOR_X_STEPS_PER_REV 200 // steps per revolution
#define MOTOR_X_MICROSTEP 16 // steps per revolution multiplier
#define MOTOR_X_MM_PER_REV 6.35 // mm per revolution

// y direction stepper motor pins
#define MOTOR_Y_PIN_PULSE 22 //define Pulse pin (GREEN)
#define MOTOR_Y_PIN_DIR 24 //define Direction pin (BLUE)
#define MOTOR_Y_PIN_ENABLE 26 //define Enable Pin (WHITE)
#define MOTOR_Y_STEPS_PER_REV 200 // steps per revolution
#define MOTOR_Y_MICROSTEP 16 // steps per revolution multiplier
#define MOTOR_Y_MM_PER_REV 6.35 // mm per revolution

// z direction stepper motor pins
#define MOTOR_Z_PIN_PULSE 35 //define Pulse pin (GREEN)
#define MOTOR_Z_PIN_DIR 33 //define Direction pin (BLUE)
#define MOTOR_Z_PIN_ENABLE 31 //define Enable Pin (WHITE)
#define MOTOR_Z_STEPS_PER_REV 200 // steps per revolution
#define MOTOR_Z_MICROSTEP 16 // steps per revolution multiplier
#define MOTOR_Z_MM_PER_REV 1.27 // mm per revolution

// ---------------------------------------------------------------------------
// Encoders (read-only position feedback)
// ---------------------------------------------------------------------------
// Encoders measure where an axis actually went, independently of the step
// counter, which only records what was commanded. They do not feed back into
// motion control - the two numbers are reported side by side so a divergence
// is visible (missed steps, a stall, or de-energized drivers).
//
// TURN AN AXIS ENCODER ON OR OFF HERE. Set to 1 only once the encoder is
// physically fitted AND its pins below are correct. A disabled axis costs no
// pins, no interrupts and no RAM - its Encoder object is never constructed.
#define ENCODER_X_ENABLED 0
#define ENCODER_Y_ENABLED 0
#define ENCODER_Z_ENABLED 1

// Channel pins. For full 4x quadrature decoding both channels must be on
// external-interrupt pins; on the Mega 2560 those are 2, 3, 18, 19, 20 and 21.
// One interrupt pin plus one ordinary pin still works but loses counts at speed.
// Note pins 2 and 3 are free - no motor uses them (motors are on 22-27, 31-35).
#define ENCODER_X_PIN_A -1  // not fitted - set real pins before enabling
#define ENCODER_X_PIN_B -1
#define ENCODER_Y_PIN_A -1  // not fitted - set real pins before enabling
#define ENCODER_Y_PIN_B -1
#define ENCODER_Z_PIN_A 2   // INT0
#define ENCODER_Z_PIN_B 3   // INT1

// Encoder resolution per revolution.
#define ENCODER_X_PPR 2000.0
#define ENCODER_Y_PPR 2000.0
#define ENCODER_Z_PPR 2000.0

// Quadrature multiplier applied to the figure above. A decoder sees four edges
// per pulse cycle, so this is 4 when the datasheet quotes pulses per channel
// and 1 when it already quotes counts per revolution.
//
// Measured against hardware on the Z axis: a 5 mm commanded move read back
// 1.25 mm with this set to 4, i.e. exactly 1/4 of the truth, so the 2000 figure
// is already counts per revolution. Left at 1.
#define ENCODER_COUNTS_PER_PULSE 1.0

// Travel per encoder revolution. The encoder turns with the lead screw, so
// this is the screw lead - keep each in sync with the matching MOTOR_*_MM_PER_REV.
#define ENCODER_X_MM_PER_REV 6.35
#define ENCODER_Y_MM_PER_REV 6.35
#define ENCODER_Z_MM_PER_REV 1.27
