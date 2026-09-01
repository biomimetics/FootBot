/* // common.h
* Header file that contains pin definitions and constants for the stepper motor controller
*/

#define BAUD_RATE 19200

#define AXIS_COUNT 3
// x direction stepper motor pins
#define MOTOR_X_PIN_PULSE 27 //define Pulse pin (GREEN)
#define MOTOR_X_PIN_DIR 25 //define Direction pin (BLUE)
#define MOTOR_X_PIN_ENABLE 23 //define Enable Pin (WHITE)
#define MOTOR_X_STEPS_PER_REV 200 // steps per revolution
#define MOTOR_X_MM_PER_REV 1.27 // mm per revolution

// y direction stepper motor pins
#define MOTOR_Y_PIN_PULSE 22 //define Pulse pin (GREEN)
#define MOTOR_Y_PIN_DIR 24 //define Direction pin (BLUE)
#define MOTOR_Y_PIN_ENABLE 26 //define Enable Pin (WHITE)
#define MOTOR_Y_STEPS_PER_REV 200 // steps per revolution
#define MOTOR_Y_MM_PER_REV 1.27 // mm per revolution

// z direction stepper motor pins
#define MOTOR_Z_PIN_PULSE 35 //define Pulse pin (GREEN)
#define MOTOR_Z_PIN_DIR 33 //define Direction pin (BLUE)
#define MOTOR_Z_PIN_ENABLE 31 //define Enable Pin (WHITE)
#define MOTOR_Z_STEPS_PER_REV 200 // steps per revolution
#define MOTOR_Z_MM_PER_REV 1.27 // mm per revolution

// encoder info - NOTE: currently only the z-axis encoder is attached. This should be updated if we mount an encoder on there.
// UNUSED
#define ENCODER_X_PPR 2000.0
#define ENCODER_X_PIN_A 2
#define ENCODER_X_PIN_B 3
#define ENCODER_X_DEG_TO_DISTANCE 1.27 / 360.0 // 1.27 mm per revolution

// UNUSED
#define ENCODER_Y_PPR 2000.0
#define ENCODER_Y_PIN_A 2
#define ENCODER_Y_PIN_B 3
#define ENCODER_Y_DEG_TO_DISTANCE 1.27 / 360.0 // 1.27 mm per revolution

// ATTACHED BUT UNUSED
#define ENCODER_Z_PPR 2000.0
#define ENCODER_Z_PIN_A 2
#define ENCODER_Z_PIN_B 3
#define ENCODER_Z_DEG_TO_DISTANCE 1.27 / 360.0 // 1.27 mm per revolution