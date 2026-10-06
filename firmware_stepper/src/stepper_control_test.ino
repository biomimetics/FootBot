#include "StepperMotor.h"
#include "MultiAxisStepper.h"
#include "StageController.h"
#include "SerialComms.h"
#include "AxisEncoder.h"

#include "common_constants.h"

// define stepper motor objects
StepperMotor motor_x(MOTOR_X_PIN_DIR, MOTOR_X_PIN_PULSE, MOTOR_X_PIN_ENABLE);
StepperMotor motor_y(MOTOR_Y_PIN_DIR, MOTOR_Y_PIN_PULSE, MOTOR_Y_PIN_ENABLE);
StepperMotor motor_z(MOTOR_Z_PIN_DIR, MOTOR_Z_PIN_PULSE, MOTOR_Z_PIN_ENABLE);

// define helper object for multi-axis control
MultiAxisStepper multi_axis_stepper(motor_x, motor_y, motor_z);

// define helper object for directly controlling stage position
StageController stage_controller(multi_axis_stepper, 
                                 (float[AXIS_COUNT]){MOTOR_X_STEPS_PER_REV * MOTOR_X_MICROSTEP, 
                                  MOTOR_Y_STEPS_PER_REV * MOTOR_Y_MICROSTEP, 
                                  MOTOR_Z_STEPS_PER_REV * MOTOR_Z_MICROSTEP},
                                 (float[AXIS_COUNT]){MOTOR_X_MM_PER_REV, MOTOR_Y_MM_PER_REV, MOTOR_Z_MM_PER_REV});

// define the read-only encoders (measure actual travel). Which axes are
// actually fitted is set by ENCODER_*_ENABLED in common_constants.h.
AxisEncoder encoders[AXIS_COUNT];
const char AXIS_NAMES[AXIS_COUNT] = { 'X', 'Y', 'Z' };

// define serial communication object
SerialComms serialComms(Serial);

void setup()
{
  Serial.begin(BAUD_RATE);
  serialComms.begin();

  stage_controller.setSpeeds(COMMON_SPEED, COMMON_SPEED, COMMON_SPEED);

  stage_controller.setPulseWidths(COMMON_PULSE_WIDTH, COMMON_PULSE_WIDTH, COMMON_PULSE_WIDTH);

  encoders[0].begin(ENCODER_X_ENABLED, ENCODER_X_PIN_A, ENCODER_X_PIN_B,
                    ENCODER_X_PPR, ENCODER_COUNTS_PER_PULSE, ENCODER_X_MM_PER_REV);
  encoders[1].begin(ENCODER_Y_ENABLED, ENCODER_Y_PIN_A, ENCODER_Y_PIN_B,
                    ENCODER_Y_PPR, ENCODER_COUNTS_PER_PULSE, ENCODER_Y_MM_PER_REV);
  encoders[2].begin(ENCODER_Z_ENABLED, ENCODER_Z_PIN_A, ENCODER_Z_PIN_B,
                    ENCODER_Z_PPR, ENCODER_COUNTS_PER_PULSE, ENCODER_Z_MM_PER_REV);

  serialComms.sendStatus("STARTING UP");
}

void loop()
{
  SerialComms::Command command;
  if (!serialComms.readCommand(command)) {
    return;
  }

  switch (command.type) {
    case SerialComms::COMMAND_MOVE_RELATIVE:
      stage_controller.moveByMm(command.xMm, command.yMm, command.zMm);
      serialComms.sendStatus("MOVE_RELATIVE");
      break;

    case SerialComms::COMMAND_MOVE_ABSOLUTE:
      stage_controller.moveToMm(command.xMm, command.yMm, command.zMm);
      serialComms.sendStatus("MOVE_ABSOLUTE");
      break;

    case SerialComms::COMMAND_SET_SPEED:
      stage_controller.setSpeeds(command.speedX, command.speedY, command.speedZ);
      serialComms.sendStatus("SET_SPEED");
      break;

    case SerialComms::COMMAND_STOP:
      stage_controller.stop();
      serialComms.sendStatus("STOP");
      break;

    case SerialComms::COMMAND_ENABLE:
      stage_controller.enable();
      serialComms.sendStatus("ENABLE");
      break;

    case SerialComms::COMMAND_DISABLE:
      stage_controller.disable();
      serialComms.sendStatus("DISABLE");
      break;

    case SerialComms::COMMAND_QUERY_ENCODER: {
      // One line per fitted axis. A board with no encoders answers with a
      // status line rather than silence, so the host can tell them apart.
      bool anyEnabled = false;
      for (int i = 0; i < AXIS_COUNT; i++) {
        if (!encoders[i].isEnabled()) {
          continue;
        }
        anyEnabled = true;
        serialComms.sendEncoderFeedback(
          AXIS_NAMES[i],
          encoders[i].getPositionMm(),
          encoders[i].getCounts(),
          stage_controller.getPositionMm(static_cast<MultiAxisStepper::Axis>(i)));
      }
      if (!anyEnabled) {
        serialComms.sendStatus("NO_ENCODERS");
      }
      break;
    }

    case SerialComms::COMMAND_ZERO_ENCODER:
      for (int i = 0; i < AXIS_COUNT; i++) {
        encoders[i].zero();
      }
      serialComms.sendStatus("ZERO_ENC");
      break;

    case SerialComms::COMMAND_QUERY_POSITION:
      serialComms.sendPositionFeedback(
        stage_controller.getPositionMm(MultiAxisStepper::X_AXIS),
        stage_controller.getPositionMm(MultiAxisStepper::Y_AXIS),
        stage_controller.getPositionMm(MultiAxisStepper::Z_AXIS));
      break;

    default: {
      char errorMessage[128];
      if (command.raw[0] != '\0') {
        snprintf(errorMessage, sizeof(errorMessage), "INVALID_COMMAND,%s", command.raw);
      } else {
        strncpy(errorMessage, "INVALID_COMMAND", sizeof(errorMessage));
        errorMessage[sizeof(errorMessage) - 1] = '\0';
      }
      serialComms.sendError(errorMessage);
      break;
    }
  }

  while (stage_controller.busy()) {
    stage_controller.update();
  }
}
