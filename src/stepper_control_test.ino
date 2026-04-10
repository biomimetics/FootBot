#include "StepperMotor.h"
#include "MultiAxisStepper.h"
#include "StageController.h"
#include "SerialComms.h"
//#include <Encoder.h>

#include "common_constants.h"

// define stepper motor objects
StepperMotor motor_x(MOTOR_X_PIN_DIR, MOTOR_X_PIN_PULSE, MOTOR_X_PIN_ENABLE);
StepperMotor motor_y(MOTOR_Y_PIN_DIR, MOTOR_Y_PIN_PULSE, MOTOR_Y_PIN_ENABLE);
StepperMotor motor_z(MOTOR_Z_PIN_DIR, MOTOR_Z_PIN_PULSE, MOTOR_Z_PIN_ENABLE);

// define helper object for multi-axis control
MultiAxisStepper multi_axis_stepper(motor_x, motor_y, motor_z);

// define helper object for directly controlling stage position
StageController stage_controller(multi_axis_stepper, 
                                 (float[AXIS_COUNT]){MOTOR_X_STEPS_PER_REV, MOTOR_Y_STEPS_PER_REV, MOTOR_Z_STEPS_PER_REV},
                                 (float[AXIS_COUNT]){MOTOR_X_MM_PER_REV, MOTOR_Y_MM_PER_REV, MOTOR_Z_MM_PER_REV});

// define serial communication object
SerialComms serialComms(Serial);

void setup()
{
  Serial.begin(BAUD_RATE);
  serialComms.begin();

  
  int initial_speed = 2000; // microseconds between steps (lower = faster)
  stage_controller.setSpeeds(initial_speed, initial_speed, initial_speed);

  int initial_pulse_width = 2000; // microseconds
  stage_controller.setPulseWidths(initial_pulse_width, initial_pulse_width, initial_pulse_width);

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
