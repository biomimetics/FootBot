# FootBot

PlatformIO firmware for an Arduino Mega (`megaatmega2560`) controlling a 3-axis
(X/Y/Z) stepper-driven mechanical test stage, plus a Python-based PC-side
serial interface for driving it.

## Where to look

- [`src/`](src/) — firmware source (`StepperMotor`, `MultiAxisStepper`,
  `StageController`, `SerialComms`) and `stepper_control_test.ino`, the entry
  point wiring these together.
- [`include/`](include/) — headers for the above, plus
  [`include/README.md`](include/README.md), which documents the serial
  command protocol used to talk to the board.
- [`scripts/StageCommandComms.py`](scripts/StageCommandComms.py) — Python
  client implementing that same protocol, for one-off commands or interactive
  control from a PC. Usage examples are in [`include/README.md`](include/README.md).
- [`lib/`](lib/) — private/project-specific libraries (currently empty; see
  [`lib/README`](lib/README) for the PlatformIO convention).
- [`test/`](test/) — PlatformIO unit tests (see [`test/README`](test/README)).
- [`archive/`](archive/) — earlier non-PlatformIO Arduino and MATLAB code kept
  for reference.
- [`platformio.ini`](platformio.ini) — build/board/environment configuration.

## Building

Standard PlatformIO workflow (`pio run`, `pio run -t upload`) targeting the
`megaatmega2560` environment defined in `platformio.ini`.
