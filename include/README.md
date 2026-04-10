# Utils Documentation

This folder contains shared stage control utilities for the BML XYZ stage project.

## Arduino Serial Interface

The Arduino side exposes a high-level serial command protocol through `SerialComms.h` and `SerialComms.cpp`.

### Supported commands

- `MOVE_REL,<x_mm>,<y_mm>,<z_mm>`
  - Perform a relative stage movement in millimeters.

- `MOVE_ABS,<x_mm>,<y_mm>,<z_mm>`
  - Perform an absolute stage movement to the given position in millimeters.

- `SET_SPEED,<speed_x>,<speed_y>,<speed_z>`
  - Set the step speed for each axis.

- `STOP`
  - Immediately stop stage motion.

- `ENABLE`
  - Enable the stage motors.

- `DISABLE`
  - Disable the stage motors.

- `QUERY_POS`
  - Request the current stage position.

### Arduino feedback format

The Arduino responds with line-oriented messages enclosed in angle brackets.

- Position feedback:
  - `<POSITION,<x_mm>,<y_mm>,<z_mm>>`

- Status response:
  - `<STATUS,<message>>`

- Error response:
  - `<ERROR,<message>>`

## Python PC Interface

The PC-side helper class is implemented in `StageCommandComms.py`.
It is compatible with the Arduino `SerialComms` protocol and supports both one-off command invocation and an interactive command mode.

### Requirements

- Python 3
- `pyserial`

Install with:

```bash
pip install pyserial
```

### Usage examples

Open a serial port and send a query position command:

```bash
python utils/StageCommandComms.py --port COM3 --baud 115200 --query-pos
```

Send a relative move command:

```bash
python utils/StageCommandComms.py --port COM3 --move-rel 1.0 0.0 -0.5
```

Send a speed update:

```bash
python utils/StageCommandComms.py --port COM3 --set-speed 2000 2000 2000
```

Enter interactive mode:

```bash
python utils/StageCommandComms.py --port COM3 --interactive
```

### Interactive mode commands

- `move_rel X Y Z`
- `move_abs X Y Z`
- `set_speed X Y Z`
- `stop`
- `enable`
- `disable`
- `query_pos`
- `raw <COMMAND>`
- `quit`

### Python class example

```python
from utils.StageCommandComms import StageCommandComms

comms = StageCommandComms(port='COM3', baudrate=115200)
comms.move_relative(1.0, 0.0, -0.5)
response = comms.wait_for_response()
print(response)
comms.close()
```
