#!/usr/bin/env python3
"""Minimal single-axis stepper test.

Talks straight to the Arduino over pyserial (no GUI, no StageCommandComms) so
there is as little as possible between you and the motor.

Usage:
    python scripts/test_axis.py x                  # jog X +5 mm then -5 mm
    python scripts/test_axis.py y --dist 10        # Y, 10 mm out and back
    python scripts/test_axis.py z                  # Z, 2 mm (finer lead screw)
    python scripts/test_axis.py all                # X, then Y, then Z in turn
    python scripts/test_axis.py x --reps 4
    python scripts/test_axis.py y --speed 800      # slower pulses (us between steps)
    python scripts/test_axis.py x --port /dev/ttyACM2
    python scripts/test_axis.py z --no-return      # don't come back to start
"""

import argparse
import sys
import time

import serial
from serial.tools import list_ports

BAUD = 115200
BOOT_DELAY = 2.0          # Arduino resets when the port opens
ARDUINO_VENDOR_ID = 0x2341

# Index into the MOVE_REL,x,y,z argument list.
AXIS_INDEX = {"x": 0, "y": 1, "z": 2}

# Z uses a 1.27 mm/rev screw against 6.35 mm/rev on X and Y, so the same jog
# distance takes five times as many steps and five times as long. It also
# usually has the least travel, so default it shorter.
DEFAULT_DIST = {"x": 5.0, "y": 5.0, "z": 2.0}


def find_port():
    """Prefer the stable by-id path for the Arduino; fall back to any ACM device."""
    candidates = list(list_ports.comports())
    for port in candidates:
        if port.vid == ARDUINO_VENDOR_ID:
            if port.serial_number:
                return f"/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_{port.serial_number}-if00"
            return port.device
    for port in candidates:
        if "ACM" in port.device:
            return port.device
    return None


def move_command(axis, distance):
    """Build a MOVE_REL line that touches only the requested axis."""
    values = [0.0, 0.0, 0.0]
    values[AXIS_INDEX[axis]] = distance
    return "MOVE_REL,{},{},{}".format(*values)


def send(ser, line, label=None):
    print(f">>> {line}")
    ser.write((line + "\n").encode("utf-8"))
    ser.flush()
    drain(ser, label=label)


def drain(ser, settle=0.3, label=None):
    """Print every reply line until the board goes quiet."""
    deadline = time.time() + settle
    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        print(f"<<< {raw.decode('utf-8', errors='replace').strip()}")
        deadline = time.time() + settle
    if label:
        print(f"    ({label})")


def wait_for_move(ser, timeout):
    """Moves are blocking in the firmware; just poll position until it settles."""
    end = time.time() + timeout
    last = None
    while time.time() < end:
        ser.write(b"QUERY_POS\n")
        ser.flush()
        raw = ser.readline().decode("utf-8", errors="replace").strip()
        if raw:
            print(f"<<< {raw}")
            if raw == last:
                return
            last = raw
        time.sleep(0.25)
    print("    (timed out waiting for the move to finish)")


def read_encoders(ser, settle=0.3):
    """Print encoder readings. The board sends one ENCODER line per fitted axis."""
    ser.write(b"QUERY_ENC\n")
    ser.flush()
    deadline = time.time() + settle
    while time.time() < deadline:
        raw = ser.readline().decode("utf-8", errors="replace").strip()
        if not raw:
            continue
        print(f"<<< {raw}")
        if "INVALID_COMMAND" in raw:
            print("    (this firmware predates encoder support - reflash to use it)")
            return
        if "NO_ENCODERS" in raw:
            print("    (no encoders enabled - see ENCODER_*_ENABLED in common_constants.h)")
            return
        deadline = time.time() + settle


def jog_axis(ser, axis, distance, args):
    """Run the out-and-back cycles for one axis."""
    print(f"\n=== {axis.upper()} axis, {distance} mm ===")
    print("Only this axis should move. If another one does, it is a wiring fault:")
    print("the firmware never pulses an axis with zero steps remaining.")

    for rep in range(1, args.reps + 1):
        print(f"\n--- {axis.upper()} cycle {rep}/{args.reps} ---")
        print(f"Jogging {axis.upper()} +{distance} mm")
        send(ser, move_command(axis, distance))
        wait_for_move(ser, args.timeout)

        if not args.no_encoder:
            read_encoders(ser)

        if not args.no_return:
            print(f"Jogging {axis.upper()} -{distance} mm")
            send(ser, move_command(axis, -distance))
            wait_for_move(ser, args.timeout)

            if not args.no_encoder:
                read_encoders(ser)


def main():
    parser = argparse.ArgumentParser(description="Jog a single stage axis back and forth.")
    parser.add_argument("axis", nargs="?", default="x", choices=["x", "y", "z", "all"],
                        help="axis to test, or 'all' for X then Y then Z (default x)")
    parser.add_argument("--port", help="serial device (default: autodetect the Arduino)")
    parser.add_argument("--dist", type=float,
                        help="mm per jog (default 5 for X/Y, 2 for Z)")
    parser.add_argument("--reps", type=int, default=1, help="out-and-back cycles (default 1)")
    parser.add_argument("--speed", type=int, help="microseconds between steps; lower is faster (firmware default 500)")
    parser.add_argument("--no-return", action="store_true", help="only jog in the + direction")
    parser.add_argument("--timeout", type=float, default=30.0, help="seconds to wait for one move")
    parser.add_argument("--no-encoder", action="store_true",
                        help="skip encoder readouts (ENCODER,<axis>,<measured>,<counts>,<commanded>,<error>)")
    parser.add_argument("--pause", type=float, default=1.0,
                        help="seconds between axes when testing 'all' (default 1)")
    args = parser.parse_args()

    axes = ["x", "y", "z"] if args.axis == "all" else [args.axis]

    port = args.port or find_port()
    if not port:
        sys.exit("No serial port found. Is the Arduino plugged in?")
    print(f"Port: {port}")

    try:
        ser = serial.Serial(port, BAUD, timeout=1.0)
    except serial.SerialException as exc:
        sys.exit(f"Could not open {port}: {exc}")

    with ser:
        print(f"Waiting {BOOT_DELAY}s for the board to boot...")
        time.sleep(BOOT_DELAY)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        drain(ser, label="startup chatter")

        try:
            send(ser, "QUERY_POS", label="starting position")
            if args.speed:
                # SET_SPEED takes all three axes; apply the same value to each.
                send(ser, f"SET_SPEED,{args.speed},{args.speed},{args.speed}")
            send(ser, "ENABLE", label="drivers energized - the motors should now hold")

            if not args.no_encoder:
                # Zero the encoders so measured travel is comparable to the
                # commanded distance for this run.
                send(ser, "ZERO_ENC")

            for index, axis in enumerate(axes):
                distance = args.dist if args.dist is not None else DEFAULT_DIST[axis]
                jog_axis(ser, axis, distance, args)
                if index < len(axes) - 1 and args.pause > 0:
                    time.sleep(args.pause)

            send(ser, "QUERY_POS", label="final position")
        except KeyboardInterrupt:
            print("\nInterrupted - sending STOP")
            send(ser, "STOP")
        except OSError as exc:
            # Errno 5 here means the board re-enumerated mid-run (USB reset or brownout).
            sys.exit(f"\nSerial I/O failed ({exc}). The board most likely reset; replug and retry.")
        finally:
            try:
                send(ser, "DISABLE", label="drivers released")
            except OSError:
                pass


if __name__ == "__main__":
    main()
