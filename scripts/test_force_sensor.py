#!/usr/bin/env python3
"""Simple tester for HX711 force sensor Arduino on /dev/ttyACM1.

Sends `QUERY_FORCE` and parses `<FORCE,<value>>` responses, printing values.
"""
import time
import sys

try:
    import serial
except ImportError:
    print("pyserial is required. Install with: python3 -m pip install pyserial")
    sys.exit(1)

PORT = "/dev/ttyACM2"
BAUD = 115200
POLL_INTERVAL = 0.5


def parse_line(line):
    line = line.strip()
    if line.startswith("<") and line.endswith(">"):
        line = line[1:-1]
    parts = [p.strip() for p in line.split(",")]
    if not parts:
        return None
    tag = parts[0].upper()
    if tag == "FORCE" and len(parts) >= 2:
        try:
            return float(parts[1])
        except ValueError:
            return None
    return None


def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as exc:
        print(f"Failed to open {PORT}: {exc}")
        return

    print(f"Connected to {PORT} at {BAUD} baud. Listening for stream. Ctrl-C to stop.")
    # Ensure streaming is enabled on the device
    try:
        ser.write(b"STREAM_ON\n")
    except Exception:
        pass
    try:
        # In streaming mode the Arduino continuously sends lines like <FORCE,123.456>
        while True:
            try:
                line = ser.readline().decode("utf-8", errors="replace").strip()
            except Exception as exc:
                print(f"Read failed: {exc}")
                break

            if not line:
                # no data within timeout
                continue

            # show timestamped raw line for debugging
            ts = time.strftime('%H:%M:%S')
            val = parse_line(line)
            if val is not None:
                print(f"{ts}  Force: {val:.3f}")
            else:
                # handle other messages like STATUS or ERROR
                print(f"{ts}  Raw: {line}")

            # tiny sleep to avoid busy loop
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("Stopping")
    finally:
        ser.close()


if __name__ == '__main__':
    main()
