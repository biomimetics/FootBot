"""PC-side command interface for stage control via Arduino serial.

This module provides a small Python class compatible with the Arduino
`SerialComms` protocol. It sends formatted command lines and parses
response lines returned by the Arduino.
"""

import argparse
import time

try:
    import serial
except ImportError as exc:
    raise ImportError(
        "pyserial is required for StageCommandComms. Install with `pip install pyserial`."
    ) from exc


class StageCommandComms:
    RESPONSE_POSITION = "POSITION"
    RESPONSE_STATUS = "STATUS"
    RESPONSE_ERROR = "ERROR"
    RESPONSE_UNKNOWN = "UNKNOWN"

    class Response:
        def __init__(self, response_type, payload, raw):
            self.type = response_type
            self.payload = payload
            self.raw = raw

        def __repr__(self):
            return f"<Response type={self.type} payload={self.payload!r}>"

    def __init__(self, port=None, baudrate=115200, timeout=1.0):
        self._port = None
        self._baudrate = baudrate
        self._timeout = timeout
        self._port_name = ""
        self._serial = None

        if port is not None:
            self.open(port, baudrate=baudrate, timeout=timeout)

    def open(self, port, baudrate=None, timeout=None):
        """Open a serial port for Arduino communication."""
        self.close()
        self._port = port
        self._baudrate = baudrate if baudrate is not None else self._baudrate
        self._timeout = timeout if timeout is not None else self._timeout
        self._serial = serial.Serial(port=self._port, baudrate=self._baudrate, timeout=self._timeout)
        self._port_name = port
        # Arduino boards often reset when a serial port opens. Wait for boot and clear startup data.
        time.sleep(2.0)
        try:
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
        except AttributeError:
            pass

    def close(self):
        """Close the serial connection if it is open."""
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
        self._serial = None

    @property
    def is_open(self):
        return self._serial is not None and self._serial.is_open

    @property
    def port_name(self):
        return self._port_name

    def set_port_name(self, port_name):
        """Store a human-readable port name for the current connection."""
        self._port_name = port_name or ""

    def set_port(self, port):
        """Change the active serial port name without opening it."""
        self._port = port
        self._port_name = port

    def _send_line(self, line):
        if not self.is_open:
            raise RuntimeError("Serial port is not open")
        sanitized = ''.join(ch for ch in line if 0x20 <= ord(ch) <= 0x7E)
        sanitized = sanitized.strip()
        data = (sanitized + "\n").encode("utf-8")
        self._serial.write(data)
        self._serial.flush()

    def move_relative(self, x_mm, y_mm, z_mm):
        """Send a relative move command to the Arduino."""
        self._send_line(f"MOVE_REL,{x_mm},{y_mm},{z_mm}")

    def move_absolute(self, x_mm, y_mm, z_mm):
        """Send an absolute move command to the Arduino."""
        self._send_line(f"MOVE_ABS,{x_mm},{y_mm},{z_mm}")

    def set_speed(self, speed_x, speed_y, speed_z):
        """Send a speed update command for each axis."""
        self._send_line(f"SET_SPEED,{speed_x},{speed_y},{speed_z}")

    def stop(self):
        """Send a stop command to the Arduino."""
        self._send_line("STOP")

    def enable(self):
        """Send an enable command to the Arduino."""
        self._send_line("ENABLE")

    def disable(self):
        """Send a disable command to the Arduino."""
        self._send_line("DISABLE")

    def query_position(self):
        """Request the current stage position from the Arduino."""
        self._send_line("QUERY_POS")

    def send_raw(self, raw_command):
        """Send a raw command string to the Arduino."""
        self._send_line(raw_command)

    def read_response(self, timeout=None):
        """Read a single response line from the Arduino and parse it."""
        if not self.is_open:
            raise RuntimeError("Serial port is not open")

        original_timeout = self._serial.timeout
        if timeout is not None:
            self._serial.timeout = timeout

        try:
            raw = self._serial.readline().decode("utf-8", errors="replace").strip()
        finally:
            if timeout is not None:
                self._serial.timeout = original_timeout

        if raw == "":
            return None

        return self._parse_response(raw)

    def wait_for_response(self, expected_type=None, timeout=None):
        """Wait for a matching response type until the timeout expires."""
        end_time = time.time() + (timeout if timeout is not None else self._timeout)
        while time.time() < end_time:
            response = self.read_response(timeout=min(0.1, max(0.0, end_time - time.time())))
            if response is None:
                continue
            if expected_type is None or response.type == expected_type:
                return response
        return None

    def _parse_response(self, raw_line):
        text = raw_line.strip()
        if text.startswith("<") and text.endswith(">"):
            text = text[1:-1].strip()

        parts = [part.strip() for part in text.split(",") if part.strip() != ""]
        if not parts:
            return self.Response(self.RESPONSE_UNKNOWN, {"raw": raw_line}, raw_line)

        tag = parts[0].upper()
        if tag == self.RESPONSE_POSITION and len(parts) == 4:
            try:
                return self.Response(
                    self.RESPONSE_POSITION,
                    {
                        "x_mm": float(parts[1]),
                        "y_mm": float(parts[2]),
                        "z_mm": float(parts[3]),
                    },
                    raw_line,
                )
            except ValueError:
                return self.Response(self.RESPONSE_ERROR, {"message": raw_line}, raw_line)

        if tag == self.RESPONSE_STATUS:
            return self.Response(self.RESPONSE_STATUS, {"message": ",".join(parts[1:])}, raw_line)

        if tag == self.RESPONSE_ERROR:
            return self.Response(self.RESPONSE_ERROR, {"message": ",".join(parts[1:])}, raw_line)

        return self.Response(self.RESPONSE_UNKNOWN, {"message": raw_line}, raw_line)


def _build_parser():
    parser = argparse.ArgumentParser(
        description="Stage command demo for Arduino SerialComms-compatible firmware."
    )
    parser.add_argument("--port", required=True, help="Serial port name (e.g. COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--move-rel", nargs=3, type=float, metavar=("X", "Y", "Z"), help="Relative move in mm")
    group.add_argument("--move-abs", nargs=3, type=float, metavar=("X", "Y", "Z"), help="Absolute move in mm")
    group.add_argument("--set-speed", nargs=3, type=int, metavar=("X", "Y", "Z"), help="Set speed for X/Y/Z")
    group.add_argument("--stop", action="store_true", help="Send stop command")
    group.add_argument("--enable", action="store_true", help="Send enable command")
    group.add_argument("--disable", action="store_true", help="Send disable command")
    group.add_argument("--query-pos", action="store_true", help="Request current position")
    group.add_argument("--raw", type=str, help="Send a raw command string (comma or space separated)")
    group.add_argument("--interactive", action="store_true", help="Enter interactive command mode")

    parser.add_argument("--timeout", type=float, default=1.0, help="Read timeout in seconds")
    return parser


def _print_response(response):
    if response is None:
        print("No response received.")
        return
    print(f"Raw: {response.raw}")
    print(f"Type: {response.type}")
    print(f"Payload: {response.payload}")


def _interactive_loop(comms):
    print("Entering interactive mode. Type 'help' for commands, 'quit' to exit.")
    while True:
        try:
            line = input("stage> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting interactive mode.")
            break

        if not line:
            continue
        if line.lower() in {"quit", "exit"}:
            break
        if line.lower() in {"help", "?"}:
            print("Commands:")
            print("  move_rel X Y Z")
            print("  move_abs X Y Z")
            print("  set_speed X Y Z")
            print("  stop")
            print("  enable")
            print("  disable")
            print("  query_pos")
            print("  raw <COMMAND>  (comma or space separated)")
            print("  quit")
            continue

        parts = line.split()
        cmd = parts[0].lower()
        args = parts[1:]

        try:
            if cmd == "move_rel" and len(args) == 3:
                comms.move_relative(float(args[0]), float(args[1]), float(args[2]))
            elif cmd == "move_abs" and len(args) == 3:
                comms.move_absolute(float(args[0]), float(args[1]), float(args[2]))
            elif cmd == "set_speed" and len(args) == 3:
                comms.set_speed(int(args[0]), int(args[1]), int(args[2]))
            elif cmd == "stop":
                comms.stop()
            elif cmd == "enable":
                comms.enable()
            elif cmd == "disable":
                comms.disable()
            elif cmd == "query_pos":
                comms.query_position()
            elif cmd == "raw" and args:
                comms.send_raw(" ".join(args))
            else:
                print("Unknown command or wrong arguments. Type 'help' for usage.")
                continue
        except Exception as exc:
            print(f"Error sending command: {exc}")
            continue

        response = comms.wait_for_response(timeout=comms._timeout)
        _print_response(response)


def main():
    parser = _build_parser()
    args = parser.parse_args()

    comms = StageCommandComms(port=args.port, baudrate=args.baud, timeout=args.timeout)

    if args.interactive:
        _interactive_loop(comms)
        comms.close()
        return

    if args.move_rel:
        comms.move_relative(*args.move_rel)
    elif args.move_abs:
        comms.move_absolute(*args.move_abs)
    elif args.set_speed:
        comms.set_speed(*args.set_speed)
    elif args.stop:
        comms.stop()
    elif args.enable:
        comms.enable()
    elif args.disable:
        comms.disable()
    elif args.query_pos:
        comms.query_position()
    elif args.raw:
        comms.send_raw(args.raw)
    else:
        parser.error("No command specified.")

    response = comms.wait_for_response(timeout=args.timeout)
    _print_response(response)
    comms.close()


if __name__ == "__main__":
    main()
