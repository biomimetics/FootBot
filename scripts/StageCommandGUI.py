"""Stage control GUI for Arduino-based X/Y/Z stages.

Provides a Tkinter desktop window that lets you:
  - Connect to the stage Arduino over USB serial
  - Move the X, Y, and Z axes by a configurable step size
  - Set the movement speed of each axis independently
  - Stop stepper motors instantly with a Stop button
  - Connect to a separate force-sensor Arduino and view a live force graph
  - Run a two-point force calibration sequence
  - Connect to a Trill flex touch-sensor Arduino and view a live bar chart
  - Record stage movements and force readings to a timestamped CSV

Hardware setup
--------------
Stage Arduino  → typically /dev/ttyACM0  (Linux) or COM3 (Windows)
Force Arduino  → typically /dev/ttyACM1  (Linux) or COM4 (Windows)
Touch Arduino  → typically /dev/ttyACM2  (Linux) or COM5 (Windows)

How to run
----------
    python StageCommandGUI.py
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

import csv
import os
import threading
import time
import re
from collections import deque

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None  # force-sensor features will be disabled if pyserial is missing
    list_ports = None

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from StageCommandComms import StageCommandComms


# ---------------------------------------------------------------------------
# Default settings — edit these to match your hardware before launching
# ---------------------------------------------------------------------------
DEFAULT_STAGE_PORT    = "/dev/ttyACM0"   # serial port for the stage Arduino
DEFAULT_FORCE_PORT    = "/dev/ttyACM1"   # serial port for the force-sensor Arduino
DEFAULT_BAUD          = 115200           # baud rate used by both Arduinos
DEFAULT_XY_STEP_MM    = 1.0             # default X/Y movement per button press (mm)
DEFAULT_Z_STEP_MM     = 0.5             # default Z movement per button press (mm)
DEFAULT_XY_SPEED_MM_S = 2.0            # default X/Y speed (mm/s)
DEFAULT_Z_SPEED_MM_S  = 0.4            # default Z speed (mm/s)

DEFAULT_TOUCH_PORT    = "/dev/ttyACM2"   # serial port for the Trill touch sensor Arduino
DEFAULT_FORCE_Y_MAX   = 1000            # default upper Y limit on the force graph (grams)
DEFAULT_TOUCH_Y_MIN   = 0               # default lower Y limit on the touch bar chart (raw units)
DEFAULT_TOUCH_Y_MAX   = 400             # default upper Y limit on the touch bar chart (raw units)

# Boards identified by their USB serial number, which never changes for a given
# board across replugs, reboots or machines. This is what makes the port
# dropdowns stable when Linux renumbers /dev/ttyACM* - which it does routinely.
#
# To add a board: plug it in, run
#     python -c "from serial.tools import list_ports; [print(p.device, p.serial_number, p.manufacturer, p.product) for p in list_ports.comports()]"
# and paste its serial number here against the role it plays.
KNOWN_BOARDS = {
    # Confirmed: answers QUERY_POS with a POSITION line.
    "75136323337351C07192": "stage",
    # Second Mega, currently silent on open and unresponsive to GET_CAL /
    # STREAM_ON / QUERY_POS. Role unconfirmed, so deliberately not mapped -
    # it still appears in the dropdowns, just without a role label.
    # "5563930313535111E1C1": "force",
}

# Fallback role guesses by USB vendor ID, used only when the serial number is
# not in KNOWN_BOARDS above.
VENDOR_HINTS = {
    0x2341: "Arduino",       # Arduino SA
    0x2A03: "Arduino Srl",
    0x2E8A: "Raspberry Pi",  # Pico-based touch sensor
}

ROLE_LABELS = {"stage": "Stage", "force": "Force", "touch": "Touch"}


def describe_ports():
    """List candidate serial ports with a human-readable label for each.

    Returns a list of (device, label, role) tuples, role being 'stage',
    'force', 'touch' or None when the board is not recognised.
    """
    if list_ports is None:
        return []

    described = []
    for port in sorted(list_ports.comports(), key=lambda p: p.device):
        # Skip the motherboard's phantom /dev/ttyS* UARTs, which outnumber the
        # real devices roughly 30 to 1 and are never what the user wants.
        if "ACM" not in port.device and "USB" not in port.device:
            continue

        role = KNOWN_BOARDS.get(port.serial_number or "")
        if role:
            detail = ROLE_LABELS[role]
        else:
            detail = port.product or VENDOR_HINTS.get(port.vid or 0) or port.manufacturer or "unknown"
        described.append((port.device, f"{port.device} - {detail}", role))
    return described


def port_from_label(text):
    """Recover a device path from a dropdown label, or from hand-typed text."""
    return (text or "").split(" - ")[0].strip()


FORCE_WINDOW_S     = 10.0    # how many seconds of force history to show in the graph
FORCE_PLOT_MS      = 250     # how often to redraw the force graph (milliseconds)
POSITION_UPDATE_MS = 2000    # how often to poll the stage position (milliseconds)
# Commanded vs measured positions always differ slightly (quadrature resolution,
# backlash). Highlight the measured value only once the gap exceeds this.
ENCODER_DRIFT_WARN_MM = 0.05

TOUCH_PLOT_MS = 100          # how often to redraw the touch bar chart (milliseconds)

# Polling the stage blocks the GUI thread briefly, so it only runs fast while
# recording, where a denser position trace is worth the slight lag.
RECORD_POSITION_UPDATE_MS = 500
RECORD_STATUS_MS          = 500   # how often to flush the file and refresh the row count
RECORD_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
RECORD_FIELDS = [
    "time_s", "unix_time", "event", "force_g",
    "x_mm", "y_mm", "z_mm", "x_enc_mm", "y_enc_mm", "z_enc_mm",
    "move_x_mm", "move_y_mm", "move_z_mm",
]

# ---------------------------------------------------------------------------
# Motor physical constants — from firmware_stepper/include/common_constants.h
# ---------------------------------------------------------------------------
# All three axes use the same motor and microstepping setting.
_STEPS_PER_REV = 200   # full steps per motor revolution
_MICROSTEPS    = 16    # microstepping divisor set on the driver board

# Lead-screw pitch differs between axes.
_XY_MM_PER_REV = 6.35  # X and Y lead screws: 1/4-inch pitch
_Z_MM_PER_REV  = 1.27  # Z lead screw: finer pitch for better resolution

# Steps per mm — derived from the motor and lead-screw constants above.
XY_STEPS_PER_MM = (_STEPS_PER_REV * _MICROSTEPS) / _XY_MM_PER_REV  # ≈ 503.9 steps/mm
Z_STEPS_PER_MM  = (_STEPS_PER_REV * _MICROSTEPS) / _Z_MM_PER_REV   # ≈ 2519.7 steps/mm


def mm_per_s_to_us_per_step(mm_per_s, steps_per_mm):
    """Convert a speed in mm/s to the µs/step value the firmware expects.

    The firmware schedules each step as:
        next_step_time = now + speed_us * 2   (µs)
    so:
        step_freq (steps/s) = 1,000,000 / (speed_us * 2)
        velocity  (mm/s)    = step_freq / steps_per_mm
                            = 500,000  / (speed_us * steps_per_mm)
    Rearranging:
        speed_us = 500,000 / (mm_per_s * steps_per_mm)
    """
    return max(1, round(500_000 / (mm_per_s * steps_per_mm)))


# ---------------------------------------------------------------------------
# Main GUI class
# ---------------------------------------------------------------------------

class StageControlGUI:
    """Main application window for controlling the XYZ stage."""

    def __init__(self, root):
        self.root = root
        self.root.title("Stage Control")

        # comms object is created on connect and set to None when disconnected
        self.comms = None

        # list of axis movement buttons — used to enable/disable them as a group
        self._motion_buttons = []

        # Recording state. Rows arrive from both the force reader thread and the
        # GUI thread, so the writer is guarded by _record_lock.
        self._record_file   = None
        self._record_writer = None
        self._record_lock   = threading.Lock()
        self._record_start  = 0.0
        self._record_rows   = 0
        # Last known positions, carried onto every row so force readings can be
        # matched to where the stage was without a separate join.
        self._last_pos = {"x_mm": None, "y_mm": None, "z_mm": None}
        self._last_enc = {"x_enc_mm": None, "y_enc_mm": None, "z_enc_mm": None}

        self._build_widgets()
        self._update_connection_state(connected=False)
        self.refresh_ports()

    # =========================================================================
    # GUI SETUP
    # =========================================================================

    def _build_widgets(self):
        """Create a two-column layout and call each section builder.

        Left column  — all stage controls (connection, step size, speed, motion, status)
        Right column — sensor plots stacked vertically (force on top, touch on bottom)
        """
        self.main_frame = ttk.Frame(self.root, padding=12)
        self.main_frame.grid(sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        # Left panel: stage controls sit in column 0 (natural/fixed width)
        self.left_panel = ttk.Frame(self.main_frame)
        self.left_panel.grid(row=0, column=0, sticky="ns", padx=(0, 8))

        # Right panel: sensor plots sit in column 1 (expands to fill remaining width)
        self.right_panel = ttk.Frame(self.main_frame)
        self.right_panel.grid(row=0, column=1, sticky="nsew")
        self.right_panel.rowconfigure(0, weight=1)  # force frame grows
        self.right_panel.rowconfigure(1, weight=1)  # touch frame grows

        self.main_frame.columnconfigure(0, weight=0)  # left: don't stretch
        self.main_frame.columnconfigure(1, weight=1)  # right: take all extra space
        self.main_frame.rowconfigure(0, weight=1)

        # Build left-column frames (rows 0-4 inside left_panel)
        self._build_connection_frame()
        self._build_step_frame()
        self._build_speed_frame()
        self._build_control_frame()
        self._build_status_frame()
        self._build_record_frame()

        # Build right-column sensor frames (rows 0-1 inside right_panel)
        self._build_force_frame()
        self._build_touch_frame()

    def _build_connection_frame(self):
        """Left panel row 0 — serial port settings and the Connect/Disconnect button."""
        conn = ttk.LabelFrame(self.left_panel, text="Connection")
        conn.grid(row=0, column=0, sticky="ew", padx=2, pady=2)
        conn.columnconfigure(1, weight=1)

        ttk.Label(conn, text="Port:").grid(row=0, column=0, sticky="w", padx=4, pady=4)
        # Editable combobox: pick a detected board, or type a path by hand.
        self.port_entry = ttk.Combobox(conn)
        self.port_entry.insert(0, DEFAULT_STAGE_PORT)
        self.port_entry.grid(row=0, column=1, sticky="ew", padx=4, pady=4)

        ttk.Label(conn, text="Baud:").grid(row=0, column=2, sticky="w", padx=4, pady=4)
        self.baud_entry = ttk.Entry(conn, width=10)
        self.baud_entry.insert(0, str(DEFAULT_BAUD))
        self.baud_entry.grid(row=0, column=3, sticky="w", padx=4, pady=4)

        self.connect_button = ttk.Button(conn, text="Connect", command=self.connect)
        self.connect_button.grid(row=0, column=4, sticky="e", padx=4, pady=4)

        self.refresh_ports_button = ttk.Button(conn, text="Refresh ports", command=self.refresh_ports)
        self.refresh_ports_button.grid(row=1, column=0, columnspan=5, sticky="ew", padx=4, pady=(0, 4))

    def _build_step_frame(self):
        """Left panel row 1 — entry boxes for X/Y and Z step sizes."""
        step = ttk.LabelFrame(self.left_panel, text="Step sizes (mm)")
        step.grid(row=1, column=0, sticky="ew", padx=2, pady=2)
        step.columnconfigure(1, weight=1)
        step.columnconfigure(3, weight=1)

        ttk.Label(step, text="X/Y step:").grid(row=0, column=0, sticky="w", padx=4, pady=4)
        self.xy_step_entry = ttk.Entry(step)
        self.xy_step_entry.insert(0, str(DEFAULT_XY_STEP_MM))
        self.xy_step_entry.grid(row=0, column=1, sticky="ew", padx=4, pady=4)

        ttk.Label(step, text="Z step:").grid(row=0, column=2, sticky="w", padx=4, pady=4)
        self.z_step_entry = ttk.Entry(step)
        self.z_step_entry.insert(0, str(DEFAULT_Z_STEP_MM))
        self.z_step_entry.grid(row=0, column=3, sticky="ew", padx=4, pady=4)

    def _build_speed_frame(self):
        """Row 2 — per-axis speed entry boxes and a Set Speed button.

        Speeds are entered in mm/s. The conversion to µs/step (required by the
        firmware) is done automatically inside set_speed() using the motor's
        physical constants (steps/mm and the firmware timing formula).

        X and Y share the same lead screw pitch so their steps/mm are identical.
        Z has a finer pitch and therefore a different steps/mm value — the same
        mm/s value will result in a different µs/step number for Z.
        """
        speed = ttk.LabelFrame(self.left_panel, text="Speed (mm/s)")
        speed.grid(row=2, column=0, sticky="ew", padx=2, pady=2)
        speed.columnconfigure(1, weight=1)
        speed.columnconfigure(3, weight=1)
        speed.columnconfigure(5, weight=1)

        ttk.Label(speed, text="X:").grid(row=0, column=0, sticky="w", padx=4, pady=4)
        self.x_speed_entry = ttk.Entry(speed)
        self.x_speed_entry.insert(0, str(DEFAULT_XY_SPEED_MM_S))
        self.x_speed_entry.grid(row=0, column=1, sticky="ew", padx=4, pady=4)

        ttk.Label(speed, text="Y:").grid(row=0, column=2, sticky="w", padx=4, pady=4)
        self.y_speed_entry = ttk.Entry(speed)
        self.y_speed_entry.insert(0, str(DEFAULT_XY_SPEED_MM_S))
        self.y_speed_entry.grid(row=0, column=3, sticky="ew", padx=4, pady=4)

        ttk.Label(speed, text="Z:").grid(row=0, column=4, sticky="w", padx=4, pady=4)
        self.z_speed_entry = ttk.Entry(speed)
        self.z_speed_entry.insert(0, str(DEFAULT_Z_SPEED_MM_S))
        self.z_speed_entry.grid(row=0, column=5, sticky="ew", padx=4, pady=4)

        self.set_speed_button = ttk.Button(speed, text="Set Speed", command=self.set_speed)
        self.set_speed_button.grid(row=0, column=6, sticky="e", padx=4, pady=4)

    def _build_control_frame(self):
        """Left panel row 3 — +/- buttons for each axis plus a Stop button.

        Motors are enabled automatically before each move and disabled
        immediately afterwards, so no manual Enable/Disable buttons are needed.
        """
        control = ttk.LabelFrame(self.left_panel, text="Motion controls")
        control.grid(row=3, column=0, sticky="nsew", padx=2, pady=2)
        control.columnconfigure(0, weight=1)

        axis_frame = ttk.Frame(control)
        axis_frame.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)

        self._add_axis_buttons(axis_frame, "X", row=0)
        self._add_axis_buttons(axis_frame, "Y", row=1)
        self._add_axis_buttons(axis_frame, "Z", row=2)

        self.stop_button = ttk.Button(control, text="Stop", command=self.stop_motion)
        self.stop_button.grid(row=1, column=0, sticky="ew", padx=4, pady=6)

    def _add_axis_buttons(self, parent, axis_name, row):
        """Add a label and +/- pair of buttons for one axis to *parent*."""
        ttk.Label(parent, text=f"{axis_name}:").grid(row=row, column=0, sticky="w", padx=4, pady=4)

        minus_btn = ttk.Button(
            parent, text=f"-{axis_name}",
            command=lambda a=axis_name: self.move_axis(a, negative=True),
        )
        plus_btn = ttk.Button(
            parent, text=f"+{axis_name}",
            command=lambda a=axis_name: self.move_axis(a, negative=False),
        )

        minus_btn.grid(row=row, column=1, sticky="ew", padx=4, pady=4)
        plus_btn.grid( row=row, column=2, sticky="ew", padx=4, pady=4)

        # Keep a flat list of all axis buttons so we can enable/disable them
        # together without searching the widget tree.
        self._motion_buttons.extend([minus_btn, plus_btn])

    def _build_status_frame(self):
        """Left panel row 4 — connection status text and X/Y/Z position labels."""
        status = ttk.Frame(self.left_panel)
        status.grid(row=4, column=0, sticky="ew", padx=2, pady=2)
        status.columnconfigure(1, weight=1)

        ttk.Label(status, text="Status:").grid(row=0, column=0, sticky="w", padx=4, pady=4)
        self.status_label = ttk.Label(status, text="Disconnected", foreground="red")
        self.status_label.grid(row=0, column=1, sticky="w", padx=4, pady=4)

        pos_frame = ttk.LabelFrame(status, text="Current Positions (mm)")
        pos_frame.grid(row=1, column=0, columnspan=2, sticky="ew", padx=4, pady=4)
        pos_frame.columnconfigure(1, weight=1)
        pos_frame.columnconfigure(3, weight=1)
        pos_frame.columnconfigure(5, weight=1)

        ttk.Label(pos_frame, text="X:").grid(row=0, column=0, sticky="w", padx=4, pady=2)
        self.x_pos_label = ttk.Label(pos_frame, text="--", foreground="blue")
        self.x_pos_label.grid(row=0, column=1, sticky="w", padx=4, pady=2)

        ttk.Label(pos_frame, text="Y:").grid(row=0, column=2, sticky="w", padx=4, pady=2)
        self.y_pos_label = ttk.Label(pos_frame, text="--", foreground="blue")
        self.y_pos_label.grid(row=0, column=3, sticky="w", padx=4, pady=2)

        ttk.Label(pos_frame, text="Z:").grid(row=0, column=4, sticky="w", padx=4, pady=2)
        self.z_pos_label = ttk.Label(pos_frame, text="--", foreground="blue")
        self.z_pos_label.grid(row=0, column=5, sticky="w", padx=4, pady=2)

        # Measured encoder positions. The row above is the commanded step count;
        # this one is what the encoders actually read. They diverge when steps
        # are missed, the axis stalls, or the drivers are de-energized.
        ttk.Label(pos_frame, text="Measured:").grid(row=1, column=0, sticky="w", padx=4, pady=2)
        self.enc_labels = {}
        for col, axis in enumerate(("X", "Y", "Z")):
            label = ttk.Label(pos_frame, text="n/a", foreground="gray")
            label.grid(row=1, column=1 + col * 2, sticky="w", padx=4, pady=2)
            self.enc_labels[axis] = label

        zero_btn = ttk.Button(pos_frame, text="Zero encoders", command=self.zero_encoders)
        zero_btn.grid(row=2, column=0, columnspan=6, sticky="w", padx=4, pady=(2, 4))

    def _build_record_frame(self):
        """Left panel row 5 — start/stop recording of stage and force data."""
        record = ttk.LabelFrame(self.left_panel, text="Recording")
        record.grid(row=5, column=0, sticky="ew", padx=2, pady=2)
        record.columnconfigure(1, weight=1)

        self.record_button = ttk.Button(record, text="Start Recording", command=self.toggle_recording)
        self.record_button.grid(row=0, column=0, sticky="w", padx=4, pady=4)

        self.record_label = ttk.Label(record, text="Not recording", foreground="gray")
        self.record_label.grid(row=0, column=1, sticky="w", padx=4, pady=4)

    def _build_force_frame(self):
        """Right panel row 0 — force-sensor connection, live line graph, and calibration."""
        force = ttk.LabelFrame(self.right_panel, text="Force Sensor")
        force.grid(row=0, column=0, sticky="nsew", padx=2, pady=2)
        force.columnconfigure(1, weight=1)  # port entry expands
        force.rowconfigure(1, weight=1)     # graph row expands

        # --- Serial port controls ---
        ttk.Label(force, text="Port:").grid(row=0, column=0, sticky="w", padx=4, pady=4)
        self.force_port_entry = ttk.Combobox(force)
        self.force_port_entry.insert(0, DEFAULT_FORCE_PORT)
        self.force_port_entry.grid(row=0, column=1, sticky="ew", padx=4, pady=4)

        ttk.Label(force, text="Baud:").grid(row=0, column=2, sticky="w", padx=4, pady=4)
        self.force_baud_entry = ttk.Entry(force, width=8)
        self.force_baud_entry.insert(0, str(DEFAULT_BAUD))
        self.force_baud_entry.grid(row=0, column=3, sticky="w", padx=4, pady=4)

        self.force_connect_button = ttk.Button(force, text="Connect Force", command=self.toggle_force)
        self.force_connect_button.grid(row=0, column=4, sticky="e", padx=4, pady=4)

        self.force_calib_button = ttk.Button(force, text="Calibrate", command=self._force_calibrate)
        self.force_calib_button.grid(row=0, column=5, sticky="e", padx=4, pady=4)

        # Y-axis max — user can type a new value; the plot picks it up on the next redraw
        ttk.Label(force, text="Y max (g):").grid(row=0, column=6, sticky="w", padx=(8, 2), pady=4)
        self.force_y_max_entry = ttk.Entry(force, width=6)
        self.force_y_max_entry.insert(0, str(DEFAULT_FORCE_Y_MAX))
        self.force_y_max_entry.grid(row=0, column=7, sticky="w", padx=(0, 4), pady=4)

        # --- Matplotlib line graph ---
        self.force_fig = Figure(figsize=(6, 2.5), dpi=100)
        self.force_ax  = self.force_fig.add_subplot(111)
        self.force_ax.set_title("Force (g)")
        self.force_ax.set_ylabel("Force (g)")
        self.force_ax.set_xlabel("Time (s)")
        self.force_line, = self.force_ax.plot([], [], "-b")

        self.force_canvas = FigureCanvasTkAgg(self.force_fig, master=force)
        self.force_canvas.get_tk_widget().grid(row=1, column=0, columnspan=8, sticky="nsew")

        # --- Internal state for the force-sensor thread ---
        # The reader thread runs in the background and fills _force_data.
        # _force_lock protects _force_data from simultaneous access by the
        # reader thread and the main (GUI) thread.
        # _force_serial_lock gives the calibration commands exclusive use of
        # the serial port so they don't race with the normal reader loop.
        self._force_data       = deque()          # list of (timestamp, grams) tuples
        self._force_lock       = threading.Lock() # guards _force_data
        self._force_serial_lock = threading.Lock() # guards _force_serial during calibration

        self._force_serial  = None   # serial.Serial object when connected
        self._force_thread  = None   # background reader thread
        self._force_running = False  # set to False to stop the reader thread

        # Regex that matches the Arduino's <FORCE,123.45> output lines
        self._force_regex = re.compile(r"<FORCE,\s*([+-]?\d*\.?\d+)>")

        # Start the periodic graph redraw (runs even when no sensor is connected)
        self._schedule_force_plot()

    # =========================================================================
    # CONNECTION MANAGEMENT
    # =========================================================================

    def refresh_ports(self):
        """Rescan USB serial devices and repopulate all three port dropdowns.

        Each dropdown preselects the board matching its role, but only while
        that field still holds a value the user has not edited - a hand-typed
        or already-correct path is never overwritten.
        """
        described = describe_ports()
        labels = [label for _, label, _ in described]
        by_role = {role: label for _, label, role in described if role}

        widgets = (
            (self.port_entry, "stage"),
            (self.force_port_entry, "force"),
            (self.touch_port_entry, "touch"),
        )
        devices = {dev: dev_role for dev, _, dev_role in described}
        for widget, role in widgets:
            widget["values"] = labels
            current = port_from_label(widget.get())
            preferred = by_role.get(role)

            # A port identified as another role's board is always wrong here -
            # the stale /dev/ttyACM* defaults routinely point at whichever
            # board happened to enumerate first. Never leave one in place.
            claimed_by_other = devices.get(current) not in (None, role)

            if preferred and (claimed_by_other or current not in devices):
                widget.set(preferred)
            elif claimed_by_other:
                # No known board for this role, but the current one definitely
                # belongs to a different role. Clear it rather than guess.
                widget.set("")

        if not described:
            self._set_status("No USB serial devices found", color="red")
        else:
            unknown = sum(1 for _, _, role in described if role is None)
            message = f"Found {len(described)} port(s)"
            if unknown:
                message += f", {unknown} unidentified"
            self._set_status(message)

    def _set_status(self, message, color="black"):
        """Update the status label text and colour."""
        self.status_label.config(text=message, foreground=color)

    def _update_connection_state(self, connected):
        """Enable or disable controls that require an active stage connection."""
        state = "normal" if connected else "disabled"

        # Enable/disable the Stop and Set Speed buttons
        self.stop_button.config(state=state)
        self.set_speed_button.config(state=state)

        # Enable/disable all six axis movement buttons
        for btn in self._motion_buttons:
            btn.config(state=state)

        # Swap Connect ↔ Disconnect label and lock the port fields while connected
        self.connect_button.config(text="Disconnect" if connected else "Connect")
        self.port_entry.config(state="disabled" if connected else "normal")
        self.baud_entry.config(state="disabled" if connected else "normal")

    def connect(self):
        """Open the serial connection to the stage Arduino.

        If already connected, this disconnects instead (the button toggles).
        """
        if self.comms is not None and self.comms.is_open:
            self.disconnect()
            return

        port = port_from_label(self.port_entry.get())
        if not port:
            messagebox.showwarning("Missing port", "Please enter a serial port to connect.")
            return

        try:
            baud = int(self.baud_entry.get().strip())
        except ValueError:
            messagebox.showwarning("Invalid baud", "Baud rate must be a whole number.")
            return

        try:
            self.comms = StageCommandComms(port=port, baudrate=baud, timeout=1.0)
            self._update_connection_state(connected=True)
            self._set_status(f"Connected to {port}", color="green")
            # Give the Arduino 500 ms to settle after the connection reset, then
            # fetch and display the current position.
            self.root.after(500, self._update_positions)
            self._schedule_position_update()
        except Exception as exc:
            self.comms = None
            self._set_status(f"Connect failed: {exc}", color="red")
            messagebox.showerror("Connection failed", str(exc))

    def disconnect(self):
        """Close the serial connection to the stage Arduino."""
        if self.comms is None:
            return
        try:
            self.comms.close()
        except Exception:
            pass
        self.comms = None
        self._update_connection_state(connected=False)
        self._set_status("Disconnected", color="red")
        self._update_positions()   # resets the position labels to "--"

        # Stop the force sensor connection too, if it is running
        if self._force_running:
            try:
                self._stop_force()
            except Exception:
                pass

    # =========================================================================
    # STAGE MOTION
    # =========================================================================

    def move_axis(self, axis, negative=False):
        """Send a relative move command for one axis.

        Parameters
        ----------
        axis:     "X", "Y", or "Z"
        negative: True to move in the negative direction
        """
        if self.comms is None or not self.comms.is_open:
            messagebox.showwarning("Not connected", "Please connect to the Arduino before moving.")
            return

        # Read the step size from the appropriate entry box
        raw = self.xy_step_entry.get().strip() if axis in {"X", "Y"} else self.z_step_entry.get().strip()
        try:
            step = float(raw)
        except ValueError:
            messagebox.showwarning("Invalid step size", f"'{raw}' is not a valid number for the {axis} step.")
            return

        if negative:
            step = -step

        x = step if axis == "X" else 0.0
        y = step if axis == "Y" else 0.0
        z = step if axis == "Z" else 0.0

        try:
            self.comms.enable()
            self.comms.move_relative(x, y, z)
            self.comms.disable()
            self._record_event("move", move_x_mm=x, move_y_mm=y, move_z_mm=z)
            self._set_status(f"Moved  X={x:.3f}  Y={y:.3f}  Z={z:.3f}")
            # Refresh positions shortly after — small delay lets the Arduino complete the move
            self.root.after(100, self._update_positions)
        except Exception as exc:
            self._set_status(f"Move failed: {exc}", color="red")
            messagebox.showerror("Move failed", str(exc))

    def stop_motion(self):
        """Send the STOP command to immediately halt all movement."""
        if self.comms is None or not self.comms.is_open:
            messagebox.showwarning("Not connected", "Please connect before sending commands.")
            return
        try:
            self.comms.stop()
            self._record_event("stop")
            self._set_status("Stop command sent")
        except Exception as exc:
            self._set_status(f"Stop failed: {exc}", color="red")
            messagebox.showerror("Stop failed", str(exc))

    def set_speed(self):
        """Read the X/Y/Z speed entries (mm/s), convert to µs/step, and send to Arduino.

        The conversion uses each axis's steps/mm derived from the motor's physical
        constants (lead-screw pitch, steps/rev, microstepping) and the firmware's
        timing formula:

            speed_us = 500,000 / (mm_per_s × steps_per_mm)

        X and Y use XY_STEPS_PER_MM; Z uses Z_STEPS_PER_MM.
        """
        if self.comms is None or not self.comms.is_open:
            messagebox.showwarning("Not connected", "Please connect to the Arduino before setting speed.")
            return

        # Map each axis to its entry widget and the correct steps/mm constant.
        axis_info = [
            ("X", self.x_speed_entry, XY_STEPS_PER_MM),
            ("Y", self.y_speed_entry, XY_STEPS_PER_MM),
            ("Z", self.z_speed_entry, Z_STEPS_PER_MM),
        ]

        errors = []
        mm_per_s_values = {}
        for axis, entry, _ in axis_info:
            raw = entry.get().strip()
            try:
                value = float(raw)
                if value <= 0:
                    raise ValueError("must be greater than zero")
                mm_per_s_values[axis] = value
            except ValueError:
                errors.append(f"{axis}: '{raw}' is not a valid positive number")

        if errors:
            messagebox.showwarning("Invalid speed", "\n".join(errors))
            return

        # Convert mm/s → µs/step for each axis
        us_per_step = {
            axis: mm_per_s_to_us_per_step(mm_per_s_values[axis], steps_per_mm)
            for axis, _, steps_per_mm in axis_info
        }

        try:
            self.comms.set_speed(us_per_step["X"], us_per_step["Y"], us_per_step["Z"])
            self._set_status(
                f"Speed set  "
                f"X={mm_per_s_values['X']:.2f} mm/s ({us_per_step['X']} µs/step)  "
                f"Y={mm_per_s_values['Y']:.2f} mm/s ({us_per_step['Y']} µs/step)  "
                f"Z={mm_per_s_values['Z']:.2f} mm/s ({us_per_step['Z']} µs/step)"
            )
        except Exception as exc:
            self._set_status(f"Set speed failed: {exc}", color="red")
            messagebox.showerror("Set speed failed", str(exc))

    def _update_positions(self):
        """Ask the stage for its current position and update the X/Y/Z labels."""
        if self.comms is None or not self.comms.is_open:
            self.x_pos_label.config(text="--")
            self.y_pos_label.config(text="--")
            self.z_pos_label.config(text="--")
            return

        try:
            self.comms.query_position()
            response = self.comms.wait_for_response(
                expected_type=self.comms.RESPONSE_POSITION, timeout=1.0
            )
            if response and response.type == self.comms.RESPONSE_POSITION:
                x = response.payload.get("x_mm", 0.0)
                y = response.payload.get("y_mm", 0.0)
                z = response.payload.get("z_mm", 0.0)
                self.x_pos_label.config(text=f"{x:.3f}")
                self.y_pos_label.config(text=f"{y:.3f}")
                self.z_pos_label.config(text=f"{z:.3f}")
                self._last_pos = {"x_mm": x, "y_mm": y, "z_mm": z}
            else:
                self.x_pos_label.config(text="??")
                self.y_pos_label.config(text="??")
                self.z_pos_label.config(text="??")
        except Exception as exc:
            self.x_pos_label.config(text="ERR")
            self.y_pos_label.config(text="ERR")
            self.z_pos_label.config(text="ERR")
            print(f"Position query failed: {exc}")

        self._update_encoders()
        self._record_event("position")

    def _update_encoders(self):
        """Refresh the measured-position labels from any fitted encoders.

        Axes without an encoder, and firmware predating encoder support, both
        leave the labels showing "n/a" rather than reporting an error.
        """
        if self.comms is None or not self.comms.is_open:
            for label in self.enc_labels.values():
                label.config(text="n/a", foreground="gray")
            return

        try:
            self.comms.query_encoder()
            # One ENCODER line arrives per fitted axis, so keep reading until
            # the board stops talking.
            seen = set()
            while True:
                response = self.comms.read_response(timeout=0.2)
                if response is None:
                    break
                if response.type != self.comms.RESPONSE_ENCODER:
                    break
                axis = response.payload.get("axis")
                if axis not in self.enc_labels:
                    continue
                seen.add(axis)
                measured = response.payload.get("measured_mm", 0.0)
                error = response.payload.get("error_mm", 0.0)
                # Flag a drift larger than the warning threshold so a missed
                # step is visible without comparing the two rows by eye.
                color = "red" if abs(error) > ENCODER_DRIFT_WARN_MM else "green"
                self.enc_labels[axis].config(text=f"{measured:.3f}", foreground=color)
                self._last_enc[f"{axis.lower()}_enc_mm"] = measured

            for axis, label in self.enc_labels.items():
                if axis not in seen:
                    label.config(text="n/a", foreground="gray")
        except Exception as exc:
            for label in self.enc_labels.values():
                label.config(text="ERR", foreground="red")
            print(f"Encoder query failed: {exc}")

    def zero_encoders(self):
        """Zero every fitted encoder at the current physical location."""
        if self.comms is None or not self.comms.is_open:
            messagebox.showwarning("Not connected", "Please connect before zeroing encoders.")
            return
        try:
            self.comms.zero_encoder()
            self.comms.wait_for_response(timeout=1.0)
            self._record_event("zero_encoders")
            self._set_status("Encoders zeroed")
            self._update_encoders()
        except Exception as exc:
            self._set_status(f"Zero encoders failed: {exc}", color="red")

    def _schedule_position_update(self):
        """Re-schedule position polling as long as the stage is connected."""
        if self.comms is not None and self.comms.is_open:
            self._update_positions()
            interval = RECORD_POSITION_UPDATE_MS if self._record_writer else POSITION_UPDATE_MS
            self.root.after(interval, self._schedule_position_update)

    # =========================================================================
    # FORCE SENSOR
    # =========================================================================

    def toggle_force(self):
        """Connect to or disconnect from the force-sensor Arduino."""
        if self._force_running:
            self._stop_force()
        else:
            self._start_force()

    def _start_force(self):
        """Open the force-sensor serial port and start the background reader thread."""
        if serial is None:
            messagebox.showerror("Missing dependency", "pyserial is required for force sensor support.")
            return

        port = port_from_label(self.force_port_entry.get())
        try:
            baud = int(self.force_baud_entry.get().strip())
        except ValueError:
            messagebox.showwarning("Invalid baud", "Baud rate must be a whole number.")
            return

        try:
            self._force_serial = serial.Serial(port, baud, timeout=1.0)
        except Exception as exc:
            messagebox.showerror("Open failed", str(exc))
            return

        self._force_running = True
        self.force_connect_button.config(text="Disconnect Force")
        self.force_calib_button.config(state="normal")

        # Start the reader in a daemon thread so it exits automatically when
        # the main window is closed.
        self._force_thread = threading.Thread(target=self._force_reader_loop, daemon=True)
        self._force_thread.start()

    def _stop_force(self):
        """Stop the background reader thread and close the force serial port."""
        self._force_running = False
        if self._force_thread is not None:
            self._force_thread.join(timeout=1.0)
        if self._force_serial is not None:
            try:
                self._force_serial.close()
            except Exception:
                pass
        self._force_serial = None
        self._force_thread = None
        self.force_connect_button.config(text="Connect Force")
        self.force_calib_button.config(state="disabled")

    def _force_reader_loop(self):
        """Background thread: continuously read force values from the serial port.

        Each line from the Arduino is expected to look like:  <FORCE,123.45>
        Parsed values are stored in self._force_data as (timestamp, grams) tuples.
        Old data outside the rolling window is discarded automatically.
        """
        while self._force_running and self._force_serial is not None:
            # Try to get the serial lock briefly.
            # If the calibration code is currently holding the lock we skip this
            # read cycle rather than blocking — this keeps calibration commands
            # from racing with streaming data reads.
            got_lock = self._force_serial_lock.acquire(timeout=0.1)
            if not got_lock:
                time.sleep(0.01)
                continue

            try:
                try:
                    line = self._force_serial.readline().decode("utf-8", errors="ignore").strip()
                except Exception:
                    line = ""

                if not line:
                    continue

                match = self._force_regex.search(line)
                if match:
                    try:
                        force_value = float(match.group(1))
                    except ValueError:
                        continue

                    now = time.time()
                    with self._force_lock:
                        self._force_data.append((now, force_value))
                        # Remove readings older than the display window
                        cutoff = now - FORCE_WINDOW_S
                        while self._force_data and self._force_data[0][0] < cutoff:
                            self._force_data.popleft()
                    self._record_event("force", force_g=force_value, now=now)
            finally:
                self._force_serial_lock.release()

    def _schedule_force_plot(self):
        """Redraw the live force graph, then schedule the next redraw.

        This runs on the main (GUI) thread via Tkinter's after() mechanism.
        It is safe to update Tkinter widgets from here.
        """
        # Grab a snapshot of the current data under the lock so the reader
        # thread can keep writing without waiting for us to finish drawing.
        with self._force_lock:
            if self._force_data:
                now = time.time()
                # Convert absolute timestamps to seconds-ago (negative = past)
                times  = [t - now for (t, _) in self._force_data]
                values = [v       for (_, v) in self._force_data]
            else:
                times, values = [], []

        if times:
            self.force_line.set_xdata(times)
            self.force_line.set_ydata(values)
            self.force_ax.set_xlim(-FORCE_WINDOW_S, 0)
            self.force_ax.set_title(f"Current Force: {values[-1]:.1f} g")
        else:
            self.force_line.set_xdata([])
            self.force_line.set_ydata([])
            self.force_ax.set_xlim(-FORCE_WINDOW_S, 0)

        try:
            y_max = float(self.force_y_max_entry.get().strip())
        except ValueError:
            y_max = DEFAULT_FORCE_Y_MAX
        self.force_ax.set_ylim(0, y_max)

        try:
            self.force_canvas.draw_idle()
        except Exception:
            pass

        # Schedule the next update
        self.root.after(FORCE_PLOT_MS, self._schedule_force_plot)

    # =========================================================================
    # FORCE SENSOR CALIBRATION
    # =========================================================================

    def _send_force_command(self, cmd, wait_prefix=None, timeout=5.0):
        """Send a text command to the force sensor and optionally wait for a reply.

        Parameters
        ----------
        cmd:         Command string to send (e.g. "CALIBRATE")
        wait_prefix: If given, block until a response starting with this prefix
                     arrives (e.g. "STATUS,ZERO_DONE"). Returns None on timeout.
        timeout:     Maximum seconds to wait for the response.

        Returns the raw response string, or None if not waiting / timed out.
        """
        if self._force_serial is None:
            return None

        try:
            self._force_serial.write((cmd + "\n").encode("utf-8"))
            self._force_serial.flush()
        except Exception:
            return None

        if wait_prefix is None:
            return None

        # Hold the serial lock so the reader thread does not consume the response
        # we are waiting for.
        acquired = self._force_serial_lock.acquire(timeout=timeout)
        if not acquired:
            return None

        try:
            deadline = time.time() + timeout
            while time.time() < deadline:
                try:
                    resp = self._force_serial.readline().decode("utf-8", errors="ignore").strip()
                except Exception:
                    resp = ""
                if resp.startswith(f"<{wait_prefix}"):
                    return resp
            return None
        finally:
            self._force_serial_lock.release()

    def _force_calibrate(self):
        """Run a two-point calibration sequence for the force sensor.

        Steps
        -----
        1. Tell the sensor to start calibration.
        2. Ask the user to remove all load, then confirm zero.
        3. Ask the user to place a known weight on the sensor.
        4. Send the known weight to the sensor so it can compute the scale factor.
        5. Optionally save the calibration to the sensor's EEPROM.
        """
        if self._force_serial is None:
            messagebox.showwarning("Not connected", "Connect to the force sensor first.")
            return

        # Step 1: Start calibration on the sensor
        self._send_force_command("CALIBRATE", wait_prefix="STATUS", timeout=2.0)

        # Step 2: Zero the sensor with no load
        if not messagebox.askokcancel(
            "Calibration",
            "Remove all load from the sensor, then press OK to record the zero point.",
        ):
            return

        zero_resp = self._send_force_command(
            "CONFIRM_ZERO", wait_prefix="STATUS,ZERO_DONE", timeout=10.0
        )
        if zero_resp is None:
            messagebox.showerror("Calibration", "Zeroing failed or timed out.")
            return

        # Step 3: Open a dialog so the user can place a weight and enter its value.
        # We use a non-blocking (non-modal) window so the user can still move the
        # stage while the dialog is open.
        dlg = tk.Toplevel(self.root)
        dlg.title("Calibration — Apply Weight")
        dlg.geometry("360x120")
        dlg.transient(self.root)
        dlg.grab_release()   # allow the main window to stay interactive

        ttk.Label(
            dlg,
            text=(
                "Zero recorded. Place a known weight on the sensor and position the stage "
                "as needed. Enter the weight in grams below, then click Apply Weight."
            ),
            wraplength=340,
        ).pack(padx=8, pady=6)

        entry_frame = ttk.Frame(dlg)
        entry_frame.pack(fill="x", padx=8, pady=4)
        ttk.Label(entry_frame, text="Weight (g):").grid(row=0, column=0, sticky="w")
        weight_entry = ttk.Entry(entry_frame)
        weight_entry.grid(row=0, column=1, sticky="ew", padx=6)
        entry_frame.columnconfigure(1, weight=1)

        def on_apply():
            """Called when the user clicks Apply Weight."""
            try:
                weight_g = float(weight_entry.get().strip())
            except ValueError:
                messagebox.showerror("Calibration", "Please enter a valid number for the weight.")
                return

            dlg.destroy()

            # Step 4: Send the known weight so the sensor can compute its scale factor
            cal_resp = self._send_force_command(
                f"SET_WEIGHT,{weight_g}",
                wait_prefix="STATUS,CAL_COMPLETE",
                timeout=10.0,
            )
            if cal_resp is None:
                messagebox.showerror("Calibration", "Calibration step failed or timed out.")
                return

            # Parse the offset and scale factor out of the response
            match = re.search(r"<STATUS,CAL_COMPLETE,\s*([+-]?\d+),\s*([0-9.+\-eE]+)>", cal_resp)
            offset    = None
            scale_val = None
            if match:
                try:
                    offset    = int(match.group(1))
                    scale_val = float(match.group(2))
                except ValueError:
                    pass

            if scale_val is not None:
                summary = f"Calibration complete.\nOffset = {offset},  Scale = {scale_val:.6f}"
            else:
                summary = "Calibration complete."

            # Step 5: Offer to save to EEPROM
            if messagebox.askyesno("Calibration", summary + "\n\nSave calibration to device EEPROM?"):
                self._send_force_command("SAVE_CAL", wait_prefix="STATUS,CAL_SAVED", timeout=3.0)
                messagebox.showinfo("Calibration", "Calibration saved to EEPROM.")
            else:
                messagebox.showinfo("Calibration", "Calibration not saved.")

        def on_cancel():
            """Called when the user clicks Cancel."""
            dlg.destroy()
            messagebox.showinfo("Calibration", "Calibration cancelled.")

        btn_frame = ttk.Frame(dlg)
        btn_frame.pack(fill="x", padx=8, pady=6)
        ttk.Button(btn_frame, text="Apply Weight", command=on_apply).pack(side="right", padx=6)
        ttk.Button(btn_frame, text="Cancel",       command=on_cancel).pack(side="right")

    # =========================================================================
    # RECORDING
    # =========================================================================

    def toggle_recording(self):
        """Start or stop recording stage and force data to CSV."""
        if self._record_writer is not None:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self):
        """Ask for an output file and start streaming rows into it.

        Rows are written as they arrive rather than buffered, so a crash or
        unplugged board mid-run still leaves everything up to that point on disk.
        """
        os.makedirs(RECORD_DIR, exist_ok=True)
        default_name = time.strftime("recording_%Y%m%d_%H%M%S.csv")
        path = filedialog.asksaveasfilename(
            title="Save recording as",
            initialdir=RECORD_DIR,
            initialfile=default_name,
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*")],
        )
        if not path:
            return

        try:
            f = open(path, "w", newline="")
        except Exception as exc:
            messagebox.showerror("Recording failed", str(exc))
            return

        writer = csv.DictWriter(f, fieldnames=RECORD_FIELDS)
        writer.writeheader()
        with self._record_lock:
            self._record_file   = f
            self._record_start  = time.time()
            self._record_rows   = 0
            self._record_writer = writer

        self._record_path = path
        self.record_button.config(text="Stop Recording")
        self._record_event("start")
        # Grab a starting position straight away rather than waiting for the poll.
        self._update_positions()
        self._schedule_record_status()

    def stop_recording(self):
        """Close the recording file."""
        if self._record_writer is None:
            return
        self._record_event("end")
        with self._record_lock:
            f = self._record_file
            rows = self._record_rows
            self._record_writer = None
            self._record_file   = None
        try:
            f.close()
        except Exception:
            pass
        self.record_button.config(text="Start Recording")
        self.record_label.config(
            text=f"Saved {rows} rows to {os.path.basename(self._record_path)}",
            foreground="black",
        )
        self._set_status(f"Recording saved to {self._record_path}")

    def _record_event(self, event, force_g=None, now=None, **move):
        """Append one row to the recording, if one is running.

        Safe to call from any thread. Every row carries the most recent
        commanded and encoder positions alongside the event itself.
        """
        if self._record_writer is None:
            return
        if now is None:
            now = time.time()
        with self._record_lock:
            if self._record_writer is None:  # stopped while we waited for the lock
                return
            row = {
                "time_s":    f"{now - self._record_start:.4f}",
                "unix_time": f"{now:.4f}",
                "event":     event,
                "force_g":   force_g,
                **self._last_pos,
                **self._last_enc,
                **move,
            }
            self._record_writer.writerow(row)
            self._record_rows += 1

    def _schedule_record_status(self):
        """Flush the file and refresh the row counter while recording."""
        if self._record_writer is None:
            return
        with self._record_lock:
            rows = self._record_rows
            try:
                self._record_file.flush()
            except Exception:
                pass
        elapsed = time.time() - self._record_start
        self.record_label.config(
            text=f"Recording  {elapsed:.0f} s, {rows} rows", foreground="red"
        )
        self.root.after(RECORD_STATUS_MS, self._schedule_record_status)

    def on_close(self):
        """Close the recording cleanly before the window goes away."""
        self.stop_recording()
        self.root.destroy()

    # =========================================================================
    # TOUCH SENSOR
    # =========================================================================

    def _build_touch_frame(self):
        """Right panel row 1 — touch sensor connection and live capacitance bar chart.

        Supports two sensor types, selectable via radio buttons:

        Trill Flex (default)
            Each line is space-separated zero-padded integers, one per channel.
            Example: 0234 0412 0089 0731 ...
            All channels are plotted.

        Raspberry Pi Pico
            Each line is comma-separated floats.
            Example: 0.0, 1.23, 4.56, 7.89, 0.12, ...
            Only the 2nd–5th values (indices 1–4) are plotted as 4 bars.
        """
        touch = ttk.LabelFrame(self.right_panel, text="Touch Sensor")
        touch.grid(row=1, column=0, sticky="nsew", padx=2, pady=2)
        touch.columnconfigure(1, weight=1)  # port entry expands
        touch.rowconfigure(2, weight=1)     # graph row expands

        # --- Row 0: serial port controls ---
        ttk.Label(touch, text="Port:").grid(row=0, column=0, sticky="w", padx=4, pady=4)
        self.touch_port_entry = ttk.Combobox(touch)
        self.touch_port_entry.insert(0, DEFAULT_TOUCH_PORT)
        self.touch_port_entry.grid(row=0, column=1, sticky="ew", padx=4, pady=4)

        ttk.Label(touch, text="Baud:").grid(row=0, column=2, sticky="w", padx=4, pady=4)
        self.touch_baud_entry = ttk.Entry(touch, width=8)
        self.touch_baud_entry.insert(0, str(DEFAULT_BAUD))
        self.touch_baud_entry.grid(row=0, column=3, sticky="w", padx=4, pady=4)

        self.touch_connect_button = ttk.Button(
            touch, text="Connect Touch", command=self.toggle_touch
        )
        self.touch_connect_button.grid(row=0, column=4, sticky="e", padx=4, pady=4)

        # Y-axis min/max — user can type new values; the plot picks them up on the next redraw
        ttk.Label(touch, text="Y min:").grid(row=0, column=5, sticky="w", padx=(8, 2), pady=4)
        self.touch_y_min_entry = ttk.Entry(touch, width=6)
        self.touch_y_min_entry.insert(0, str(DEFAULT_TOUCH_Y_MIN))
        self.touch_y_min_entry.grid(row=0, column=6, sticky="w", padx=(0, 4), pady=4)

        ttk.Label(touch, text="Y max:").grid(row=0, column=7, sticky="w", padx=(4, 2), pady=4)
        self.touch_y_max_entry = ttk.Entry(touch, width=6)
        self.touch_y_max_entry.insert(0, str(DEFAULT_TOUCH_Y_MAX))
        self.touch_y_max_entry.grid(row=0, column=8, sticky="w", padx=(0, 4), pady=4)

        # --- Row 1: sensor type selector ---
        # This controls how incoming serial lines are parsed.
        # Changing the type while disconnected takes effect on the next connection.
        self._touch_sensor_type = tk.StringVar(value="trill")

        type_frame = ttk.Frame(touch)
        type_frame.grid(row=1, column=0, columnspan=7, sticky="w", padx=4, pady=(0, 2))

        ttk.Label(type_frame, text="Sensor type:").pack(side="left", padx=(0, 8))

        self.touch_radio_trill = ttk.Radiobutton(
            type_frame, text="Trill Flex",
            variable=self._touch_sensor_type, value="trill",
        )
        self.touch_radio_trill.pack(side="left", padx=4)

        self.touch_radio_pico = ttk.Radiobutton(
            type_frame, text="Raspberry Pi Pico",
            variable=self._touch_sensor_type, value="pico",
        )
        self.touch_radio_pico.pack(side="left", padx=4)

        # Keep a list so we can lock/unlock both buttons together
        self._touch_type_radios = [self.touch_radio_trill, self.touch_radio_pico]

        # --- Row 2: Matplotlib bar chart ---
        self.touch_fig = Figure(figsize=(6, 2.5), dpi=100)
        self.touch_ax  = self.touch_fig.add_subplot(111)
        self.touch_ax.set_title("Touch Sensor — no data yet")
        self.touch_ax.set_xlabel("Channel")
        self.touch_ax.set_ylabel("Raw value")

        self.touch_canvas = FigureCanvasTkAgg(self.touch_fig, master=touch)
        self.touch_canvas.get_tk_widget().grid(
            row=2, column=0, columnspan=9, sticky="nsew"
        )

        # --- Internal state ---
        self._touch_lock    = threading.Lock()
        self._touch_values  = []    # most recent parsed values as a list of numbers
        self._touch_serial  = None  # serial.Serial object when connected
        self._touch_thread  = None  # background reader thread
        self._touch_running = False

        # Start the periodic chart redraw (runs even before a sensor is connected)
        self._schedule_touch_plot()

    def toggle_touch(self):
        """Connect to or disconnect from the Trill touch sensor Arduino."""
        if self._touch_running:
            self._stop_touch()
        else:
            self._start_touch()

    def _start_touch(self):
        """Open the touch sensor serial port and start the background reader thread."""
        if serial is None:
            messagebox.showerror("Missing dependency", "pyserial is required for touch sensor support.")
            return

        port = port_from_label(self.touch_port_entry.get())
        try:
            baud = int(self.touch_baud_entry.get().strip())
        except ValueError:
            messagebox.showwarning("Invalid baud", "Baud rate must be a whole number.")
            return

        try:
            self._touch_serial = serial.Serial(port, baud, timeout=1.0)
        except Exception as exc:
            messagebox.showerror("Open failed", str(exc))
            return

        self._touch_running = True
        self.touch_connect_button.config(text="Disconnect Touch")
        self.touch_port_entry.config(state="disabled")
        self.touch_baud_entry.config(state="disabled")
        for radio in self._touch_type_radios:
            radio.config(state="disabled")

        self._touch_thread = threading.Thread(target=self._touch_reader_loop, daemon=True)
        self._touch_thread.start()

    def _stop_touch(self):
        """Stop the background reader thread and close the touch sensor serial port."""
        self._touch_running = False
        if self._touch_thread is not None:
            self._touch_thread.join(timeout=1.0)
        if self._touch_serial is not None:
            try:
                self._touch_serial.close()
            except Exception:
                pass
        self._touch_serial  = None
        self._touch_thread  = None
        self.touch_connect_button.config(text="Connect Touch")
        self.touch_port_entry.config(state="normal")
        self.touch_baud_entry.config(state="normal")
        for radio in self._touch_type_radios:
            radio.config(state="normal")

    def _touch_reader_loop(self):
        """Background thread: read and parse lines from the touch sensor serial port.

        Parsing depends on the sensor type selected at connection time:

        Trill Flex
            Lines are space-separated zero-padded integers.
            Example:  0234 0412 0089 0731 ...
            All values are stored.

        Raspberry Pi Pico
            Lines are comma-separated floats.
            Example:  0.0, 1.23, 4.56, 7.89, 0.12, ...
            Only the 2nd–5th values (indices 1–4) are stored as 4 floats.
            Lines with fewer than 5 fields are skipped.
        """
        # Snapshot the selected type once at thread start so it stays consistent
        # for the duration of this connection.
        sensor_type = self._touch_sensor_type.get()

        while self._touch_running and self._touch_serial is not None:
            try:
                line = self._touch_serial.readline().decode("utf-8", errors="ignore").strip()
            except Exception:
                continue

            if not line:
                continue

            if sensor_type == "trill":
                # Space-separated zero-padded integers, one per channel
                try:
                    values = [int(token) for token in line.split()]
                except ValueError:
                    continue

            else:  # "pico"
                # Comma-separated fields; each field may have trailing whitespace-
                # separated tokens (e.g. "36.50  a98") — take only the first token
                # of each field so those extras don't break float parsing.
                try:
                    fields = [float(f.strip().split()[0]) for f in line.split(",")]
                except (ValueError, IndexError):
                    continue
                if len(fields) < 5:
                    continue  # not enough fields — skip this line
                values = [-f for f in fields[1:5]]  # flip sign so touch is positive.

            if values:
                with self._touch_lock:
                    self._touch_values = values

    def _schedule_touch_plot(self):
        """Redraw the touch bar chart with the latest channel values, then reschedule.

        This runs on the main (GUI) thread so it is safe to update Tkinter widgets.
        """
        with self._touch_lock:
            values = list(self._touch_values)  # snapshot so we release the lock quickly

        self.touch_ax.cla()  # clear previous bars

        try:
            y_min = float(self.touch_y_min_entry.get().strip())
        except ValueError:
            y_min = DEFAULT_TOUCH_Y_MIN
        try:
            y_max = float(self.touch_y_max_entry.get().strip())
        except ValueError:
            y_max = DEFAULT_TOUCH_Y_MAX

        sensor_label = "Pico" if self._touch_sensor_type.get() == "pico" else "Trill Flex"

        if values:
            n = len(values)
            self.touch_ax.bar(range(n), values, color="steelblue")
            self.touch_ax.set_title(f"{sensor_label} — {n} channels,  latest reading")
            self.touch_ax.set_xlim(-0.5, n - 0.5)
        else:
            self.touch_ax.set_title(f"{sensor_label} — no data yet")

        self.touch_ax.set_xlabel("Channel")
        self.touch_ax.set_ylabel("Raw value")
        self.touch_ax.set_ylim(y_min, y_max)
        

        try:
            self.touch_canvas.draw_idle()
        except Exception:
            pass

        self.root.after(TOUCH_PLOT_MS, self._schedule_touch_plot)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    root = tk.Tk()
    app = StageControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
