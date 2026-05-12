"""Basic stage control GUI for Arduino-based X/Y/Z stages.

This module provides a simple Tkinter interface that connects to the
Arduino via StageCommandComms, enables the axes, and sends relative
move commands using user-defined X/Y and Z step sizes.
"""

import tkinter as tk
from tkinter import ttk, messagebox

from StageCommandComms import StageCommandComms


class StageControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Stage Control")
        self.comms = None

        self._build_widgets()
        self._update_connection_state(False)

    def _build_widgets(self):
        self.main_frame = ttk.Frame(self.root, padding=12)
        self.main_frame.grid(sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        self._build_connection_frame()
        self._build_step_frame()
        self._build_control_frame()
        self._build_status_frame()

    def _build_connection_frame(self):
        conn = ttk.LabelFrame(self.main_frame, text="Connection")
        conn.grid(row=0, column=0, sticky="ew", padx=2, pady=2)
        conn.columnconfigure(1, weight=1)

        ttk.Label(conn, text="Port:").grid(row=0, column=0, sticky="w", padx=4, pady=4)
        self.port_entry = ttk.Entry(conn)
        self.port_entry.grid(row=0, column=1, sticky="ew", padx=4, pady=4)
        self.port_entry.insert(0, "/dev/ttyACM0")

        ttk.Label(conn, text="Baud:").grid(row=0, column=2, sticky="w", padx=4, pady=4)
        self.baud_entry = ttk.Entry(conn, width=10)
        self.baud_entry.grid(row=0, column=3, sticky="w", padx=4, pady=4)
        self.baud_entry.insert(0, "115200")

        self.connect_button = ttk.Button(conn, text="Connect", command=self.connect)
        self.connect_button.grid(row=0, column=4, sticky="e", padx=4, pady=4)

    def _build_step_frame(self):
        step = ttk.LabelFrame(self.main_frame, text="Step sizes (mm)")
        step.grid(row=1, column=0, sticky="ew", padx=2, pady=2)
        step.columnconfigure(1, weight=1)
        step.columnconfigure(3, weight=1)

        ttk.Label(step, text="X/Y step:").grid(row=0, column=0, sticky="w", padx=4, pady=4)
        self.xy_step_entry = ttk.Entry(step)
        self.xy_step_entry.grid(row=0, column=1, sticky="ew", padx=4, pady=4)
        self.xy_step_entry.insert(0, "1.0")

        ttk.Label(step, text="Z step:").grid(row=0, column=2, sticky="w", padx=4, pady=4)
        self.z_step_entry = ttk.Entry(step)
        self.z_step_entry.grid(row=0, column=3, sticky="ew", padx=4, pady=4)
        self.z_step_entry.insert(0, "0.5")

    def _build_control_frame(self):
        control = ttk.LabelFrame(self.main_frame, text="Motion controls")
        control.grid(row=2, column=0, sticky="nsew", padx=2, pady=2)
        control.columnconfigure(0, weight=1)
        control.columnconfigure(1, weight=1)
        control.columnconfigure(2, weight=1)

        axis_frame = ttk.Frame(control)
        axis_frame.grid(row=0, column=0, columnspan=3, sticky="nsew", padx=4, pady=4)

        self._add_axis_buttons(axis_frame, "X", row=0)
        self._add_axis_buttons(axis_frame, "Y", row=1)
        self._add_axis_buttons(axis_frame, "Z", row=2)

        self.enable_button = ttk.Button(control, text="Enable Motion", command=self.enable_motion)
        self.enable_button.grid(row=1, column=0, sticky="ew", padx=4, pady=6)

        self.disable_button = ttk.Button(control, text="Disable Motion", command=self.disable_motion)
        self.disable_button.grid(row=1, column=1, sticky="ew", padx=4, pady=6)

        self.stop_button = ttk.Button(control, text="Stop", command=self.stop_motion)
        self.stop_button.grid(row=1, column=2, sticky="ew", padx=4, pady=6)

    def _add_axis_buttons(self, parent, axis_name, row):
        label = ttk.Label(parent, text=f"{axis_name}:")
        label.grid(row=row, column=0, sticky="w", padx=4, pady=4)

        minus = ttk.Button(parent, text=f"-{axis_name}", command=lambda a=axis_name: self.move_axis(a, negative=True))
        minus.grid(row=row, column=1, sticky="ew", padx=4, pady=4)

        plus = ttk.Button(parent, text=f"+{axis_name}", command=lambda a=axis_name: self.move_axis(a, negative=False))
        plus.grid(row=row, column=2, sticky="ew", padx=4, pady=4)

    def _build_status_frame(self):
        status = ttk.Frame(self.main_frame)
        status.grid(row=3, column=0, sticky="ew", padx=2, pady=2)
        status.columnconfigure(1, weight=1)

        ttk.Label(status, text="Status:").grid(row=0, column=0, sticky="w", padx=4, pady=4)
        self.status_label = ttk.Label(status, text="Disconnected", foreground="red")
        self.status_label.grid(row=0, column=1, sticky="w", padx=4, pady=4)

        # Position display
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

    def _set_status(self, message, color="black"):
        self.status_label.config(text=message, foreground=color)

    def _update_positions(self):
        """Query and update the current motor positions."""
        if self.comms is None or not self.comms.is_open:
            self.x_pos_label.config(text="--")
            self.y_pos_label.config(text="--")
            self.z_pos_label.config(text="--")
            return

        try:
            self.comms.query_position()
            response = self.comms.wait_for_response(expected_type=self.comms.RESPONSE_POSITION, timeout=1.0)
            if response and response.type == self.comms.RESPONSE_POSITION:
                x = response.payload.get("x_mm", 0.0)
                y = response.payload.get("y_mm", 0.0)
                z = response.payload.get("z_mm", 0.0)
                self.x_pos_label.config(text=f"{x:.3f}")
                self.y_pos_label.config(text=f"{y:.3f}")
                self.z_pos_label.config(text=f"{z:.3f}")
            else:
                self.x_pos_label.config(text="??")
                self.y_pos_label.config(text="??")
                self.z_pos_label.config(text="??")
        except Exception as exc:
            self.x_pos_label.config(text="ERR")
            self.y_pos_label.config(text="ERR")
            self.z_pos_label.config(text="ERR")
            print(f"Position query failed: {exc}")  # For debugging

    def _schedule_position_update(self):
        """Schedule the next position update if connected."""
        if self.comms is not None and self.comms.is_open:
            self.root.after(2000, self._schedule_position_update)  # Update every 2 seconds
            self._update_positions()

    def _update_connection_state(self, connected):
        state = "normal" if connected else "disabled"
        self.enable_button.config(state=state)
        self.disable_button.config(state=state)
        self.stop_button.config(state=state)

        for child in self.main_frame.winfo_children():
            if isinstance(child, ttk.LabelFrame) and child.cget("text") == "Motion controls":
                for button in child.winfo_children():
                    if isinstance(button, ttk.Button) and button not in {self.enable_button, self.disable_button, self.stop_button}:
                        button.config(state=state)

        self.connect_button.config(text="Disconnect" if connected else "Connect")
        self.port_entry.config(state="disabled" if connected else "normal")
        self.baud_entry.config(state="disabled" if connected else "normal")
        self.xy_step_entry.config(state="normal")
        self.z_step_entry.config(state="normal")

    def connect(self):
        if self.comms is not None and self.comms.is_open:
            self.disconnect()
            return

        port = self.port_entry.get().strip()
        baud_text = self.baud_entry.get().strip()

        if not port:
            messagebox.showwarning("Missing port", "Please enter a serial port to connect.")
            return

        try:
            baud = int(baud_text)
        except ValueError:
            messagebox.showwarning("Invalid baud", "Baud rate must be a whole number.")
            return

        try:
            self.comms = StageCommandComms(port=port, baudrate=baud, timeout=1.0)
            self._update_connection_state(True)
            self._set_status(f"Connected to {port}", color="green")
            # Update positions on connect
            self.root.after(500, self._update_positions)  # Delay to allow Arduino to settle
            # Start periodic position updates
            self._schedule_position_update()
        except Exception as exc:
            self.comms = None
            self._set_status(f"Connect failed: {exc}", color="red")
            messagebox.showerror("Connection failed", str(exc))

    def disconnect(self):
        if self.comms is None:
            return
        try:
            self.comms.close()
        except Exception:
            pass
        self.comms = None
        self._update_connection_state(False)
        self._set_status("Disconnected", color="red")
        self._update_positions()  # Reset positions to --

    def _parse_step(self, axis):
        raw = self.xy_step_entry.get().strip() if axis in {"X", "Y"} else self.z_step_entry.get().strip()
        try:
            value = float(raw)
        except ValueError:
            raise ValueError(f"Invalid {axis} step size: {raw}")
        return value

    def move_axis(self, axis, negative=False):
        if self.comms is None or not self.comms.is_open:
            messagebox.showwarning("Not connected", "Please connect to the Arduino before moving stages.")
            return

        try:
            step = self._parse_step(axis)
        except ValueError as exc:
            messagebox.showwarning("Invalid step size", str(exc))
            return

        if negative:
            step = -step

        x = step if axis == "X" else 0.0
        y = step if axis == "Y" else 0.0
        z = step if axis == "Z" else 0.0

        try:
            self.comms.move_relative(x, y, z)
            self._set_status(f"Sent move_rel {x:.3f}, {y:.3f}, {z:.3f}")
            # Update positions after move
            self.root.after(100, self._update_positions)  # Small delay to allow move to complete
        except Exception as exc:
            self._set_status(f"Move failed: {exc}", color="red")
            messagebox.showerror("Move failed", str(exc))

    def enable_motion(self):
        self._send_simple_command(self.comms.enable, "Enabled motion")

    def disable_motion(self):
        self._send_simple_command(self.comms.disable, "Disabled motion")

    def stop_motion(self):
        self._send_simple_command(self.comms.stop, "Stop command sent")

    def _send_simple_command(self, command_fn, success_message):
        if self.comms is None or not self.comms.is_open:
            messagebox.showwarning("Not connected", "Please connect before sending commands.")
            return

        try:
            command_fn()
            self._set_status(success_message)
        except Exception as exc:
            self._set_status(f"Command failed: {exc}", color="red")
            messagebox.showerror("Command failed", str(exc))


def main():
    root = tk.Tk()
    StageControlGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
