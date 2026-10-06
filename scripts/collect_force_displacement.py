#!/usr/bin/env python3
"""Collect a force-displacement curve, optionally with touch sensor data.

Usage: adjust parameters below or call from CLI (see --help).

Behavior:
- Connects to stage via StageCommandComms and to force sensor serial port.
- Optionally connects to a touch sensor serial port (--touch-port).
- Sensor type is selected with --sensor-type (trill or pico).
- Moves Z in small increments until total displacement reached.
- Records timestamp, Z position (mm), measured Z encoder position (mm, if the
  firmware supports it), force (g), and (if connected) one full
  row of touch capacitance values per step.
- CSV columns: time_s, z_mm, z_enc_mm, force_g [, touch_0, touch_1, ..., touch_N-1]
- If |force| > safety limit at any point, moves Z back by the distance moved
  and aborts, saving partial data.

Plot:
- Force vs Z displacement (left Y axis, blue).
- Trill: max touch channel vs Z (right Y axis, orange).
- Pico: all 4 capacitance channels vs Z (right Y axis, individual lines).
"""

import time
import csv
import argparse
import re

from StageCommandComms import StageCommandComms

try:
    import serial
except Exception:
    serial = None

import numpy as np
import matplotlib.pyplot as plt
from aquarel import load_theme
from scipy.interpolate import UnivariateSpline

# Nord-based palette from the aquarel arctic_light theme
PALETTE = ["#8FBCBB", "#BF616A", "#D08770", "#EBCB8B", "#A3BE8C", "#B48EAD"]


def _nice_step(vmax, target_ticks=6):
    """Return a "round" tick step (1/2/2.5/5 x a power of 10) so that
    roughly *target_ticks* steps span [0, vmax]."""
    if vmax <= 0:
        return 1.0
    raw_step = vmax / target_ticks
    magnitude = 10 ** np.floor(np.log10(raw_step))
    for mult in (1, 2, 2.5, 5, 10):
        step = mult * magnitude
        if step >= raw_step:
            return step
    return 10 * magnitude


def _scatter_with_spline(ax, x, y, color, label=None, marker_size=18):
    """Scatter the raw (x, y) points and overlay a smoothed spline through them.

    Points with y=None are dropped. The label is attached to the scatter
    (not the spline) so the legend shows one marker per series.
    """
    pts = [(xi, yi) for xi, yi in zip(x, y) if yi is not None]
    if not pts:
        return
    xs, ys = zip(*pts)
    ax.scatter(xs, ys, s=marker_size, color=color, alpha=0.6,
               edgecolors='none', label=label)

    xs_arr, ys_arr = np.array(xs), np.array(ys)
    order = np.argsort(xs_arr)
    xs_sorted, ys_sorted = xs_arr[order], ysport[order]
    xs_uniq, idx = np.unique(xs_sorted, return_index=True)
    if len(xs_uniq) > 3:
        spline = UnivariateSpline(xs_uniq, ys_sorted[idx], k=3)
        x_smooth = np.linspace(xs_uniq[0], xs_uniq[-1], 200)
        ax.plot(x_smooth, spline(x_smooth), '-', color=color, linewidth=2)


# ---------------------------------------------------------------------------
# Parsing helpers
# ---------------------------------------------------------------------------

def parse_force_line(line):
    """Extract a float force value from a <FORCE,123.45> formatted line.

    Returns the value as a float, or None if the line doesn't match.
    """
    m = re.search(r"<FORCE,\s*([+-]?\d*\.?\d+)>", line)
    if not m:
        return None
    try:
        return float(m.group(1))
    except ValueError:
        return None


def parse_touch_line(line):
    """Parse a Trill flex sensor line into a list of integers.

    The Trill firmware outputs one line per reading in the form:
        0234 0412 0089 0731 ...  (space-separated, zero-padded 4-digit integers)

    Returns a list of ints, or None if the line can't be parsed.
    """
    try:
        values = [int(token) for token in line.split()]
        return values if values else None
    except ValueError:
        return None


def parse_pico_line(line):
    """Parse a Pico capacitance sensor line.

    Format: comma-separated floats, possibly with trailing tokens per field.
    Returns a list of 4 floats (channels 1-4, negated), or None on failure.
    """
    try:
        fields = [float(f.strip().split()[0]) for f in line.split(",")]
    except (ValueError, IndexError):
        return None
    if len(fields) < 5:
        return None
    return [-f for f in fields[1:5]]


def read_latest(ser, parse_fn, window_s=0.5):
    """Read serial lines for up to *window_s* seconds and return the last
    successfully parsed value (so we always get the freshest reading).

    *parse_fn* should accept a stripped string and return a parsed value or None.
    Returns the most recently parsed value, or None if nothing arrived.
    """
    result = None
    deadline = time.time() + window_s
    while time.time() < deadline:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
        except Exception:
            line = ''
        if not line:
            continue
        parsed = parse_fn(line)
        if parsed is not None:
            result = parsed
    return result


def read_averaged(ser, parse_fn, n, window_s=None):
    """Collect up to *n* parsed values and return their element-wise average.

    Reads for up to *window_s* seconds (defaults to n * 0.3 s, giving each
    reading ~300 ms to arrive).  If fewer than *n* samples arrive before the
    deadline, the average is computed over however many were collected.

    For list-valued parse functions (e.g. touch channels), averaging is done
    per-channel.  For scalar parse functions, a plain mean is returned.
    Returns None if no samples were collected.
    """
    if window_s is None:
        window_s = n * 0.3

    samples = []
    deadline = time.time() + window_s
    while time.time() < deadline and len(samples) < n:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
        except Exception:
            line = ''
        if not line:
            continue
        parsed = parse_fn(line)
        if parsed is not None:
            samples.append(parsed)

    if not samples:
        return None

    if isinstance(samples[0], list):
        n_ch = len(samples[0])
        return [
            sum(s[i] for s in samples if i < len(s)) / len(samples)
            for i in range(n_ch)
        ]
    return sum(samples) / len(samples)


def _wait_for_line(ser, token, timeout_s=3.0):
    """Block until a line containing *token* arrives on *ser*, or timeout.

    Used to wait out the Arduino's post-reset boot sequence (triggered by
    opening the serial port) before sending commands or trusting readings.
    """
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
        except Exception:
            line = ''
        if token in line:
            return True
    return False


def load_rows_from_csv(path):
    """Load previously-collected rows from a CSV written by this script.

    Returns a list of dicts with keys: time_s, z_mm, z_enc_mm, force_g,
    touch (list or None),
    matching the row format produced by collect().
    """
    rows = []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        touch_fields = sorted(
            (name for name in reader.fieldnames if name.startswith('touch_')),
            key=lambda name: int(name.split('_')[1])
        )
        for r in reader:
            touch_values = None
            if touch_fields:
                vals = [float(r[name]) if r[name] != '' else None for name in touch_fields]
                if any(v is not None for v in vals):
                    touch_values = vals
            rows.append({
                'time_s':  float(r['time_s']) if r['time_s'] != '' else None,
                'z_mm':    float(r['z_mm']) if r['z_mm'] != '' else None,
                # z_enc_mm is absent from CSVs written before encoder support.
                'z_enc_mm': (float(r['z_enc_mm'])
                             if r.get('z_enc_mm') not in (None, '') else None),
                'force_g': float(r['force_g']) if r['force_g'] != '' else None,
                'touch':   touch_values,
            })
    return rows


def read_encoder_mm(stage, axis='Z', timeout=1.0):
    """Return the measured encoder position in mm for one axis.

    Returns None when the axis has no encoder fitted, or when the firmware
    predates encoder support - in both cases the caller falls back to the
    commanded position.
    """
    try:
        stage.query_encoder()
    except Exception:
        return None

    # The board sends one ENCODER line per fitted axis, so keep reading until
    # the requested axis turns up or the window closes.
    end = time.time() + timeout
    while time.time() < end:
        resp = stage.read_response(timeout=max(0.0, end - time.time()))
        if resp is None:
            continue
        if resp.type == stage.RESPONSE_ENCODER:
            if resp.payload.get('axis') == axis.upper():
                return resp.payload.get('measured_mm')
            continue
        if resp.type == stage.RESPONSE_STATUS and 'NO_ENCODERS' in resp.payload.get('message', ''):
            return None
        if resp.type == stage.RESPONSE_ERROR:
            return None
    return None


# ---------------------------------------------------------------------------
# Core collection function
# ---------------------------------------------------------------------------

def collect(stage_port, stage_baud, force_port, force_baud,
            speed_mm_s, total_disp_mm, step_mm, out_csv,
            safety_limit_g=800.0,
            touch_port=None, touch_baud=115200,
            average_touch=5, sensor_type='trill', channel=None):
    """Run the force-displacement collection sequence.

    Parameters
    ----------
    stage_port / stage_baud : stage Arduino serial settings
    force_port / force_baud : force sensor serial settings
    speed_mm_s              : nominal Z movement speed (mm/s)
    total_disp_mm           : total Z distance to travel (mm)
    step_mm                 : Z increment per data point (mm)
    out_csv                 : output file path (not written here; caller writes)
    safety_limit_g          : abort if |force| exceeds this value
    touch_port / touch_baud : touch sensor serial settings.
                              Pass touch_port=None to skip touch collection.
    sensor_type             : 'trill' or 'pico' — selects the parse function.
    channel                 : 1-indexed touch channel to report/plot
                              (1-4 for pico, 1-30 for trill); None shows all.

    Returns
    -------
    rows : list of dicts with keys: time_s, z_mm, force_g, touch (list or None)
    ok   : True if collection completed normally, False if safety-aborted
    """
    if serial is None:
        raise RuntimeError('pyserial is required (pip install pyserial)')

    touch_parse_fn = parse_pico_line if sensor_type == 'pico' else parse_touch_line

    # --- Open stage and force sensor ---
    stage = StageCommandComms(port=stage_port, baudrate=stage_baud, timeout=1.0)
    try:
        stage.enable()
    except Exception:
        pass
    try:
        stage.set_speed(0, 0, int(max(1, speed_mm_s)))
    except Exception:
        pass

    # Zero the encoders so measured travel shares an origin with the commanded
    # position. Harmless on firmware without encoder support.
    try:
        stage.zero_encoder()
        stage.wait_for_response(timeout=0.5)
    except Exception:
        pass

    force_ser = serial.Serial(force_port, force_baud, timeout=1.0)
    # Opening the port toggles DTR and reboots the Arduino, which re-runs
    # setup() (including scale.tare()) after its own ~1 s startup delay.
    # Wait for that reboot/tare to finish before moving, or the first data
    # points land while the sensor is still zeroing.
    _wait_for_line(force_ser, 'FORCE_SENSOR_READY', timeout_s=3.0)

    # --- Optionally open touch sensor ---
    touch_ser = None
    if touch_port is not None:
        try:
            touch_ser = serial.Serial(touch_port, touch_baud, timeout=1.0)
            print(f"Touch sensor connected on {touch_port}")
        except Exception as exc:
            print(f"Warning: could not open touch sensor port {touch_port}: {exc}")
            touch_ser = None

    moved = 0.0
    start_time = time.time()
    rows = []
    steps = max(1, int(abs(total_disp_mm) / step_mm))

    try:
        for i in range(steps):
            # Move one step in +Z
            stage.move_relative(0.0, 0.0, step_mm)
            moved += step_mm

            # Wait for motion to settle (longer steps or slower speeds need more time)
            settle = max(0.01, step_mm / max(1e-3, speed_mm_s))
            time.sleep(settle)

            # Read the freshest force value available in the next 0.5 s
            force = read_latest(force_ser, parse_force_line, window_s=0.5)

            # Read and average touch rows (if connected)
            touch_values = None
            if touch_ser is not None:
                touch_values = read_averaged(touch_ser, touch_parse_fn, n=average_touch)

            # Query current stage position
            stage.query_position()
            resp = stage.wait_for_response(expected_type=stage.RESPONSE_POSITION, timeout=1.0)
            zpos = None
            if resp and resp.type == stage.RESPONSE_POSITION:
                zpos = resp.payload.get('z_mm', None)

            # Measured Z travel, independent of the commanded step count
            z_enc = read_encoder_mm(stage, axis='Z')

            timestamp = time.time() - start_time
            row = {
                'time_s':  timestamp,
                'z_mm':    zpos if zpos is not None else moved,
                'z_enc_mm': z_enc,
                'force_g': force,
                'touch':   touch_values,   # list of ints, or None
            }
            rows.append(row)

            if touch_values:
                if channel is not None and channel - 1 < len(touch_values):
                    ch_val = touch_values[channel - 1]
                    label = 'cap' if sensor_type == 'pico' else 'touch'
                    touch_summary = f", {label}_ch{channel}={ch_val:.2f}"
                elif sensor_type == 'pico':
                    touch_summary = ", cap=" + ", ".join(f"{v:.2f}" for v in touch_values)
                else:
                    touch_summary = f", touch_max={max(touch_values)}"
            else:
                touch_summary = ""
            enc_summary = "" if z_enc is None else f" (enc {z_enc:.3f})"
            print(f"{i+1}/{steps}: z={row['z_mm']:.3f} mm{enc_summary}, force={row['force_g']}{touch_summary}")

            # Safety check
            if force is not None and abs(force) > safety_limit_g:
                print(f"Overforce detected (>{safety_limit_g} g), aborting and returning to start.")
                try:
                    stage.move_relative(0.0, 0.0, -moved)
                except Exception:
                    pass
                return rows, False

        # Completed — return to start position and disable motors
        try:
            stage.move_relative(0.0, 0.0, -moved)
        except Exception:
            pass
        try:
            stage.disable()
        except Exception:
            pass
        return rows, True

    finally:
        for conn in [force_ser, touch_ser, stage]:
            try:
                conn.close()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(description="Collect a force-displacement curve.")
    p.add_argument('--stage-port',  default='/dev/ttyACM0')
    p.add_argument('--stage-baud',  type=int, default=115200)
    p.add_argument('--force-port',  default='/dev/ttyACM1')
    p.add_argument('--force-baud',  type=int, default=115200)
    p.add_argument('--touch-port',  default='/dev/ttyACM2',
                   help='Touch sensor serial port (omit to skip touch collection)')
    p.add_argument('--sensor-type', choices=['trill', 'pico'], default='trill',
                   help='Touch sensor type: trill (default) or pico')
    p.add_argument('--channel', type=int, default=None,
                   help='Touch sensor channel to plot/print: 1-4 for pico, '
                        '1-30 for trill; omit to show all channels')
    p.add_argument('--touch-baud',  type=int, default=115200)
    p.add_argument('--speed',   type=float, default=0.2,  help='mm/s nominal speed')
    p.add_argument('--total',   type=float, default=5.0,  help='total Z displacement (mm)')
    p.add_argument('--step',    type=float, default=0.1,  help='step size per move (mm)')
    p.add_argument('--out',     default='force_disp.csv', help='output CSV file path')
    p.add_argument('--no-show', action='store_true',      help='save plot but do not display it')
    p.add_argument('--safety',  type=float, default=800.0,
                   help='abort if |force| exceeds this value in grams')
    p.add_argument('--avg-touch', type=int, default=5,
                   help='number of touch readings to average per step (default: 5)')
    p.add_argument('--title',   default=None,
                   help='plot title (default: "Force-Displacement" [+ " + Touch"])')
    p.add_argument('--plot-encoder', action='store_true',
                   help='plot force against measured encoder Z instead of commanded Z '
                        '(falls back to commanded Z for any row without an encoder reading)')
    p.add_argument('--replot', action='store_true',
                   help='skip collection; just re-plot an existing --out CSV')
    args = p.parse_args()

    if args.channel is not None:
        max_channel = 4 if args.sensor_type == 'pico' else 30
        if not (1 <= args.channel <= max_channel):
            p.error(f'--channel must be between 1 and {max_channel} for sensor-type {args.sensor_type}')

    if args.replot:
        rows = load_rows_from_csv(args.out)
        print(f"Loaded {len(rows)} rows from {args.out}")
    else:
        rows, ok = collect(
            args.stage_port, args.stage_baud,
            args.force_port, args.force_baud,
            args.speed, args.total, args.step, args.out,
            safety_limit_g=args.safety,
            touch_port=args.touch_port,
            touch_baud=args.touch_baud,
            average_touch=args.avg_touch,
            sensor_type=args.sensor_type,
            channel=args.channel,
        )

        # --- Write CSV ---
        # Discover how many touch channels appeared in the data
        n_touch = max(
            (len(r['touch']) for r in rows if r.get('touch')),
            default=0
        )
        touch_fields = [f'touch_{i}' for i in range(n_touch)]
        fieldnames = ['time_s', 'z_mm', 'z_enc_mm', 'force_g'] + touch_fields

        with open(args.out, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
            writer.writeheader()
            for r in rows:
                csv_row = {
                    'time_s':  r['time_s'],
                    'z_mm':    r['z_mm'],
                    'z_enc_mm': r.get('z_enc_mm'),
                    'force_g': r['force_g'],
                }
                if r.get('touch'):
                    for idx, val in enumerate(r['touch']):
                        csv_row[f'touch_{idx}'] = val
                writer.writerow(csv_row)

        status = 'Collection complete' if ok else 'Collection aborted (safety limit)'
        print(f"{status}. Data saved to {args.out}")

    # --- Plot ---
    # Only use rows that have a valid Z position and force reading
    valid = [r for r in rows if r['z_mm'] is not None and r['force_g'] is not None]

    if args.plot_encoder:
        n_enc = sum(1 for r in valid if r.get('z_enc_mm') is not None)
        if n_enc == 0:
            print('No encoder readings in this data; plotting commanded Z instead.')
        elif n_enc < len(valid):
            print(f'{len(valid) - n_enc} of {len(valid)} rows have no encoder '
                  'reading; those fall back to commanded Z.')

    def _z_of(row):
        if args.plot_encoder and row.get('z_enc_mm') is not None:
            return row['z_enc_mm']
        return row['z_mm']

    zs = [_z_of(r)     for r in valid]
    fs = [r['force_g'] for r in valid]

    if not zs:
        print('No valid force/position pairs to plot.')
        return

    theme = load_theme('arctic_light')
    theme.apply()
    # Theme requests Helvetica Neue/Arial, which aren't installed on Linux;
    # fall back to a bundled sans-serif to avoid findfont warnings.
    plt.rcParams['font.sans-serif'] = ['DejaVu Sans']

    force_color = PALETTE[0]
    touch_color = PALETTE[1]

    _, ax1 = plt.subplots(figsize=(7, 4))

    # Force curve — left Y axis
    _scatter_with_spline(ax1, zs, fs, force_color, label='Force (g)')
    ax1.set_xlabel('Z position, measured (mm)' if args.plot_encoder else 'Z position (mm)')
    ax1.set_ylabel('Force (g)', color=force_color)
    ax1.tick_params(axis='y', labelcolor=force_color)

    # Touch curve(s) — right Y axis (only when touch data was collected)
    has_touch = any(r.get('touch') for r in valid)

    if has_touch:
        ax2 = ax1.twinx()
        if args.sensor_type == 'pico':
            if args.channel is not None:
                ch = args.channel - 1
                ch_vals = [r['touch'][ch] if r.get('touch') and ch < len(r['touch']) else None
                           for r in valid]
                _scatter_with_spline(ax2, zs, ch_vals, PALETTE[ch % len(PALETTE)],
                                      label=f'Cap ch{args.channel}')
                ax2.set_ylim(0,1)
            else:
                n_ch = max(len(r['touch']) for r in valid if r.get('touch'))
                for ch in range(n_ch):
                    ch_vals = [r['touch'][ch] if r.get('touch') and ch < len(r['touch']) else None
                               for r in valid]
                    _scatter_with_spline(ax2, zs, ch_vals, PALETTE[ch % len(PALETTE)],
                                          label=f'Cap ch{ch + 1}')
            ax2.set_ylabel('Relative Capacitance')
        elif args.channel is not None:
            ch = args.channel - 1
            ch_vals = [r['touch'][ch] if r.get('touch') and ch < len(r['touch']) else None
                       for r in valid]
            _scatter_with_spline(ax2, zs, ch_vals, touch_color, label=f'Touch ch{args.channel}')
            ax2.set_ylabel('Relative Capacitance', color=touch_color)
            ax2.tick_params(axis='y', labelcolor=touch_color)
        else:
            touch_maxes = [max(r['touch']) if r.get('touch') else None for r in valid]
            _scatter_with_spline(ax2, zs, touch_maxes, touch_color, label='Touch max (raw)')
            ax2.set_ylabel('Relative Capacitance', color=touch_color)
            ax2.tick_params(axis='y', labelcolor=touch_color)

        # Align gridlines: both axes start at 0, use round step sizes, and
        # share the same tick count, so a right-axis label sits at the same
        # height as its left-axis gridline.
        ax1.set_ylim(bottom=0)
        ax2.set_ylim(bottom=0)
        step1 = _nice_step(ax1.get_ylim()[1])
        step2 = _nice_step(ax2.get_ylim()[1])
        n_ticks = max(
            int(np.ceil(ax1.get_ylim()[1] / step1)),
            int(np.ceil(ax2.get_ylim()[1] / step2)),
        ) + 1
        ax1.set_ylim(0, step1 * (n_ticks - 1))
        ax2.set_ylim(0, step2 * (n_ticks - 1))
        ax1.set_yticks(np.arange(n_ticks) * step1)
        ax2.set_yticks(np.arange(n_ticks) * step2)

        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper left')
    else:
        ax1.legend(loc='upper left')

    default_title = 'Force-Displacement' + (' + Touch' if has_touch else '')
    plt.title(args.title if args.title is not None else default_title)
    plt.tight_layout()

    png = args.out.rsplit('.', 1)[0] + '.png'
    plt.savefig(png)
    print(f'Plot saved to {png}')

    if not args.no_show:
        plt.show()


if __name__ == '__main__':
    main()
