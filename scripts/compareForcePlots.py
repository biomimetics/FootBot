#!/usr/bin/env python3
"""Compare relative-capacitance / force ratio across multiple force-displacement runs.

Reads force_disp_2.csv, force_disp_5.csv, force_disp_10.csv from the data/
folder and plots, for all three runs, three stacked subplots vs Z position:
force, capacitance (channel CHANNEL), and their ratio. Uses the same
aquarel plotting style as collect_force_displacement.py.
"""

import os
import csv

import numpy as np
import matplotlib.pyplot as plt
from aquarel import load_theme
from scipy.interpolate import UnivariateSpline

PALETTE = ["#8FBCBB", "#BF616A", "#D08770", "#EBCB8B", "#A3BE8C", "#B48EAD"]

DATA_DIR = os.path.join(os.path.dirname(__file__), 'data')
RUNS = ['2', '5', '10']
CHANNEL = 5  # 1-indexed touch channel to use for the ratio

FORCE_RANGE_G = (50.0, 1200.0)   # only keep points with force_g in this range
CAP_RANGE = (5.0, 100.0)         # only keep points with capacitance in this range


def _scatter_with_spline(ax, x, y, color, label=None, marker_size=18):
    """Scatter the raw (x, y) points and overlay a smoothed spline through them."""
    pts = [(xi, yi) for xi, yi in zip(x, y) if yi is not None]
    if not pts:
        return
    xs, ys = zip(*pts)
    ax.scatter(xs, ys, s=marker_size, color=color, alpha=0.6,
               edgecolors='none', label=label)

    xs_arr, ys_arr = np.array(xs), np.array(ys)
    order = np.argsort(xs_arr)
    xs_sorted, ys_sorted = xs_arr[order], ys_arr[order]
    xs_uniq, idx = np.unique(xs_sorted, return_index=True)
    if len(xs_uniq) > 3:
        spline = UnivariateSpline(xs_uniq, ys_sorted[idx], k=3)
        x_smooth = np.linspace(xs_uniq[0], xs_uniq[-1], 200)
        ax.plot(x_smooth, spline(x_smooth), '-', color=color, linewidth=2)


def load_run(path):
    """Read a force_disp CSV and return (z_mm, force_g, cap, ratio) lists.

    cap   = touch channel CHANNEL reading.
    ratio = cap / force_g, computed from the same filtered values plotted
            in the force/cap subplots.

    Rows are kept only where force_g falls in FORCE_RANGE_G and cap falls
    in CAP_RANGE (both inclusive).
    """
    zs, forces, caps, ratios = [], [], [], []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        touch_fields = sorted(
            (name for name in reader.fieldnames if name.startswith('touch_')),
            key=lambda name: int(name.split('_')[1])
        )
        ch_field = touch_fields[CHANNEL - 1]
        for row in reader:
            z = row['z_mm']
            force = row['force_g']
            touch = row[ch_field]
            if z == '' or force == '' or touch == '':
                continue
            z = float(z)
            force = float(force)
            touch_val = float(touch)
            if not (FORCE_RANGE_G[0] <= force <= FORCE_RANGE_G[1]):
                continue
            if not (CAP_RANGE[0] <= touch_val <= CAP_RANGE[1]):
                continue
            zs.append(z)
            forces.append(force)
            caps.append(touch_val)
            ratios.append(touch_val / force)
    return zs, forces, caps, ratios


def main():
    theme = load_theme('arctic_light')
    theme.apply()
    plt.rcParams['font.sans-serif'] = ['DejaVu Sans']

    fig, (ax_force, ax_cap, ax_ratio) = plt.subplots(
        3, 1, figsize=(7, 10), sharex=True
    )

    for i, run in enumerate(RUNS):
        path = os.path.join(DATA_DIR, f'force_disp_{run}.csv')
        zs, forces, caps, ratios = load_run(path)
        if not zs:
            print(f'No valid data in {path}, skipping.')
            continue
        color = PALETTE[i % len(PALETTE)]
        label = f'Pouch Pressure {run} PSI'
        _scatter_with_spline(ax_force, zs, forces, color, label=label)
        _scatter_with_spline(ax_cap, zs, caps, color, label=label)
        _scatter_with_spline(ax_ratio, zs, ratios, color, label=label)

    ax_force.set_ylabel('Force (g)')
    ax_force.legend(loc='upper left')
    ax_force.set_title('Force vs Displacement')

    ax_cap.set_ylabel('Relative Capacitance')
    ax_cap.legend(loc='upper left')
    ax_cap.set_title('Capacitance vs Displacement')

    ax_ratio.set_xlabel('Z position (mm)')
    ax_ratio.set_ylabel('Capacitance / Force (1/g)')
    ax_ratio.legend(loc='upper left')
    ax_ratio.set_title('Capacitance / Force vs Displacement')

    plt.tight_layout()

    png = os.path.join(DATA_DIR, 'compare_force_plots.png')
    plt.savefig(png)
    print(f'Plot saved to {png}')
    plt.show()


if __name__ == '__main__':
    main()
