# Web-Based XYZ Stage Control

This folder contains a Flask-based browser interface for manual stage control.

## Requirements

- Python 3
- `Flask`
- `pyserial`

Install dependencies with:

```bash
pip install -r requirements.txt
```

## Run the GUI

From the repository root:

```bash
python scripts/ManualControlGUI.py
```

Then open the browser at `http://127.0.0.1:8080`.

## Features

- Flask-based web interface with tabbed controls
- Connection tab for serial port management
- Motion tab for absolute/relative moves and speed settings
- Display tab for workspace visualization and refresh
- Live canvas rendering of X/Y position
- Console-style log updates and status display
