import os
import sys
import threading
import time

# Ensure the local scripts folder is available for imports.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from flask import Flask, jsonify, render_template, request

try:
    from serial.tools import list_ports
except ImportError:
    list_ports = None

from StageCommandComms import StageCommandComms

app = Flask(__name__, template_folder='templates')
lock = threading.RLock()
controller = StageCommandComms()
state = {
    'connected': False,
    'port': '',
    'baud': 115200,
    'ports': [],
    'status': 'Disconnected',
    'log': [],
    'x_mm': 0.0,
    'y_mm': 0.0,
    'z_mm': 0.0,
    # Measured encoder positions per axis, keyed X/Y/Z. An axis with no encoder
    # fitted stays absent, which the UI renders as "n/a".
    'encoders': {},
    'stage_size_in': 7.0,
    'workspace_width_in': 5.5,
    'workspace_height_in': 5.5,
}


def _log(message):
    timestamp = time.strftime('%H:%M:%S')
    entry = f'[{timestamp}] {message}'
    state['log'] = [entry] + state['log'][:99]
    state['status'] = message


def _refresh_ports():
    ports = []
    if list_ports is not None:
        ports = [port.device for port in list_ports.comports()]
    state['ports'] = ports
    if state['port'] not in ports and ports:
        state['port'] = ports[0]
    _log(f'Refreshed ports ({len(ports)})')


def _query_position():
    if not state['connected']:
        _log('Cannot query: not connected')
        return False, 'Not connected'

    try:
        controller.query_position()
        response = controller.wait_for_response(
            expected_type=StageCommandComms.RESPONSE_POSITION,
            timeout=1.0,
        )
        if response is None:
            _log('Position query timed out')
            return False, 'Position query timed out'

        payload = response.payload
        state['x_mm'] = payload['x_mm']
        state['y_mm'] = payload['y_mm']
        state['z_mm'] = payload['z_mm']
        _log(f"Position: {state['x_mm']:.2f}, {state['y_mm']:.2f}, {state['z_mm']:.2f}")
        _query_encoders()
        return True, 'Position updated'
    except Exception as exc:
        _log(f'Query failed: {exc}')
        return False, str(exc)


def _query_encoders():
    """Refresh measured encoder positions. Silent no-op without encoder support."""
    if not state['connected']:
        return

    try:
        controller.query_encoder()
        readings = {}
        # One ENCODER line arrives per fitted axis; read until the board stops.
        while True:
            response = controller.read_response(timeout=0.2)
            if response is None or response.type != StageCommandComms.RESPONSE_ENCODER:
                break
            axis = response.payload.get('axis')
            if axis:
                readings[axis] = {
                    'measured_mm': response.payload.get('measured_mm'),
                    'commanded_mm': response.payload.get('commanded_mm'),
                    'error_mm': response.payload.get('error_mm'),
                }
        state['encoders'] = readings
    except Exception as exc:
        state['encoders'] = {}
        _log(f'Encoder query failed: {exc}')


def _send_command(action, *args):
    if not state['connected']:
        _log('Cannot send command: not connected')
        return False, 'Not connected'

    try:
        action(*args)
        _log(f'Sent {action.__name__}')
        return True, f'Sent {action.__name__}'
    except Exception as exc:
        _log(f'Command failed: {exc}')
        return False, str(exc)


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/ports')
def api_ports():
    _refresh_ports()
    return jsonify(
        ports=state['ports'],
        port=state['port'],
        baud=state['baud'],
        connected=state['connected'],
    )


@app.route('/api/status')
def api_status():
    return jsonify(
        connected=state['connected'],
        port=state['port'],
        baud=state['baud'],
        status=state['status'],
        x_mm=state['x_mm'],
        y_mm=state['y_mm'],
        z_mm=state['z_mm'],
        encoders=state['encoders'],
        stage_size_in=state['stage_size_in'],
        workspace_width_in=state['workspace_width_in'],
        workspace_height_in=state['workspace_height_in'],
        log=state['log'],
    )


@app.route('/api/position')
def api_position():
    if state['connected']:
        _query_position()
    return jsonify(
        connected=state['connected'],
        status=state['status'],
        position={
            'x_mm': state['x_mm'],
            'y_mm': state['y_mm'],
            'z_mm': state['z_mm'],
        },
        encoders=state['encoders'],
    )


@app.route('/api/connect', methods=['POST'])
def api_connect():
    payload = request.get_json(silent=True) or {}
    port = payload.get('port', state['port'])
    baud = int(payload.get('baud', state['baud']))

    if not port:
        _refresh_ports()
        return jsonify(success=False, message='No serial port selected', ports=state['ports']), 400

    with lock:
        if state['connected']:
            return jsonify(success=True, message='Already connected')

        try:
            controller.open(port, baudrate=baud)
            state['connected'] = True
            state['port'] = port
            state['baud'] = baud
            _log(f'Connected to {port} @ {baud}')
            _query_position()
            return jsonify(success=True, message='Connected', port=port, baud=baud)
        except Exception as exc:
            state['connected'] = False
            _log(f'Connect failed: {exc}')
            return jsonify(success=False, message=str(exc)), 500


@app.route('/api/disconnect', methods=['POST'])
def api_disconnect():
    with lock:
        if state['connected']:
            try:
                controller.close()
            except Exception:
                pass
            state['connected'] = False
            _log('Disconnected')
            return jsonify(success=True, message='Disconnected')
    return jsonify(success=True, message='Already disconnected')


@app.route('/api/command', methods=['POST'])
def api_command():
    payload = request.get_json(silent=True) or {}
    action = payload.get('action', '')

    if action == 'move_abs':
        success, message = _send_command(
            controller.move_absolute,
            float(payload.get('x_mm', 0.0)),
            float(payload.get('y_mm', 0.0)),
            float(payload.get('z_mm', 0.0)),
        )
    elif action == 'move_rel':
        success, message = _send_command(
            controller.move_relative,
            float(payload.get('x_mm', 0.0)),
            float(payload.get('y_mm', 0.0)),
            float(payload.get('z_mm', 0.0)),
        )
    elif action == 'set_speed':
        success, message = _send_command(
            controller.set_speed,
            int(payload.get('speed_x', 0)),
            int(payload.get('speed_y', 0)),
            int(payload.get('speed_z', 0)),
        )
    elif action == 'stop':
        success, message = _send_command(controller.stop)
    elif action == 'enable':
        success, message = _send_command(controller.enable)
    elif action == 'disable':
        success, message = _send_command(controller.disable)
    elif action == 'query_pos':
        success, message = _query_position()
    elif action == 'zero_enc':
        success, message = _send_command(controller.zero_encoder)
        if success:
            _query_encoders()
    else:
        return jsonify(success=False, message='Unknown action'), 400

    if success and state['connected'] and action != 'query_pos':
        _query_position()

    return jsonify(
        success=success,
        message=message,
        connected=state['connected'],
        position={
            'x_mm': state['x_mm'],
            'y_mm': state['y_mm'],
            'z_mm': state['z_mm'],
        },
        encoders=state['encoders'],
        status=state['status'],
    )


if __name__ == '__main__':
    _refresh_ports()
    app.run(host='0.0.0.0', port=8080, threaded=True)

