"""Local web interface and persistent, validated controller settings."""

import base64
import binascii
import hmac
import json
import math
import os
import select
import secrets
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2


SETTINGS = {
    # name: (minimum, maximum, step); lower bounds of zero are inclusive.
    'threshold': (0, 255, 1), 'contrast': (0, 255, 1),
    'max_line_width': (0, 1, 0.01), 'smoothing': (0, None, 0.01),
    'roi_start': (0, 1, 0.01), 'roi_top_width': (0, 1, 0.01),
    'roi_bottom_width': (0, 1, 0.01), 'min_fraction': (0, 1, 0.001),
    'max_fraction': (0, 1, 0.001), 'speed': (0, 1, 0.01),
    'kp': (0, None, 0.01), 'ki': (0, None, 0.01), 'kd': (0, None, 0.01),
    'ramp_rate': (0, None, 0.01), 'timeout': (0, None, 0.05),
}


def validate(settings):
    if set(settings) != set(SETTINGS) | {'light_line'}:
        raise ValueError('Unknown or missing setting')
    for name, (low, high, _) in SETTINGS.items():
        value = settings[name]
        try:
            valid = (not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)
                     and value >= low and (high is None or value <= high))
        except OverflowError:
            valid = False
        if not valid:
            raise ValueError(f'Invalid value for {name}')
        if name in ('max_line_width', 'roi_top_width', 'roi_bottom_width', 'ramp_rate', 'timeout') and value == 0:
            raise ValueError(f'{name} must be positive')
        if name == 'roi_start' and value == 1:
            raise ValueError('roi_start must be less than 1')
        if name in ('threshold', 'contrast') and value != int(value):
            raise ValueError(f'{name} must be an integer')
    if not isinstance(settings['light_line'], bool) or settings['min_fraction'] >= settings['max_fraction']:
        raise ValueError('Invalid line settings')
    return settings


class State:
    def __init__(self, defaults, config_path):
        self.lock = threading.Lock()
        self.path = Path(config_path)
        self.settings = defaults.copy()
        if self.path.exists():
            with self.path.open() as f:
                saved = json.load(f)
            if not isinstance(saved, dict) or set(saved) - set(self.settings):
                raise ValueError('Invalid settings file')
            self.settings.update(saved)
        validate(self.settings)
        self.mode = 'physical'
        self.owner = None
        self.sequence = -1
        self.direction = 'stop'
        self.expires = 0.0
        self.gesture = None
        self.frame = None
        self.status = {}

    def save(self, values):
        if not isinstance(values, dict) or set(values) - set(self.settings):
            raise ValueError('Unknown settings')
        with self.lock:
            updated = validate({**self.settings, **values})
            self.path.parent.mkdir(parents=True, exist_ok=True)
            fd, temp = tempfile.mkstemp(prefix='.pulveris-', dir=self.path.parent)
            try:
                with os.fdopen(fd, 'w') as f:
                    json.dump(updated, f, indent=2, allow_nan=False)
                    f.write('\n')
                os.replace(temp, self.path)
            finally:
                if os.path.exists(temp):
                    os.unlink(temp)
            self.settings = updated
            return updated.copy()

    def set_mode(self, mode, owner):
        if mode not in ('physical', 'remote') or not isinstance(owner, str) or not 16 <= len(owner) <= 128:
            raise ValueError('Invalid mode or client')
        with self.lock:
            self.mode, self.owner = mode, owner if mode == 'remote' else None
            self.sequence, self.direction, self.expires = -1, 'stop', 0.0
            self.gesture = None

    def command(self, owner, sequence, direction, gesture):
        if (not isinstance(owner, str) or not isinstance(sequence, int) or isinstance(sequence, bool)
                or not 0 <= sequence <= 2**53 - 1 or direction not in ('forward', 'backward', 'left', 'right', 'stop')
                or not isinstance(gesture, str) or not 16 <= len(gesture) <= 128):
            raise ValueError('Invalid control command')
        with self.lock:
            if self.mode != 'remote' or self.owner != owner:
                raise ValueError('Remote control is not owned by this client')
            if sequence > self.sequence:
                self.sequence = sequence
                if direction != 'stop' and gesture == self.gesture and time.monotonic() >= self.expires:
                    self.direction = 'stop'
                    raise ValueError('Control timed out; release and press again')
                self.gesture = gesture
                self.direction = direction
                self.expires = time.monotonic() + 0.5 if direction != 'stop' else 0.0

    def snapshot(self):
        with self.lock:
            direction = self.direction if time.monotonic() < self.expires else 'stop'
            return self.settings.copy(), self.mode, direction


def token_file(path):
    path = Path(path)
    try:
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    except FileExistsError:
        pass
    else:
        with os.fdopen(fd, 'w') as f:
            f.write(secrets.token_urlsafe(32) + '\n')
    token = path.read_text().strip()
    if not token:
        raise ValueError('Web token file is empty')
    return token


def serve(state, address, token_path):
    token = token_file(token_path)
    page = Path(__file__).with_name('web.html').read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def authorized(self):
            try:
                scheme, encoded = self.headers.get('Authorization', '').split(' ', 1)
                credentials = base64.b64decode(encoded, validate=True).decode('utf-8')
                user, password = credentials.split(':', 1)
                if scheme.lower() == 'basic' and user == 'robot' and hmac.compare_digest(password.encode(), token.encode()):
                    return True
            except (ValueError, UnicodeError, binascii.Error):
                pass
            self.send_response(401)
            self.send_header('WWW-Authenticate', 'Basic realm="Pulveris", charset="UTF-8"')
            self.send_header('Content-Length', '0')
            self.end_headers()
            return False

        def reply(self, code, content, content_type='application/json'):
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(content)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.wfile.write(content)

        def do_GET(self):
            if not self.authorized():
                return
            if self.path == '/':
                self.reply(200, page, 'text/html; charset=utf-8')
            elif self.path == '/api/state':
                settings, mode, direction = state.snapshot()
                with state.lock:
                    status = state.status.copy()
                self.reply(200, json.dumps({'settings': settings, 'limits': SETTINGS, 'mode': mode,
                                            'direction': direction, 'status': status}, allow_nan=False).encode())
            elif self.path == '/stream':
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.connection.settimeout(3)
                try:
                    last_frame = None
                    while True:
                        with state.lock:
                            frame = state.frame
                        if frame is not None and frame is not last_frame:
                            last_frame = frame
                            ok, image = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                            if ok:
                                data = image.tobytes()
                                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' + str(len(data)).encode() + b'\r\n\r\n' + data + b'\r\n')
                        # Stop a stalled stream when its client disconnects,
                        # even if no new camera frames arrive to trigger a write.
                        if select.select([self.connection], [], [], 0.005)[0]:
                            break
                except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
                    pass
            else:
                self.reply(404, b'Not found', 'text/plain')

        def do_POST(self):
            if not self.authorized():
                return
            origin = self.headers.get('Origin')
            if (self.headers.get_content_type() != 'application/json' or
                    (origin and origin != 'http://' + self.headers.get('Host', ''))):
                self.reply(403, b'{"error":"Invalid origin or content type"}')
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 16384:
                    raise ValueError('Invalid request size')
                body = json.loads(self.rfile.read(size))
                if not isinstance(body, dict):
                    raise ValueError('Expected an object')
                if self.path == '/api/settings':
                    response = state.save(body)
                elif self.path == '/api/mode':
                    state.set_mode(body.get('mode'), body.get('client'))
                    response = {'ok': True}
                elif self.path == '/api/control':
                    state.command(body.get('client'), body.get('sequence'), body.get('direction'), body.get('gesture'))
                    response = {'ok': True}
                else:
                    self.reply(404, b'{"error":"Not found"}')
                    return
                self.reply(200, json.dumps(response, allow_nan=False).encode())
            except (ValueError, TypeError, OSError, json.JSONDecodeError) as exc:
                self.reply(400, json.dumps({'error': str(exc)}).encode())

        def log_message(self, fmt, *args):
            pass

    server = ThreadingHTTPServer(address, Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server
