"""Follow a contrasting line using a USB camera and four motor-driver inputs."""

import argparse
import math
import os
import signal
import sys
import threading
import time
from collections import deque

import cv2
import numpy as np

import web


def clamp(value, low, high):
    return max(low, min(high, value))


def narrow_runs(mask, max_width):
    """Keep horizontal runs between two pixels and max_width inclusive."""
    if max_width < 2:
        return np.zeros_like(mask)
    height, width = mask.shape
    padded = np.zeros((height, width + 2), dtype=np.bool_)
    padded[:, 1:-1] = mask != 0
    changes = np.flatnonzero((padded[:, 1:] != padded[:, :-1]).ravel())
    starts, ends = changes[::2], changes[1::2]
    lengths = ends - starts
    valid = (lengths >= 2) & (lengths <= max_width)
    markers = np.zeros((height, width + 1), dtype=np.int8)
    flat = markers.ravel()
    flat[starts[valid]] = 1
    flat[ends[valid]] = -1
    return np.cumsum(markers[:, :-1], axis=1, dtype=np.int8).astype(np.uint8) * 255


def detect_line(frame, threshold, light_line, roi_start, top_width, bottom_width,
                min_fraction, max_fraction, contrast=8, max_line_width=0.35):
    """Find the closest valid line in a trapezoidal view of the road ahead."""
    height, width = frame.shape[:2]
    if width < 2 or height < 2:
        return None, None, None, None
    top = min(int(height * roi_start), height - 1)
    center = (width - 1) / 2
    polygon = np.array([
        [round(center * (1 - top_width)), top],
        [round(center * (1 + top_width)), top],
        [round(center * (1 + bottom_width)), height - 1],
        [round(center * (1 - bottom_width)), height - 1],
    ], dtype=np.int32)
    region = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(region, [polygon], 255)
    window = max(3, round(width * 0.25) | 1)
    # Keep the pixels needed by the adaptive window and median filter above
    # the ROI; processing the rest of the upper image cannot affect it.
    gray_top = max(0, top - window // 2 - 1)
    view = frame[gray_top:]
    gray = cv2.cvtColor(view, cv2.COLOR_BGR2GRAY) if len(view.shape) == 3 else view
    gray = cv2.medianBlur(gray, 3)
    mode = cv2.THRESH_BINARY if light_line else cv2.THRESH_BINARY_INV
    _, absolute = cv2.threshold(gray, threshold, 255, mode)
    # Compare with the local floor as well as the absolute brightness; this
    # rejects large, gradually shaded areas that a fixed threshold includes.
    local = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, mode,
                                  window, -contrast if light_line else contrast)
    mask = np.zeros_like(region)
    offset = top - gray_top
    mask[top:] = cv2.bitwise_and(cv2.bitwise_and(absolute[offset:], local[offset:]), region[top:])
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,
                                      (max(3, round(width * 0.028) | 1),
                                       max(3, round(height * 0.028) | 1)))
    # Include the closing kernel's margin above the ROI so its top edge is
    # processed the same way as in the full image.
    work_top = max(0, top - kernel.shape[0] // 2)
    narrow = narrow_runs(mask[work_top:], int(width * max_line_width))
    connected = cv2.bitwise_and(cv2.morphologyEx(narrow, cv2.MORPH_CLOSE, kernel), region[work_top:])
    count, labels, stats, _ = cv2.connectedComponentsWithStats(connected, connectivity=8)
    pixels = np.bincount(labels[narrow != 0], minlength=count)
    region_area = cv2.countNonZero(region)
    road_height = height - top
    candidates = []
    for label in range(1, count):
        _, y, _, component_height, _ = stats[label]
        y += work_top
        fraction = pixels[label] / region_area
        if component_height >= max(3, road_height * 0.2) and min_fraction <= fraction <= max_fraction:
            # Favor a long, substantial line near the robot over short marks.
            proximity = 1 + (y + component_height - 1 - top) / road_height
            candidates.append((pixels[label] * component_height * proximity, label))
    if not candidates:
        return None, None, None, polygon
    label = max(candidates)[1]
    selected = np.zeros_like(mask)
    selected[work_top:] = np.where((labels == label) & (narrow != 0), 255, 0).astype(np.uint8)
    moments = cv2.moments(selected[work_top:], binaryImage=True)
    scale = max(height - 1 - top, 1)
    base = 1 - (top - work_top) / scale
    total = base * moments['m00'] + moments['m01'] / scale
    x = (base * moments['m10'] + moments['m11'] / scale) / total
    y = work_top + (base * moments['m01'] + moments['m02'] / scale) / total
    return 2 * x / (width - 1) - 1, selected, (round(x), round(y)), polygon


def smooth_error(error, previous, dt, time_constant):
    if previous is None or time_constant == 0:
        return error
    alpha = 1 - math.exp(-dt / time_constant)
    return previous + alpha * (error - previous)


class DebugWindow:
    """GTK can use Wayland even when OpenCV is built without GUI support."""
    def __init__(self):
        import gi
        gi.require_version("Gtk", "3.0")
        gi.require_version("Gdk", "3.0")
        gi.require_version("GdkPixbuf", "2.0")
        gi.require_foreign("cairo")
        from gi.repository import Gdk, GdkPixbuf, GLib, Gtk

        if not Gtk.init_check()[0]:
            raise RuntimeError("GTK could not connect to the display")
        self.Gtk, self.GLib, self.GdkPixbuf, self.Gdk = Gtk, GLib, GdkPixbuf, Gdk
        self.closed = False
        self.pixbuf = None
        self.window = Gtk.Window(title="Line follower — dry run")
        self.window.set_default_size(640, 480)
        self.window.connect("destroy", self._closed)
        self.image = Gtk.DrawingArea()
        self.image.connect("draw", self._draw)
        self.window.add(self.image)
        self.window.show_all()

    def _closed(self, *_):
        self.closed = True

    def _draw(self, widget, context):
        if self.pixbuf is not None:
            width, height = self.pixbuf.get_width(), self.pixbuf.get_height()
            area_width, area_height = widget.get_allocated_width(), widget.get_allocated_height()
            scale = min(area_width / width, area_height / height)
            context.translate((area_width - width * scale) / 2, (area_height - height * scale) / 2)
            context.scale(scale, scale)
            self.Gdk.cairo_set_source_pixbuf(context, self.pixbuf, 0, 0)
            context.paint()
        return False

    def poll(self):
        while self.Gtk.events_pending():
            self.Gtk.main_iteration_do(False)
        if self.closed:
            raise KeyboardInterrupt

    def show(self, frame, selected, point, polygon, error, turn, left, right, mode="forward"):
        display = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR) if len(frame.shape) == 2 else frame.copy()
        if selected is not None:
            display[selected != 0] = (0.35 * display[selected != 0] + np.array([0, 180, 0]) * 0.65).astype(np.uint8)
        if polygon is not None:
            cv2.polylines(display, [polygon], True, (255, 180, 0), 1)
        height, width = display.shape[:2]
        center = (width - 1) // 2
        cv2.line(display, (center, 0), (center, height - 1), (255, 255, 0), 1)
        if point is not None:
            cv2.circle(display, point, 4, (0, 0, 255), -1)
            cv2.arrowedLine(display, (center, height - 1), point, (0, 0, 255), 2, tipLength=0.15)
        text = ("LINE LOST - STOP" if mode == "forward" else f"{mode.upper()} - line lost") if error is None else f"error {error:+.2f}  turn {turn:+.2f}"
        cv2.putText(display, text, (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
        cv2.putText(display, text, (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        # Magenta horizontal arrow shows the signed PID correction.
        if error is not None and abs(turn) > 0.01:
            target = round(center + turn * max(width * 0.35, 1))
            cv2.arrowedLine(display, (center, height - 12), (target, height - 12), (255, 0, 255), 2, tipLength=0.2)
        label = f"{mode.upper()}  L {left:+.2f}  R {right:+.2f}"
        cv2.putText(display, label, (5, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
        cv2.putText(display, label, (5, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
        pixels = self.GLib.Bytes.new(rgb.tobytes())
        self.pixbuf = self.GdkPixbuf.Pixbuf.new_from_bytes(pixels, self.GdkPixbuf.Colorspace.RGB, False, 8, width, height, width * 3)
        self.image.queue_draw()
        self.poll()


class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous = None

    def update(self, error, dt):
        derivative = 0.0 if self.previous is None else (error - self.previous) / dt
        candidate = clamp(self.integral + error * dt, -1.0, 1.0)
        output = self.kp * error + self.ki * candidate + self.kd * derivative
        # Integrate only when unsaturated or when the error unwinds saturation.
        if abs(output) <= 1 or error * output < 0:
            self.integral = candidate
        self.previous = error
        return clamp(self.kp * error + self.ki * self.integral + self.kd * derivative, -1, 1)


BOARD_GPIO = {3: 12, 5: 11, 11: 0, 12: 6, 13: 2, 15: 3, 22: 1}


class Inputs:
    def __init__(self, pins):
        import gpiod
        from gpiod.line import Bias, Direction

        self.pins = [BOARD_GPIO[pin] for pin in pins]
        self.gpio = gpiod
        self.request = gpiod.request_lines('/dev/gpiochip0', consumer='pulveris-inputs', config={
            tuple(self.pins): gpiod.LineSettings(direction=Direction.INPUT, bias=Bias.PULL_DOWN),
        })

    def read(self):
        return tuple(value == self.gpio.line.Value.ACTIVE for value in self.request.get_values(self.pins))

    def close(self):
        self.request.release()


class Motion:
    def __init__(self, rate):
        self.rate = rate
        self.left = self.right = 0.0
        self.direction = 0

    def step(self, forward, backward, line_visible, target, speed, dt):
        if (forward and backward) or (forward and not line_visible):
            self.left = self.right = 0.0
            self.direction = 0
            return self.left, self.right
        direction = int(forward) - int(backward)
        if direction and direction != self.direction and (self.left or self.right):
            target = (0.0, 0.0)
        else:
            if direction:
                self.direction = direction
            target = target if forward else (-speed, -speed) if backward else (0.0, 0.0)
        # Bound elapsed time so a stalled loop cannot cause a sudden jump.
        delta = self.rate * min(max(dt, 0.0), 0.1)
        self.left += clamp(target[0] - self.left, -delta, delta)
        self.right += clamp(target[1] - self.right, -delta, delta)
        return self.left, self.right

    def remote(self, direction, speed, dt):
        if direction == 'stop':
            self.left = self.right = 0.0
            self.direction = 0
            return self.left, self.right
        target = {'forward': (speed, speed), 'backward': (-speed, -speed),
                  'left': (-speed, speed), 'right': (speed, -speed)}[direction]
        delta = self.rate * min(max(dt, 0.0), 0.1)
        self.left += clamp(target[0] - self.left, -delta, delta)
        self.right += clamp(target[1] - self.right, -delta, delta)
        return self.left, self.right


class Motors:
    def __init__(self, pins, frequency, enable_pin):
        import gpiod
        from gpiod.line import Direction, Value

        self.values = Value
        self.pins = [BOARD_GPIO[pin] for pin in pins]
        self.enable_pin = BOARD_GPIO[enable_pin]
        self.period = 1 / frequency
        self.directions = [0, 0]
        self.duty = [0.0] * 4
        self.enabled = False
        self.lock = threading.Lock()
        self.changed = threading.Event()
        self.running = True
        self.error = None
        self.request = gpiod.request_lines('/dev/gpiochip0', consumer='pulveris-motors', config={
            tuple(self.pins + [self.enable_pin]): gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE),
        })
        try:
            self.thread = threading.Thread(target=self._pwm, daemon=True)
            self.thread.start()
        except BaseException:
            self.request.release()
            raise

    def _write(self, active):
        values = {pin: self.values.ACTIVE if on else self.values.INACTIVE for pin, on in zip(self.pins, active)}
        values[self.enable_pin] = self.values.ACTIVE if self.enabled else self.values.INACTIVE
        self.request.set_values(values)

    def _pwm(self):
        try:
            while True:
                self.changed.clear()
                with self.lock:
                    if not self.running:
                        return
                    phase = (time.monotonic() % self.period) / self.period
                    self._write([duty > phase for duty in self.duty])
                    edges = [duty - phase for duty in self.duty if phase < duty < 1]
                    delay = max(min(edges + [1 - phase]) * self.period, 0.0001) if any(0 < duty < 1 for duty in self.duty) else None
                self.changed.wait(delay)
        except Exception as exc:
            with self.lock:
                self.error = exc
                self.enabled = False
                self._write([False] * 4)

    def drive(self, left, right):
        # Pins are left backward, left forward, right backward, right forward.
        duty = [max(-left, 0), max(left, 0), max(-right, 0), max(right, 0)]
        directions = [(value > 0) - (value < 0) for value in (left, right)]
        reversing = any(old and new and old != new for old, new in zip(self.directions, directions))
        with self.lock:
            if self.error:
                raise RuntimeError('GPIO PWM failed') from self.error
            if not reversing and duty == self.duty and bool(any(directions)) == self.enabled:
                return
            was_enabled = self.enabled
            self.enabled = any(directions)
            self.duty = [0.0] * 4 if reversing else duty
            if reversing or not self.enabled or not was_enabled:
                self._write([False] * 4)
            self.changed.set()
        if reversing:
            time.sleep(self.period + 0.01)
            with self.lock:
                self.duty = duty
                self.changed.set()
        for index, direction in enumerate(directions):
            if direction:
                self.directions[index] = direction

    def close(self):
        try:
            with self.lock:
                self.running = False
                self.enabled = False
                self.duty = [0.0] * 4
                self._write([False] * 4)
        finally:
            self.changed.set()
            self.thread.join()
            self.request.release()


class Camera:
    def __init__(self, device, width, height):
        self.latest = None
        self.error = None
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self._capture, args=(device, width, height), daemon=True)
        self.thread.start()

    def _capture(self, device, width, height):
        camera = None
        try:
            camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
            if not camera.isOpened():
                raise RuntimeError(f"Cannot open camera {device}")
            camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            camera.set(cv2.CAP_PROP_FPS, 30)
            camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            while not self.stopped.is_set():
                ok, frame = camera.read()
                if not ok or frame is None:
                    raise RuntimeError("Camera read failed")
                self.latest = (time.monotonic(), frame)
        except Exception as exc:
            self.error = exc
        finally:
            if camera is not None:
                camera.release()

    def close(self):
        self.stopped.set()
        self.thread.join(timeout=0.2)


def options():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", default="/dev/video0")
    parser.add_argument("--width", type=int, default=176)
    parser.add_argument("--height", type=int, default=144)
    parser.add_argument("--threshold", type=int, default=100, help="absolute line brightness cutoff")
    parser.add_argument("--contrast", type=int, default=8, help="minimum local contrast against the floor")
    parser.add_argument("--max-line-width", type=float, default=0.35, help="maximum line width as fraction of image width")
    parser.add_argument("--smoothing", type=float, default=0.12, help="error smoothing time constant in seconds; 0 disables")
    parser.add_argument("--light-line", action="store_true", help="follow a light line on a dark surface")
    parser.add_argument("--roi-start", type=float, default=0.5, help="top of road region as fraction of frame height")
    parser.add_argument("--roi-top-width", type=float, default=0.6, help="road region width at top, as fraction of frame width")
    parser.add_argument("--roi-bottom-width", type=float, default=1.0, help="road region width at bottom, as fraction of frame width")
    parser.add_argument("--min-fraction", type=float, default=0.01)
    parser.add_argument("--max-fraction", type=float, default=0.60)
    parser.add_argument("--speed", type=float, default=0.3)
    parser.add_argument("--kp", type=float, default=0.5)
    parser.add_argument("--ki", type=float, default=0.0)
    parser.add_argument("--kd", type=float, default=0.05)
    parser.add_argument("--pins", type=int, nargs=4, default=[11, 13, 15, 12], metavar=("L_BACK", "L_FWD", "R_BACK", "R_FWD"))
    parser.add_argument("--input-pins", type=int, nargs=2, default=[22, 3], metavar=("FORWARD", "BACKWARD"), help="active-high hold-to-run BOARD pins")
    parser.add_argument("--enable-pin", type=int, default=5, help="active-high motor driver enable BOARD pin")
    parser.add_argument("--ramp-rate", type=float, default=0.6, help="maximum duty-cycle change per second")
    parser.add_argument("--pwm-hz", type=int, default=100)
    parser.add_argument("--timeout", type=float, default=0.5, help="maximum frame age in seconds")
    parser.add_argument("--dry-run", action="store_true", help="preview without driving motors; simulates forward input")
    parser.add_argument("--dry-inputs", action="store_true", help="in dry-run, read the actual GPIO inputs")
    parser.add_argument("--config", default="pulveris.json", help="persistent tuning settings file")
    parser.add_argument("--web", action="store_true", help="enable the web interface")
    parser.add_argument("--web-host", default="0.0.0.0", help="web listen address (default: all interfaces)")
    parser.add_argument("--web-port", type=int, default=8080)
    parser.add_argument("--web-token-file", default=".web-token", help="web authentication token file")
    args = parser.parse_args()
    numeric = (args.ramp_rate, args.smoothing, args.max_line_width, args.roi_start, args.roi_top_width, args.roi_bottom_width, args.min_fraction, args.max_fraction, args.speed, args.kp, args.ki, args.kd, args.timeout)
    safe_pins = set(BOARD_GPIO)
    if (not all(math.isfinite(v) for v in numeric) or args.width < 2 or args.height < 2
            or not 0 <= args.threshold <= 255 or not 0 <= args.contrast <= 255
            or not 0 < args.max_line_width <= 1 or args.smoothing < 0 or not 0 <= args.roi_start < 1
            or not 0 < args.roi_top_width <= 1 or not 0 < args.roi_bottom_width <= 1
            or not 0 <= args.min_fraction < args.max_fraction <= 1
            or not 0 <= args.speed <= 1 or min(args.kp, args.ki, args.kd) < 0
            or not 20 <= args.pwm_hz <= 1000 or args.timeout <= 0 or args.ramp_rate <= 0 or not 1 <= args.web_port <= 65535
            or len(set(args.pins + args.input_pins + [args.enable_pin])) != 7
            or not set(args.pins + args.input_pins + [args.enable_pin]) <= safe_pins or (args.dry_inputs and not args.dry_run)):
        parser.error("invalid settings or pins; choose seven distinct pins from 3, 5, 11, 12, 13, 15, 22")
    return args


def main():
    args = options()
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    motors = None
    inputs = None
    camera = None
    window = None
    server = None
    try:
        defaults = {name: getattr(args, name) for name in (*web.SETTINGS, 'light_line')}
        state = web.State(defaults, args.config)
        if args.web:
            server = web.serve(state, (args.web_host, args.web_port), args.web_token_file)
            print(f"Web interface listening on {args.web_host}:{args.web_port}; token file: {args.web_token_file}", flush=True)
        if not args.dry_run or args.dry_inputs:
            inputs = Inputs(args.input_pins)
        if not args.dry_run:
            motors = Motors(args.pins, args.pwm_hz, args.enable_pin)
        if args.dry_run and os.environ.get("WAYLAND_DISPLAY"):
            try:
                window = DebugWindow()
            except (ImportError, ValueError, RuntimeError, OSError) as exc:
                print(f"Wayland debug window unavailable: {exc}; using terminal output", file=sys.stderr)
        camera = Camera(args.camera, args.width, args.height)
        settings, control_mode, _ = state.snapshot()
        pid = PID(settings['kp'], settings['ki'], settings['kd'])
        motion = Motion(settings['ramp_rate'])
        previous_mode = control_mode
        previous_time = None
        filtered_error = None
        last_frame = None
        error = None
        target = (0.0, 0.0)
        turn = 0.0
        frame_times = deque()
        started = last_update = time.monotonic()
        while True:
            settings, control_mode, remote_direction = state.snapshot()
            pid.kp, pid.ki, pid.kd = settings['kp'], settings['ki'], settings['kd']
            motion.rate = settings['ramp_rate']
            if control_mode != previous_mode:
                if motors:
                    motors.drive(0, 0)
                motion.left = motion.right = 0.0
                motion.direction = 0
                pid.reset()
                previous_time = filtered_error = None
                previous_mode = control_mode
            if window:
                window.poll()
            now = time.monotonic()
            if camera.error:
                raise RuntimeError(str(camera.error)) from camera.error
            latest = camera.latest
            if latest is None or now - latest[0] > settings['timeout']:
                if motors:
                    motors.drive(0, 0)
                motion.left = motion.right = 0.0
                motion.direction = 0
                pid.reset()
                previous_time = filtered_error = None
                if (latest is not None and now - latest[0] > settings['timeout']) or (latest is None and now - started > 5):
                    raise RuntimeError("Camera frame timed out")
            else:
                fresh = latest is not last_frame
                if fresh:
                    processing_started = time.monotonic()
                    last_frame = latest
                    timestamp, frame = latest
                    if server:
                        with state.lock:
                            state.frame = frame
                    error, selected, point, polygon = detect_line(
                        frame, settings['threshold'], settings['light_line'], settings['roi_start'],
                        settings['roi_top_width'], settings['roi_bottom_width'], settings['min_fraction'], settings['max_fraction'],
                        settings['contrast'], settings['max_line_width'])
                    if time.monotonic() - timestamp > settings['timeout']:
                        raise RuntimeError("Camera frame timed out during processing")
                    if error is None:
                        turn = 0.0
                        target = (0.0, 0.0)
                        pid.reset()
                        previous_time = filtered_error = None
                    else:
                        dt = max(timestamp - previous_time, 0.001) if previous_time is not None else 0.001
                        error = filtered_error = smooth_error(error, filtered_error, dt, settings['smoothing'])
                        turn = pid.update(error, dt)
                        previous_time = timestamp
                        target = (clamp(settings['speed'] + turn, -1, 1), clamp(settings['speed'] - turn, -1, 1))
                forward, backward = inputs.read() if inputs else (True, False)
                mode = (f"remote {remote_direction}" if control_mode == 'remote' else
                        "conflict" if forward and backward else "forward" if forward else "backward" if backward else "stopped")
                if not forward or backward:
                    pid.reset()
                    previous_time = filtered_error = None
                update_time = time.monotonic()
                if update_time - latest[0] > settings['timeout']:
                    raise RuntimeError("Camera frame timed out during processing")
                # Image processing can take longer than a control heartbeat.
                # Check the lease again immediately before commanding motors.
                _, current_mode, remote_direction = state.snapshot()
                if current_mode != control_mode:
                    if motors:
                        motors.drive(0, 0)
                    motion.left = motion.right = 0.0
                    motion.direction = 0
                    continue
                # React quickly to a new frame while limiting repeated motor
                # updates between frames to the existing 100 Hz control rate.
                if not fresh and update_time - last_update < 0.01:
                    time.sleep(0.001)
                    continue
                if control_mode == 'remote':
                    left, right = motion.remote(remote_direction, settings['speed'], update_time - last_update)
                else:
                    left, right = motion.step(forward, backward, error is not None, target, settings['speed'], update_time - last_update)
                last_update = update_time
                if motors:
                    motors.drive(left, right)
                elif fresh:
                    displayed_turn = turn if mode == "forward" and error is not None else 0.0
                    if window:
                        window.show(frame, selected, point, polygon, error, displayed_turn, left, right, mode)
                    print(f"mode={mode} error={error if error is not None else 'lost'} turn={displayed_turn:+.2f} left={left:+.2f} right={right:+.2f}", flush=True)
                if server:
                    metrics = {}
                    if fresh:
                        finished = time.monotonic()
                        frame_times.append(finished)
                        while len(frame_times) > 1 and finished - frame_times[0] > 1:
                            frame_times.popleft()
                        fps = (len(frame_times) - 1) / (frame_times[-1] - frame_times[0]) if len(frame_times) > 1 and frame_times[-1] > frame_times[0] else None
                        metrics = {'fps': fps, 'processing_ms': (finished - processing_started) * 1000,
                                   'frame_age_ms': (finished - timestamp) * 1000,
                                   'resolution': [frame.shape[1], frame.shape[0]]}
                    with state.lock:
                        state.status.update({'mode': mode, 'error': error, 'turn': turn, 'left': left, 'right': right,
                                             'forward_input': forward, 'backward_input': backward, **metrics})
            time.sleep(0.001)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    finally:
        if motors:
            motors.close()
        if inputs:
            inputs.close()
        if camera:
            camera.close()
        if server:
            server.shutdown()
            server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
