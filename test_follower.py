import unittest
import time
from contextlib import redirect_stderr
from io import StringIO
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np

import follower


class FollowerTests(unittest.TestCase):
    def test_narrow_runs_matches_row_filter_at_edges_and_even_widths(self):
        rng = np.random.default_rng(7)
        for width in (2, 7, 32, 100):
            image = (rng.random((40, width)) > .5).astype(np.uint8) * 255
            image[0, :] = 255
            image[1, :width // 2] = 255
            for maximum in (1, 2, width // 2, width - 1, width):
                expected = np.zeros_like(image)
                for row, values in enumerate(image):
                    edges = np.flatnonzero(np.diff(np.pad(values != 0, (1, 1)).astype(np.int8)))
                    for start, end in zip(edges[::2], edges[1::2]):
                        if 2 <= end - start <= maximum:
                            expected[row, start:end] = 255
                np.testing.assert_array_equal(follower.narrow_runs(image, maximum), expected)

    def test_line_detection_and_steering(self):
        frame = np.full((100, 100, 3), 255, np.uint8)
        frame[60:, 70:80] = 0
        error = follower.detect_line(frame, 100, False, 0.6, 1, 1, 0.01, 0.6)[0]
        self.assertGreater(error, 0.4)
        self.assertGreater(follower.PID(0.5, 0, 0).update(error, 0.1), 0)
        frame[:] = 255
        self.assertIsNone(follower.detect_line(frame, 100, False, 0.6, 1, 1, 0.01, 0.6)[0])
        frame[:] = 0
        self.assertIsNone(follower.detect_line(frame, 100, False, 0.6, 1, 1, 0.01, 0.6)[0])
        frame[60:, 20:30] = 255
        self.assertLess(follower.detect_line(frame, 100, True, 0.6, 1, 1, 0.01, 0.6)[0], -0.4)

    def test_forward_view_uses_near_line_and_ignores_horizon(self):
        # A forward view: the distant line is narrow and the near line wider.
        frame = np.full((240, 320, 3), 255, np.uint8)
        frame[:110] = 0  # horizon and scenery, outside the road region
        cv2.fillPoly(frame, [np.array([[157, 120], [163, 120], [254, 239], [230, 239]])], (0, 0, 0))
        error, selected, point, polygon = follower.detect_line(frame, 100, False, 0.5, 0.6, 1, 0.01, 0.6)
        self.assertGreater(error, 0.2)
        self.assertGreater(point[0], 160)
        self.assertIsNotNone(selected)
        self.assertEqual(tuple(polygon[0]), (64, 120))
        self.assertEqual(selected[50, 160], 0)
        frame[120:] = 255
        self.assertIsNone(follower.detect_line(frame, 100, False, 0.5, 0.6, 1, 0.01, 0.6)[0])

    def test_unrelated_distant_mark_does_not_average_away_near_line(self):
        frame = np.full((200, 200, 3), 255, np.uint8)
        cv2.rectangle(frame, (35, 100), (50, 135), (0, 0, 0), -1)
        cv2.rectangle(frame, (145, 150), (160, 199), (0, 0, 0), -1)
        error = follower.detect_line(frame, 100, False, 0.5, 1, 1, 0.01, 0.6)[0]
        self.assertGreater(error, 0.4)

    def test_edge_line_wins_over_nearby_floor_mark_and_handles_a_gap(self):
        frame = np.full((240, 320, 3), 195, np.uint8)
        stripe = np.array([[225, 120], [258, 120], [319, 180],
                           [319, 239], [275, 239], [273, 185]], np.int32)
        cv2.fillPoly(frame, [stripe], (120, 120, 120))
        cv2.rectangle(frame, (270, 190), (319, 195), (195, 195, 195), -1)
        cv2.rectangle(frame, (155, 218), (185, 239), (110, 110, 110), -1)
        for image, sign in ((frame, 1), (cv2.flip(frame, 1), -1)):
            error, selected, point, _ = follower.detect_line(image, 160, False, .5, .6, 1, .01, .6, 8, .35)
            self.assertIsNotNone(error)
            self.assertGreater(sign * error, 0.5)
            self.assertIsNotNone(selected)
            self.assertGreater(sign * (point[0] - 159.5), 80)
        frame[:] = 195
        cv2.rectangle(frame, (155, 218), (185, 239), (110, 110, 110), -1)
        self.assertIsNone(follower.detect_line(frame, 160, False, .5, .6, 1, .01, .6, 8, .35)[0])

    def test_shadow_and_speckle_do_not_pull_track_off_course(self):
        rng = np.random.default_rng(42)
        floor = np.empty((240, 320), dtype=np.uint8)
        floor[:] = np.linspace(80, 185, 240)[:, None]
        floor[120:190, 30:260] -= 25  # broad shadow crosses the track
        self.assertIsNone(follower.detect_line(floor, 100, False, 0.5, 0.6, 1, 0.01, 0.6)[0])
        cv2.fillPoly(floor, [np.array([[166, 120], [182, 120], [225, 239], [197, 239]])], 20)
        noisy = np.clip(floor.astype(np.int16) + rng.normal(0, 5, floor.shape), 0, 255).astype(np.uint8)
        error, selected, _, _ = follower.detect_line(noisy, 100, False, 0.5, 0.6, 1, 0.01, 0.6)
        self.assertGreater(error, 0.15)
        self.assertLess(error, 0.4)
        self.assertEqual(selected[145, 80], 0)  # shadow is not the line

    def test_smoothing_is_time_based_and_can_be_disabled(self):
        first = follower.smooth_error(1, 0, 0.1, 0.2)
        second = follower.smooth_error(1, first, 0.1, 0.2)
        self.assertAlmostEqual(second, follower.smooth_error(1, 0, 0.2, 0.2))
        self.assertEqual(follower.smooth_error(1, None, 0.1, 0.2), 1)
        self.assertEqual(follower.smooth_error(1, 0, 0.1, 0), 1)

    def test_pid_reset_and_anti_windup(self):
        pid = follower.PID(2, 1, 0.5)
        self.assertEqual(pid.update(1, 0.1), 1)
        self.assertEqual(pid.integral, 0)
        pid.reset()
        self.assertEqual(pid.update(0, 0.1), 0)

    def test_motor_enable_stays_high_during_pwm_and_reversal(self):
        calls = []
        gpio = ModuleType('gpiod')
        line = ModuleType('gpiod.line')
        line.Direction = SimpleNamespace(OUTPUT='output')
        line.Value = SimpleNamespace(ACTIVE=1, INACTIVE=0)
        gpio.LineSettings = lambda **kwargs: kwargs

        class Request:
            def set_values(self, values):
                calls.append(values.copy())

            def release(self):
                calls.append('released')

        gpio.request_lines = lambda *args, **kwargs: Request()
        with patch.dict('sys.modules', {'gpiod': gpio, 'gpiod.line': line}):
            motors = follower.Motors([11, 13, 15, 12], 100, 5)
            try:
                motors.drive(0.3, 0.3)
                time.sleep(0.05)
                self.assertTrue(any(isinstance(call, dict) and (call[2] or call[6]) for call in calls))
                self.assertTrue(all(call[11] == 1 for call in calls if isinstance(call, dict) and (call[0] or call[2] or call[3] or call[6])))
                calls.clear()
                motors.drive(-0.3, -0.3)
                self.assertTrue(all(call[11] == 1 for call in calls if isinstance(call, dict)))
                self.assertTrue(any(all(call[pin] == 0 for pin in (0, 2, 3, 6)) for call in calls if isinstance(call, dict)))
                motors.drive(0, -0.3)
                self.assertTrue(motors.enabled)
                motors.drive(-0.3, 0)
                self.assertTrue(motors.enabled)
                motors.drive(0, 0)
                self.assertEqual(calls[-1][11], 0)
            finally:
                motors.close()
            self.assertEqual(calls[-1], 'released')

    def test_gpio_pins_reject_duplicates_and_unsafe_library_mapping(self):
        with patch('sys.argv', ['follower.py']):
            args = follower.options()
        self.assertEqual(args.pins, [11, 13, 15, 12])
        self.assertEqual(args.input_pins, [22, 3])
        self.assertEqual(args.enable_pin, 5)
        self.assertEqual(args.web_host, '0.0.0.0')
        self.assertEqual((args.width, args.height), (176, 144))
        for pins in (['11', '13', '15', '16'], ['11', '13', '15', '22']):
            with self.subTest(pins=pins), patch('sys.argv', ['follower.py', '--pins', *pins]), redirect_stderr(StringIO()):
                with self.assertRaises(SystemExit):
                    follower.options()
        with patch('sys.argv', ['follower.py', '--enable-pin', '11']), redirect_stderr(StringIO()):
            with self.assertRaises(SystemExit):
                follower.options()

    def test_camera_requests_mjpeg_and_thirty_fps(self):
        capture = SimpleNamespace(isOpened=lambda: True, set=unittest.mock.Mock(),
                                  read=lambda: (False, None), release=unittest.mock.Mock())
        with patch('follower.cv2.VideoCapture', return_value=capture):
            camera = follower.Camera('/dev/video-test', 176, 144)
            camera.thread.join(timeout=2)
        capture.set.assert_any_call(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        capture.set.assert_any_call(cv2.CAP_PROP_FRAME_WIDTH, 176)
        capture.set.assert_any_call(cv2.CAP_PROP_FRAME_HEIGHT, 144)
        capture.set.assert_any_call(cv2.CAP_PROP_FPS, 30)

    def test_hold_controls_ramp_reverse_and_fail_safe(self):
        motion = follower.Motion(1.0)
        self.assertEqual(motion.step(False, False, True, (0.5, 0.5), 0.5, 0.1), (0, 0))
        self.assertEqual(motion.step(True, False, True, (0.5, 0.5), 0.5, 0.1), (0.1, 0.1))
        self.assertEqual(motion.step(True, False, True, (0.5, 0.5), 0.5, 0.1), (0.2, 0.2))
        self.assertEqual(motion.step(False, False, True, (0, 0), 0.5, 0.1), (0.1, 0.1))
        # Reversal first completes the stop, then starts in the other direction.
        self.assertEqual(motion.step(False, True, False, (0, 0), 0.5, 0.1), (0, 0))
        self.assertEqual(motion.step(False, True, False, (0, 0), 0.5, 0.1), (-0.1, -0.1))
        self.assertEqual(motion.step(True, True, True, (0.5, 0.5), 0.5, 0.1), (0, 0))
        motion.step(True, False, True, (0.5, 0.5), 0.5, 0.1)
        self.assertEqual(motion.step(True, False, False, (0.5, 0.5), 0.5, 0.1), (0, 0))
        # A long scheduling delay must not jump immediately to full power.
        self.assertEqual(motion.step(True, False, True, (0.5, 0.5), 0.5, 20), (0.1, 0.1))

    def test_gpio_controls_drive_the_main_loop(self):
        frame = np.full((100, 100, 3), 255, np.uint8)
        frame[50:, 45:55] = 0
        commands = [(True, False)] * 5 + [(True, True)] + [(False, True)] * 5 + [(False, False)] * 5
        clock = [0.0]
        calls = []

        class Inputs:
            def __init__(self, pins):
                self.commands = iter(commands)

            def read(self):
                return next(self.commands)

            def close(self):
                pass

        class Motors:
            def __init__(self, *args):
                pass

            def drive(self, left, right):
                calls.append((left, right))

            def close(self):
                pass

        class Camera:
            def __init__(self, *args):
                self.error = None
                self.latest = (0.0, frame)

            def close(self):
                pass

        def sleep(_):
            clock[0] += 0.1
            if len(calls) == len(commands):
                raise KeyboardInterrupt

        with patch.object(follower, 'Inputs', Inputs), patch.object(follower, 'Motors', Motors), \
             patch.object(follower, 'Camera', Camera), patch('sys.argv', ['follower.py', '--timeout', '10']), \
             patch('follower.time.monotonic', side_effect=lambda: clock[0]), patch('follower.time.sleep', side_effect=sleep):
            self.assertEqual(follower.main(), 0)
        self.assertGreater(calls[4][0], 0)
        self.assertEqual(calls[5], (0, 0))
        self.assertLess(calls[10][0], 0)
        self.assertEqual(calls[-1], (0, 0))

    def test_camera_error_stops_and_closes_motors(self):
        calls = []

        class Motors:
            def __init__(self, *args):
                pass

            def close(self):
                calls.append('closed')

        class Camera:
            def __init__(self, *args):
                self.error = RuntimeError('camera failed')

            def close(self):
                calls.append('camera closed')

        with patch.object(follower, 'Inputs'), patch.object(follower, 'Motors', Motors), patch.object(follower, 'Camera', Camera), patch('sys.argv', ['follower.py']):
            self.assertEqual(follower.main(), 1)
        self.assertEqual(calls, ['closed', 'camera closed'])


if __name__ == '__main__':
    unittest.main()
