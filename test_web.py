import base64
import http.client
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np

import follower
import web


def defaults():
    with patch('sys.argv', ['follower.py']):
        args = follower.options()
    return {name: getattr(args, name) for name in (*web.SETTINGS, 'light_line')}


class WebTests(unittest.TestCase):
    def test_settings_are_validated_and_persisted_atomically(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'settings.json'
            state = web.State(defaults(), path)
            state.save({'speed': 0.55, 'kp': 1.25, 'light_line': True})
            restored = web.State(defaults(), path)
            self.assertEqual(restored.snapshot()[0]['speed'], 0.55)
            self.assertTrue(restored.snapshot()[0]['light_line'])
            for invalid in ({'speed': float('nan')}, {'speed': True}, {'kp': 10**500}, {'threshold': 3.5},
                            {'min_fraction': 0.9}, {'timeout': 0}, {'roi_start': 1}, {'pins': [1, 2, 3, 4]}):
                with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                    state.save(invalid)
            self.assertEqual(web.State(defaults(), path).snapshot()[0]['speed'], 0.55)

    def test_remote_lease_expires_and_cannot_be_revived_by_delayed_packets(self):
        with tempfile.TemporaryDirectory() as tmp, patch('web.time.monotonic') as clock:
            clock.return_value = 10
            state = web.State(defaults(), Path(tmp) / 'settings.json')
            owner, gesture = 'a' * 16, 'b' * 16
            state.set_mode('remote', owner)
            state.command(owner, 1, 'forward', gesture)
            self.assertEqual(state.snapshot()[1:], ('remote', 'forward'))
            state.command(owner, 0, 'backward', gesture)
            self.assertEqual(state.snapshot()[2], 'forward')
            clock.return_value = 10.6
            self.assertEqual(state.snapshot()[1:], ('remote', 'stop'))
            with self.assertRaises(ValueError):
                state.command(owner, 2, 'forward', gesture)
            state.command(owner, 3, 'left', 'c' * 16)
            self.assertEqual(state.snapshot()[2], 'left')
            state.set_mode('physical', owner)
            with self.assertRaises(ValueError):
                state.command(owner, 4, 'forward', 'd' * 16)
        motion = follower.Motion(1)
        self.assertEqual(motion.remote('forward', 0.5, 0.1), (0.1, 0.1))
        self.assertEqual(motion.remote('stop', 0.5, 0.1), (0, 0))

    def test_authenticated_http_tuning_control_and_stream(self):
        with tempfile.TemporaryDirectory() as tmp:
            state = web.State(defaults(), Path(tmp) / 'settings.json')
            state.frame = np.zeros((8, 8, 3), dtype=np.uint8)
            token_path = Path(tmp) / '.token'
            server = web.serve(state, ('127.0.0.1', 0), token_path)
            self.addCleanup(server.server_close)
            self.addCleanup(server.shutdown)
            self.assertEqual(token_path.stat().st_mode & 0o777, 0o600)
            token = token_path.read_text().strip()
            auth = 'Basic ' + base64.b64encode(f'robot:{token}'.encode()).decode()

            def request(method, path, data=None, headers=None):
                connection = http.client.HTTPConnection(*server.server_address, timeout=3)
                self.addCleanup(connection.close)
                body = json.dumps(data) if data is not None else None
                connection.request(method, path, body, headers or {})
                response = connection.getresponse()
                return response.status, response.read()

            self.assertEqual(request('GET', '/')[0], 401)
            self.assertEqual(request('GET', '/', headers={'Authorization': auth})[0], 200)
            headers = {'Authorization': auth, 'Content-Type': 'application/json'}
            self.assertEqual(request('POST', '/api/settings', {'speed': 0.4}, headers)[0], 200)
            self.assertEqual(json.loads((Path(tmp) / 'settings.json').read_text())['speed'], 0.4)
            self.assertEqual(request('POST', '/api/settings', {'speed': 1.5}, headers)[0], 400)
            self.assertEqual(request('POST', '/api/mode', {'mode': 'remote', 'client': 'a' * 16},
                                     {**headers, 'Origin': 'http://elsewhere'})[0], 403)
            self.assertEqual(request('POST', '/api/mode', {'mode': 'remote', 'client': 'a' * 16}, headers)[0], 200)
            self.assertEqual(request('POST', '/api/control', {'client': 'a' * 16, 'sequence': 1,
                                                              'direction': 'forward', 'gesture': 'b' * 16}, headers)[0], 200)
            self.assertEqual(state.snapshot()[2], 'forward')
            connection = http.client.HTTPConnection(*server.server_address, timeout=3)
            self.addCleanup(connection.close)
            connection.request('GET', '/stream', headers={'Authorization': auth})
            response = connection.getresponse()
            self.assertEqual(response.status, 200)
            self.assertEqual(response.readline(), b'--frame\r\n')
            self.assertEqual(response.readline(), b'Content-Type: image/jpeg\r\n')
            length = int(response.readline().split(b':', 1)[1])
            self.assertEqual(response.readline(), b'\r\n')
            self.assertTrue(response.read(length).startswith(b'\xff\xd8'))
            self.assertEqual(response.read(2), b'\r\n')
            # The stream should wait for a new frame, not resend a stale one.
            timer = threading.Timer(.4, lambda: setattr(state, 'frame', state.frame.copy()))
            timer.start()
            started = time.monotonic()
            self.assertEqual(response.readline(), b'--frame\r\n')
            self.assertGreater(time.monotonic() - started, .3)
            timer.join()
            response.close()

    def test_remote_timeout_stops_main_loop_even_if_physical_input_is_high(self):
        with tempfile.TemporaryDirectory() as tmp:
            state = web.State({**defaults(), 'timeout': 10}, Path(tmp) / 'settings.json')
            frame = np.full((100, 100, 3), 255, np.uint8)
            clock = [0.0]
            calls = []

            class Inputs:
                def __init__(self, pins):
                    pass

                def read(self):
                    return True, False

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
                    self.latest = (0.0, frame)
                    self.error = None

                def close(self):
                    pass

            def sleep(_):
                clock[0] += 0.1
                if len(calls) == 8:
                    raise KeyboardInterrupt

            with patch('follower.time.monotonic', side_effect=lambda: clock[0]), \
                    patch('follower.time.sleep', side_effect=sleep), \
                    patch.object(follower, 'Inputs', Inputs), patch.object(follower, 'Motors', Motors), \
                    patch.object(follower, 'Camera', Camera), patch('follower.web.State', return_value=state), \
                    patch('follower.web.serve', return_value=MagicMock()), \
                    patch('sys.argv', ['follower.py', '--web', '--timeout', '10']):
                state.set_mode('remote', 'a' * 16)
                state.command('a' * 16, 1, 'forward', 'b' * 16)
                self.assertEqual(follower.main(), 0)
            self.assertGreater(calls[1][0], 0)
            self.assertEqual(calls[-1], (0, 0))

    def test_live_performance_counts_processed_camera_frames(self):
        with tempfile.TemporaryDirectory() as tmp:
            state = web.State({**defaults(), 'timeout': 10}, Path(tmp) / 'settings.json')
            frame = np.full((100, 100, 3), 255, np.uint8)
            clock, commands = [0.0], []

            class Camera:
                error = None

                def __init__(self, *args):
                    pass

                @property
                def latest(self):
                    return clock[0], frame

                def close(self):
                    pass

            class Motors:
                def __init__(self, *args):
                    pass

                def drive(self, left, right):
                    commands.append((left, right))

                def close(self):
                    pass

            class Inputs:
                def __init__(self, pins):
                    pass

                def read(self):
                    return True, False

                def close(self):
                    pass

            def sleep(_):
                clock[0] += 0.1
                if len(commands) == 5:
                    raise KeyboardInterrupt

            with patch('follower.time.monotonic', side_effect=lambda: clock[0]), \
                    patch('follower.time.sleep', side_effect=sleep), \
                    patch.object(follower, 'Inputs', Inputs), patch.object(follower, 'Motors', Motors), \
                    patch.object(follower, 'Camera', Camera), patch('follower.web.State', return_value=state), \
                    patch('follower.web.serve', return_value=MagicMock()), \
                    patch('sys.argv', ['follower.py', '--web']):
                self.assertEqual(follower.main(), 0)
            self.assertAlmostEqual(state.status['fps'], 10)
            self.assertEqual(state.status['resolution'], [100, 100])
            self.assertEqual(state.status['processing_ms'], 0)


if __name__ == '__main__':
    unittest.main()
