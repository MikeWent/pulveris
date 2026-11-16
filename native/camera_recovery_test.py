#!/usr/bin/env python3
"""Verify the controller recovers when a camera source appears after startup."""

import pathlib
import re
import subprocess
import tempfile
import time


def wait_for(path, expected, process):
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        output = path.read_text(errors="replace")
        if expected in output:
            return
        if process.poll() is not None:
            break
        time.sleep(0.1)
    raise AssertionError(f"Missing {expected!r} in controller output:\n{output}")


with tempfile.TemporaryDirectory(prefix="pulveris-camera-test-") as directory:
    root = pathlib.Path(directory)
    log = root / "controller.log"
    with log.open("w") as output:
        process = subprocess.Popen(
            ["./pulveris", "--dry-run", "--camera", str(root / "frame%03d.ppm"),
             "--config", str(root / "settings.json")],
            stdout=output, stderr=output,
        )
        try:
            wait_for(log, "retrying", process)
            image = b"P6\n176 144\n255\n" + bytes([255, 255, 255]) * (176 * 144)
            for frame in range(60):
                (root / f"frame{frame:03}.ppm").write_bytes(image)
            wait_for(log, "Camera reconnected:", process)
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                output_text = log.read_text(errors="replace")
                commands = re.findall(r"mode=forward error=lost turn=0 left=([0-9.]+) right=([0-9.]+)", output_text)
                if any(float(left) > 0 and float(right) > 0 for left, right in commands):
                    break
                if process.poll() is not None:
                    break
                time.sleep(0.1)
            else:
                raise AssertionError(f"No forward motion without a line:\n{output_text}")
            assert any(float(left) > 0 and float(right) > 0 for left, right in commands), output_text
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()

print("Camera recovery and no-line forward test passed")
