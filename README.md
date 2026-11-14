# NanoPi NEO line follower

A USB camera detects a contrasting line; a PID controller steers two motors through four software PWM signals and one shared motor-enable output. Defaults assume a **dark line on a light floor** and an H-bridge with two independent, active-high inputs per motor. Two active-high hold-to-run GPIO inputs select forward line following or straight backward motion. If the line is lost, the robot stops until it reappears. A failed or stalled camera stops the robot and exits.

## Hardware

- NanoPi NEO with a forward-facing USB camera tilted down enough to see the IR-lit track ahead of the robot. IR illumination and camera IR sensitivity must make the line visibly contrast with the floor.
- A dual H-bridge motor driver accepting 3.3 V logic and PWM on each direction input. Connect the motors and a suitable motor power supply to the driver, **not** the NanoPi GPIO. Share ground between the driver and NanoPi. Follow your driver's enable, power, and current requirements.
- Default 24-pin header connections (physical BOARD numbering):

  | Signal | Pin | GPIO |
  | --- | ---: | --- |
  | Left backward | 11 | PA0 |
  | Left forward | 13 | PA2 |
  | Right backward | 15 | PA3 |
  | Right forward | 12 | PA6 |
  | Forward control input | 22 | PA1 |
  | Backward control input | 3 | PA12 |
  | Motor enable (all four EN inputs) | 5 | PA11 |
  | Ground | 14 | GND |

These GPIOs may be assigned to other peripherals; disable conflicting UART, PWM and I2C functions first. **The right-forward default changed from pin 16 to pin 12:** move that wire if you used the earlier version. Pins 3 and 5 are normally I2C SDA and SCL. The controller supports physical pins 3, 5, 11, 12, 13, 15 and 22 on the NanoPi NEO. Motor pin assignments can be changed with `--pins`; input pins with `--input-pins FORWARD BACKWARD`; and the enable pin with `--enable-pin`. All seven pins must be distinct. Check the [NanoPi NEO pinout](https://wiki.friendlyelec.com/wiki/index.php/NanoPi_NEO) for your board revision. The NanoPi's PWM hardware is limited; these are software-generated PWM signals and can jitter under Linux load. Check them with the actual driver before use. Both direction inputs low must stop/coast the chosen driver.

Connect the shared enable output to all four driver EN inputs, with a 10 kΩ pull-down from the enable line to ground so the drivers remain disabled while the NanoPi starts or the GPIO is released. Remove any jumpers tying EN to a supply. The drivers must accept an active-high 3.3 V logic signal, and their combined input current must be within the GPIO's drive rating; otherwise use an appropriate buffer or level shifter. Share ground with both drivers. Enable stays high whenever either motor is commanded to move, including during the brief direction-change interlock, and goes low when both commands stop or the controller shuts down.

For each control input, connect a normally-open switch between its GPIO pin and 3.3 V, and a 10 kΩ pull-down resistor from the pin to ground. The program also requests internal pull-downs. If another device drives the controls, use 3.3 V-compatible outputs and a common ground; never apply 5 V to a GPIO. Do not leave the inputs floating. Use momentary switches or another hold-to-run source: a high input at startup commands motion as soon as the camera and, for forward motion, the line are available.

Hold **forward** high for PID-guided line following. Hold **backward** high to reverse straight at `--speed`; the forward-facing camera cannot guide the robot behind it. Releasing the input ramps the motor commands to zero. A direction change ramps both wheels to zero before accelerating in the other direction. Set `--ramp-rate` to the maximum duty-cycle change per second (default 0.6); for example, a speed of 0.3 reaches zero in about 0.5 seconds under normal loop timing. If both inputs are high, the line is lost while forward is held, or the camera fails or stalls, the controller stops immediately rather than ramping. A live camera is required in both directions.

## Install

On a Debian/Ubuntu-based NanoPi image with Python 3.9 or newer, install uv, Python headers, a compiler, and the distribution's OpenCV bindings. The NEO uses 32-bit ARM, for which PyPI OpenCV wheels may not be available, so use the distro package rather than compiling OpenCV on the board:

```sh
sudo apt update
sudo apt install python3 python3-dev python3-opencv build-essential
uv venv --python /usr/bin/python3 --system-site-packages
uv sync
```

The GPIO backend uses the kernel GPIO character-device API through libgpiod v2. It requires access to `/dev/gpiochip0` and a kernel with GPIO character-device v2 support. To preserve the virtual environment when running as root, use its interpreter directly:

```sh
.venv/bin/python follower.py --dry-run
sudo .venv/bin/python follower.py
```

For the optional Wayland dry-run window, install GTK and its Python bindings in the system Python used by the virtual environment:

```sh
sudo apt install python3-gi python3-gi-cairo gir1.2-gtk-3.0
```

Run dry-run as the desktop user, with access to the camera and Wayland session. Dry-run simulates the forward input and does not touch GPIO. On the NanoPi, `--dry-run --dry-inputs` reads the actual control inputs without enabling motor outputs; it may require GPIO permissions. If a window cannot be opened, the controller reports why and continues printing to the terminal. The resizable window needs GTK 3 with a Wayland backend and the GTK Cairo bindings; OpenCV itself does not need GUI support. On systems with an OpenCV wheel, a plain `uv run follower.py --dry-run --camera /dev/video0` installs OpenCV automatically and leaves GPIO untouched. If using GTK from the system Python, create the virtual environment with `uv venv --python /usr/bin/python3 --system-site-packages` first. Run `uv run python -m unittest` for the hardware-free checks.

## Deploy over SSH

Install uv on the remote host first. The `dietpi` account needs SSH access and sudo privileges. From the project directory on your computer, run:

```sh
./deploy.sh HOST
```

This copies the project to `~/pulveris` on the remote host, installs any missing Debian packages and Python dependencies, and enables and starts a systemd service running as `dietpi`. The service uses the `video` and `gpio` groups for camera and GPIO access; the script creates the `gpio` group if needed and grants that group access to `/dev/gpiochip0` at service startup. Deployment selects the first USB camera's stable `/dev/v4l/by-id` link, or `/dev/video0` if none is present. The default camera request is 176×144 MJPEG at 30 fps; the camera can negotiate a different mode, and low light can reduce its actual frame rate. Re-running the script updates and restarts the service. Check it with `ssh -t dietpi@HOST 'sudo systemctl status pulveris'` and view its logs with `ssh -t dietpi@HOST 'sudo journalctl -u pulveris -b'`. To change camera or controller options, edit `ExecStart` in `/etc/systemd/system/pulveris.service` on the host, then run `sudo systemctl daemon-reload` and `sudo systemctl restart pulveris`. Deploying again replaces the service file.

## Web control and tuning

The deployed service listens on all network interfaces on port 8080. Open `http://HOST:8080` on your computer and sign in with HTTP Basic authentication: username `robot` and the password from `ssh dietpi@HOST 'cat ~/pulveris/.web-token'`. The password is created on first start and kept across deployments. The page shows the camera, control status, motor commands, actual frame size, processed fps and processing time through motor updates. To run the web server manually, add `--web` to the usual `follower.py` command; `--web-host`, `--web-port`, `--config`, and `--web-token-file` change its address, port, and file paths. HTTP sends the password, camera stream, and controls without encryption, so use it only on a trusted network.

Select **Take remote control**, then hold a direction button or arrow key. Remote control drives the motors directly; it does not follow the line or stop for line loss, although a working camera is still required. Releasing the control or losing its heartbeat for 0.5 seconds stops the motors immediately. After a timeout, release and press again to resume. **Stop** selects remote mode and stops motion, including when physical control was active. Remote mode stays selected and physical inputs are ignored until **Use physical controls** is selected; switching back can cause motion immediately if a physical input is already high. The robot starts in physical-control mode after a restart.

Settings are validated, applied to the running controller, and saved to `~/pulveris/pulveris.json` when you select Save. Saved values override corresponding command-line defaults on later starts; pin assignments and camera selection remain command-line settings. Deployments preserve both the settings file and token. Check new tuning and remote driving with the wheels lifted before operating on the floor.

## Calibrate and run

1. Aim the camera forward and down so that the lower part of the image shows the road, with the horizon and background above it. Disconnect motor power or lift the wheels and run `--dry-run`. On Wayland the live window shows the road region in cyan, the selected line in green, its weighted center and direction in red, and the PID correction as a magenta horizontal arrow (right means steer right). It also shows the signed error, correction, and left/right motor commands. Closing the window exits. Terminal output remains available without a display.

   Adjust `--roi-start` so the top edge of the trapezoid is below the horizon. Adjust `--roi-top-width` and `--roi-bottom-width` so the trapezoid covers the road at its near and far edges. The detector combines an absolute brightness cutoff with local contrast, rejects wide runs, and joins nearby line fragments. It favors substantial, vertically extended marks near the robot over short floor marks. Nearby pixels receive more weight while retaining some look-ahead for bends. The displayed error is smoothed over time; the red point shows the current unfiltered detection. It assumes the camera points along the robot's forward axis and is not mirrored. The error should be negative when the line is left of center, positive when right, and `lost` when no line is present. Use `--light-line` for a bright line; adjust `--threshold` and `--contrast` for the line and lighting. Increase `--max-line-width` if the real line is rejected as too wide, and adjust `--min-fraction` / `--max-fraction` for its visible size. `--smoothing` controls how quickly steering responds (seconds; 0 disables smoothing). Check straight sections, turns, line loss, shadows and intersections in the real camera view: thresholding cannot reliably distinguish every marking or shadow from the track.
2. With wheels lifted, connect the driver and run at low speed. With neither control high, both wheels should be stopped. Hold forward high: both wheels should move forward on a centered line; a line to the right should increase left and decrease right wheel speed. Swap the relevant forward/backward signal wires or pin assignments if a motor is reversed. The controller may reverse an inside wheel on a sharp turn. Release forward and check that both wheels slow to a stop; hold backward and check that both reverse. Verify that simultaneous high inputs stop both wheels.
3. Start with `--ki 0 --kd 0`, increase `--kp` until it follows turns, then add a little `--kd` to reduce oscillation. Add `--ki` only if there is a persistent offset. Set `--speed`, `--pwm-hz`, and the camera `--width`/`--height` for the actual robot. Run `python follower.py --help` for all options.

Ctrl-C or SIGTERM stops the outputs. Test the camera disconnect and line-loss behavior with the wheels lifted. A software process cannot guarantee stopping on power loss, a kernel freeze, SIGKILL, or an electrical fault; use a driver with a hardware watchdog or independent emergency stop when needed. Frame timestamps are recorded when OpenCV delivers a frame, so camera/driver buffering can add latency. USB camera setup and available permissions vary by image.
