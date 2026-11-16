# NanoPi NEO line follower

A USB camera detects a contrasting line; a PID controller steers two motors through four software PWM signals and one shared motor-enable output. The default is a **white line on a black floor**. The motor driver must accept two independent, active-high inputs per motor. Two active-low hold-to-run GPIO inputs select forward line following or straight backward motion. If the line is lost, the robot stops until it reappears. A failed camera stops the robot and blinks the error LED while the controller retries; if frames are too slow, the robot stops while the web controls remain available.

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
  | Tape color selector | 7 | PG11 |
  | Green line detected LED | 19 | PC0 |
  | Blue ready LED | 21 | PC1 |
  | Red error LED | 23 | PC2 |
  | Motor enable (all four EN inputs) | 5 | PA11 |
  | Ground | 14 | GND |

These GPIOs may be assigned to other peripherals; disable conflicting UART, PWM, I2C and SPI functions first. **The right-forward default changed from pin 16 to pin 12:** move that wire if you used the earlier version. Pins 3 and 5 are normally I2C SDA and SCL. The controller supports physical pins 3, 5, 7, 11, 12, 13, 15, 19, 21, 22 and 23 on the NanoPi NEO. Motor pin assignments can be changed with `--pins`; input pins with `--input-pins FORWARD BACKWARD`; the enable pin with `--enable-pin`; the tape selector with `--tape-pin`; the ready and line LEDs with `--led-pins READY LINE`; and the error LED with `--error-led-pin`. All eleven pins must be distinct. Check the [NanoPi NEO pinout](https://wiki.friendlyelec.com/wiki/index.php/NanoPi_NEO) for your board revision. The NanoPi's PWM hardware is limited; these are software-generated PWM signals and can jitter under Linux load. Check them with the actual driver before use. Both direction inputs low must stop/coast the chosen driver.

Connect each LED from its GPIO output through its own current-limiting resistor (for example, 1 kΩ) to ground, with the LED anode toward the GPIO. The green line LED lights when a fresh camera frame contains a detected line. The blue ready LED lights when a fresh frame has been processed without detecting a line. Only one LED is on at a time. The ready and line LEDs turn off during camera startup, timeouts, reconfiguration and shutdown; the line LED also turns off when the line is lost. The error LED blinks (half a second on, half a second off) on camera timeouts and controller errors, including a missing or disconnected camera. A timeout clears when frames resume. If the camera cannot be opened or reports a read failure, the controller retries every two seconds and clears the error after receiving a frame. If a camera read stalls, the robot remains stopped until frames resume. A held physical control can resume motion once the camera and line are available. For other errors that stop the controller, the error LED keeps blinking until the service is restarted; motors are stopped and the web interface remains available if it started successfully. The error LED cannot indicate failures that prevent the program or GPIO outputs from starting, or loss of power. These are active-high 3.3 V outputs.

Connect the shared enable output to all four driver EN inputs, with a 10 kΩ pull-down from the enable line to ground so the drivers remain disabled while the NanoPi starts or the GPIO is released. Remove any jumpers tying EN to a supply. The drivers must accept an active-high 3.3 V logic signal, and their combined input current must be within the GPIO's drive rating; otherwise use an appropriate buffer or level shifter. Share ground with both drivers. Enable stays high whenever either motor is commanded to move, including during the brief direction-change interlock, and goes low when both commands stop or the controller shuts down.

For each control input and the tape selector, connect a normally-open relay contact or switch between the GPIO pin and ground. The controller requests an internal pull-up on each input; for reliable wiring, add a 10 kΩ resistor from each input to 3.3 V. Remove any old pull-down resistors and any wiring that shorts an input to 3.3 V before connecting a relay to ground. Do not connect a relay contact to a separate voltage source. Use hold-to-run controls: a control contact closed at startup commands motion once it has been stable for 30 ms and the camera and, for forward motion, the line are available. Leave the tape selector open for white tape on a black floor; close it to ground for black tape on a white floor. The selector overrides the saved and web polarity setting whenever GPIO inputs are in use. Switching it stops the motors and clears the previous steering result; forward line following resumes after a frame is processed with the new polarity.

Close **forward** to ground for PID-guided line following. Close **backward** to ground to reverse straight at `--speed`; the forward-facing camera cannot guide the robot behind it. Each closure must remain stable for 30 ms before it is accepted; opening a contact is recognized immediately. Releasing the input ramps the motor commands to zero. A direction change ramps both wheels to zero before accelerating in the other direction. Set `--ramp-rate` to the maximum duty-cycle change per second (default 0.6); for example, a speed of 0.3 reaches zero in about 0.5 seconds under normal loop timing. If both contacts are closed, the line is lost while forward is held, or the camera fails or stalls, the controller stops immediately rather than ramping. A live camera is required in both directions.

## Install

On a Debian-based NanoPi image, install a C++17 compiler, OpenCV, cpp-httplib, nlohmann-json, and libgpiod **version 2 or newer**, then build:

```sh
sudo apt update
sudo apt install build-essential pkg-config libopencv-core-dev libopencv-imgproc-dev \
    libopencv-imgcodecs-dev libopencv-videoio-dev libcpp-httplib-dev \
    nlohmann-json3-dev libgpiod-dev
make -j1
make test
```

Build and run the tests with the robot service stopped on memory-constrained boards; compiling while the controller is running can stall the camera.

The GPIO backend uses the kernel GPIO character-device API through libgpiod v2. It requires access to `/dev/gpiochip0` and a kernel with GPIO character-device v2 support. Run `./pulveris --dry-run --web` for a camera and web preview without driving GPIO, or run `./pulveris --web` with GPIO and camera permissions for the robot. Dry-run simulates the forward input and uses the saved web polarity setting (white by default); `--dry-run --dry-inputs` reads actual inputs and drives the status LEDs without enabling motor outputs. The native service reports detection and motor commands in the terminal and streams the camera to the web page. The old optional Wayland debug window is not part of the native service.

## Deploy over SSH

The `dietpi` account needs SSH access and sudo privileges. The target must provide libgpiod v2 in its package repositories. From the project directory on your computer, run:

```sh
./deploy.sh HOST
```

This copies the project to `~/pulveris` on the remote host, installs any missing Debian build dependencies, builds the native service, and enables and starts a systemd service running as `dietpi`. The service uses the `video` and `gpio` groups for camera and GPIO access; the script creates the `gpio` group if needed and grants that group access to `/dev/gpiochip0` at service startup. The service selects the first USB camera's stable `/dev/v4l/by-id` link at runtime, or `/dev/video0` if none is present; it checks again on each retry so a camera connected later can be found. Pass `--camera DEVICE` to use a specific device or video file instead. The default camera request is 176×144 MJPEG at 30 fps; the camera can negotiate a different mode, and low light can reduce its actual frame rate. Re-running the script updates and restarts the service. The service must start without waiting for the camera so it can blink the error LED if the camera is missing; remove any old systemd drop-in that waits for the camera before starting the program. Check it with `ssh -t dietpi@HOST 'sudo systemctl status pulveris'` and view its logs with `ssh -t dietpi@HOST 'sudo journalctl -u pulveris -b'`. To change camera or controller options, edit `ExecStart` in `/etc/systemd/system/pulveris.service` on the host, then run `sudo systemctl daemon-reload` and `sudo systemctl restart pulveris`. Deploying again replaces the service file.

## Web control and tuning

The deployed service listens on all network interfaces on port 8080. Open `http://HOST:8080` on your computer and sign in with HTTP Basic authentication: username `robot` and the password from `ssh dietpi@HOST 'cat ~/pulveris/.web-token'`. The password is created on first start and kept across deployments. The camera overlay shows the road region in cyan, the selected line in green, its weighted center and direction in red, and the PID steering correction in magenta. The page also shows control status, motor commands, actual frame size, processed fps and processing time through motor updates. The overlay updates separately from the video and can lag slightly. To run the web server manually, add `--web` to the `./pulveris` command; `--web-host`, `--web-port`, `--config`, and `--web-token-file` change its address, port, and file paths. HTTP sends the password, camera stream, and controls without encryption, so use it only on a trusted network.

Select **Take remote control**, then hold a direction button or arrow key. Remote control drives the motors directly; it does not follow the line or stop for line loss, although a working camera is still required. Releasing the control or losing its heartbeat for 0.5 seconds stops the motors immediately. After a timeout, release and press again to resume. **Stop** selects remote mode and stops motion, including when physical control was active. Remote mode stays selected and physical inputs are ignored until **Use physical controls** is selected; switching back can cause motion if a physical contact is already closed. The robot starts in physical-control mode after a restart.

Settings, including camera resolution, are validated, applied to the running controller, and saved to `~/pulveris/pulveris.json` when you select Save. The motors stop while the camera changes resolution, then motion can resume if a control is still held. The camera may negotiate a different size; the actual size is shown under the stream. To keep detection responsive, frames larger than 320×240 are scaled to fit those dimensions for detection; the video remains at the camera resolution and the detection size is shown on the page. Saved values override corresponding command-line defaults on later starts; pin assignments and camera selection remain command-line settings. Deployments preserve both the settings file and token. Check new tuning and remote driving with the wheels lifted before operating on the floor.

## Calibrate and run

1. Aim the camera forward and down so that the lower part of the image shows the road, with the horizon and background above it. Disconnect motor power or lift the wheels and run `--dry-run`. Use `--web` to view the camera stream and detection status in a browser; signed error, correction, and left/right motor commands are also printed in the terminal.

   Adjust `--roi-start` so the top edge of the trapezoid is below the horizon. Adjust `--roi-top-width` and `--roi-bottom-width` so the trapezoid covers the road at its near and far edges. The detector combines an absolute brightness cutoff with local contrast, rejects wide runs, and joins nearby line fragments. It favors substantial, vertically extended marks near the robot over short floor marks. Nearby pixels receive more weight while retaining some look-ahead for bends. The reported error is smoothed over time. It assumes the camera points along the robot's forward axis and is not mirrored. The error should be negative when the line is left of center, positive when right, and `lost` when no line is present. Select tape polarity with the GPIO input, or use the web setting during dry-run without GPIO; adjust `--threshold` and `--contrast` for the line and lighting. Increase `--max-line-width` if the real line is rejected as too wide, and adjust `--min-fraction` / `--max-fraction` for its visible size. `--smoothing` controls how quickly steering responds (seconds; 0 disables smoothing). Check straight sections, turns, line loss, shadows and intersections in the real camera view: thresholding cannot reliably distinguish every marking or shadow from the track.
2. With wheels lifted, connect the driver and run at low speed. With both control contacts open, both wheels should be stopped. Close forward: both wheels should move forward on a centered line; a line to the right should increase left and decrease right wheel speed. Swap the relevant forward/backward signal wires or pin assignments if a motor is reversed. The controller may reverse an inside wheel on a sharp turn. Release forward and check that both wheels slow to a stop; close backward and check that both reverse. Verify that closing both contacts stops both wheels.
3. Start with `--ki 0 --kd 0`, increase `--kp` until it follows turns, then add a little `--kd` to reduce oscillation. Add `--ki` only if there is a persistent offset. Set `--speed`, `--pwm-hz`, and the camera `--width`/`--height` for the actual robot. Run `./pulveris --help` for all options.

Ctrl-C or SIGTERM stops the outputs. Test the camera disconnect and line-loss behavior with the wheels lifted. A software process cannot guarantee stopping on power loss, a kernel freeze, SIGKILL, or an electrical fault; use a driver with a hardware watchdog or independent emergency stop when needed. Frame timestamps are recorded when OpenCV delivers a frame, so camera/driver buffering can add latency. USB camera setup and available permissions vary by image.
