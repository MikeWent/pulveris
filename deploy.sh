#!/bin/sh
set -eu

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 HOST" >&2
    exit 1
fi

host="dietpi@$1"
project_dir=$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)

ssh "$host" 'mkdir -p "$HOME/pulveris"'
tar -C "$project_dir" --exclude='*.o' -cf - Makefile native web.html README.md |
    ssh "$host" 'tar -xf - -C "$HOME/pulveris"'

ssh -t "$host" '
set -eu
cd "$HOME/pulveris"

sudo systemctl show --property=Version >/dev/null
for package in build-essential pkg-config libopencv-core-dev libopencv-imgproc-dev libopencv-imgcodecs-dev libopencv-videoio-dev libcpp-httplib-dev nlohmann-json3-dev libgpiod-dev; do
    if ! dpkg -s "$package" 2>/dev/null | grep -q "^Status: install ok installed$"; then
        sudo apt-get update
        sudo apt-get install -y build-essential pkg-config libopencv-core-dev libopencv-imgproc-dev libopencv-imgcodecs-dev libopencv-videoio-dev libcpp-httplib-dev nlohmann-json3-dev libgpiod-dev
        break
    fi
done
if ! pkg-config --atleast-version=2 libgpiod; then
    echo "libgpiod version 2 or newer is required" >&2
    exit 1
fi

was_active=false
if sudo systemctl is-active --quiet pulveris.service; then
    was_active=true
    sudo systemctl stop pulveris.service
fi

# Compile with the service stopped: this board has little memory, and swapping
# under load can stall the camera and motor-control loop.
if ! (
    # Bound compiler memory on small boards to avoid exhausting the host.
    if [ "$(awk "/MemTotal:/ {print \$2}" /proc/meminfo)" -lt 300000 ]; then
        ulimit -v 240000
    fi
    make -j1 all
); then
    if [ "$was_active" = true ]; then sudo systemctl start pulveris.service; fi
    echo "Native build failed; the previous service was restored if it was running." >&2
    exit 1
fi

getent group gpio >/dev/null || sudo groupadd --system gpio

camera=/dev/video0
for device in /dev/v4l/by-id/*-video-index0; do
    if [ -e "$device" ]; then
        camera=$device
        break
    fi
done

sudo tee /etc/systemd/system/pulveris.service >/dev/null <<EOF
[Unit]
Description=Pulveris line follower
After=local-fs.target

[Service]
Type=simple
User=dietpi
SupplementaryGroups=video gpio
ExecStartPre=+/usr/bin/chgrp gpio /dev/gpiochip0
ExecStartPre=+/usr/bin/chmod g+rw /dev/gpiochip0
ExecStartPre=/bin/sh -c "until test -e $camera; do sleep 1; done"
WorkingDirectory=$HOME/pulveris
ExecStart=$HOME/pulveris/pulveris --camera "$camera" --web

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable pulveris.service
sudo systemctl restart pulveris.service
'
