#!/bin/sh
set -eu

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 HOST" >&2
    exit 1
fi

host="dietpi@$1"
project_dir=$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)

ssh "$host" 'mkdir -p "$HOME/pulveris"'
tar -C "$project_dir" -cf - follower.py web.py web.html pyproject.toml uv.lock README.md |
    ssh "$host" 'tar -xf - -C "$HOME/pulveris"'

ssh -t "$host" '
set -eu
export PATH="$HOME/.local/bin:$PATH"
cd "$HOME/pulveris"

command -v uv >/dev/null
sudo systemctl show --property=Version >/dev/null
for package in build-essential python3-dev python3-opencv; do
    if ! dpkg -s "$package" 2>/dev/null | grep -q "^Status: install ok installed$"; then
        sudo apt-get update
        sudo apt-get install -y build-essential python3-dev python3-opencv
        break
    fi
done

if sudo systemctl is-active --quiet pulveris.service; then
    sudo systemctl stop pulveris.service
fi

if [ ! -d .venv ]; then
    uv venv --python /usr/bin/python3 --system-site-packages
fi
uv sync --locked

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
ExecStart=$HOME/pulveris/.venv/bin/python $HOME/pulveris/follower.py --camera "$camera" --web

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable pulveris.service
sudo systemctl restart pulveris.service
'
