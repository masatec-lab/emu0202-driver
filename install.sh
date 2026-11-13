#!/bin/bash
# Installs the plug-in into /Library/Audio/Plug-Ins/HAL and restarts Core Audio (asks for your password).
set -euo pipefail
cd "$(dirname "$0")"
[ -d build/EMU0202.driver ] || ./build.sh

echo "Core Audio will be restarted. Apps that are playing sound (Chrome, Logic, REAPER...) lose audio"
echo "until they are restarted, so it is best to quit them first."
read -r -p "Press Enter to continue, Ctrl+C to cancel... "

DEST=/Library/Audio/Plug-Ins/HAL/EMU0202.driver
sudo rm -rf "$DEST"
sudo cp -R build/EMU0202.driver "$DEST"
sudo chown -R root:wheel "$DEST"
sudo killall coreaudiod
echo "Installed. Choose \"E-MU 0202 | USB\" in System Settings > Sound, or in your DAW's audio settings."
echo "If a browser or DAW was open, restart it (Chrome: quit with Cmd+Q and reopen)."
