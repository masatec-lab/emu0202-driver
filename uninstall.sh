#!/bin/bash
# Removes the plug-in and restarts Core Audio (asks for your password).
set -euo pipefail
sudo rm -rf /Library/Audio/Plug-Ins/HAL/EMU0202.driver
sudo killall coreaudiod
echo "Removed."
