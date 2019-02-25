#!/bin/zsh
xrandr --auto
xrandr --output eDP-1-1 --left-of HDMI-1-1
xrandr --output HDMI-1-1 --mode 1440x900
echo "Turn off what you do not need!"
