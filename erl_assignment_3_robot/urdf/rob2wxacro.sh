#! /bin/bash

set -e

# rm -rf --force ./detectibot/detectibot.pdf ./detectibot/detectibot.urdf

rosrun xacro xacro robo3.xacro -o ./test.urdf

check_urdf ./test.urdf
urdf_to_graphiz ./test.urdf

# rm -rf ./detectibot.gv
# mv detectibot.pdf ./detectibot/detectibot.pdf

# evince --fullscreen ./detectibot/detectibot.pdf
