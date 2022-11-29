#! /bin/bash

set -e

rm -rf --force ./detectibot/detectibot.pdf ./detectibot/detectibot.urdf

rosrun xacro xacro robot.xacro -o ./detectibot/detectibot.urdf

check_urdf ./detectibot/detectibot.urdf
urdf_to_graphiz ./detectibot/detectibot.urdf

rm -rf ./detectibot.gv
mv detectibot.pdf ./detectibot/detectibot.pdf

evince --fullscreen ./detectibot/detectibot.pdf
