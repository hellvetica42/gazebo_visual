#!/bin/bash
rosrun joy joy_node _dev:=/dev/input/js0 &
sleep 2
rosrun joy joy_node _dev:=/dev/input/js1 &
sleep 2
rosrun joy joy_node _dev:=/dev/input/js2 &
