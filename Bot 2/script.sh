#!/bin/bash

# Start roscore in a new terminal
gnome-terminal -- bash -c "roscore; exec bash"

# Sleep to ensure roscore has time to start
sleep 5

# Start rosserial_python node in a new terminal
gnome-terminal -- bash -c "rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200; exec bash"

# Sleep to ensure the serial node has time to start
sleep 5

# Start motor_control launch file in a new terminal
gnome-terminal -- bash -c "roslaunch motor_control motor_control.launch; exec bash"

echo "All ROS nodes have been started."
