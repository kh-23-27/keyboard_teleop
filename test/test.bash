#!/bin/bash
# SPDX-FileCopyrightText: 2025 Kenta Hirachi
# SPDX-License-Identifier: Apache-2.0

dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
colcon build|| { echo "Build failed"; exit 1; }
source $dir/.bashrc

SESSION_NAME="teleop_session"
tmux new-session -d -s $SESSION_NAME "ros2 run keyboard_teleop keyboard_teleop > $HOME/tmp/keyboard_teleop.log 2>&1"
tmux send-keys -t keyboard_teleop "ros2 run keyboard_teleop keyboard_teleop" C-m
sleep 5
tmux send-keys -t keyboard_teleop "w" C-m
sleep 1
ros2 topic echo /cmd_vel --once
ros2 topic echo /cmd_vel -n 1 > /tmp/cmd_vel_output.log

cat /tmp/cmd_vel_output.log
