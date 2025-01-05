#!/bin/bash
# SPDX-FileCopyrightText: 2024 Kenta Hirachi
# SPDX-License-Identifier: Apache-2.0

dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
colcon build
source $dir/.bashrc

timeout 60 ros2 run keyboard_teleop keyboard_teleop | tee /tmp/keyboard_teleop.log
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: -0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
echo "Waiting for messages on 'cmd_vel' topic..."
ros2 topic echo /cmd_vel -n 1 | tee /tmp/cmd_vel_output.log

cat /tmp/cmd_vel_output.log
