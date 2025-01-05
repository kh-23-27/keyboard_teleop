# SPDX-FileCopyrightText: 2025 Kenta Hirachi
# SPDX-License=Identifier: Apache 2.0

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        
        launch_ros.actions.Node(
            package='keyboard_teleop',  
            executable='keyboard_teleop'
        ),

        launch_ros.actions.Node(
            package='keyboard_teleop',
            executable='keyboard_receive',   
            output='screen',  
        ),
    ])

