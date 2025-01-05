
# SPDX-FileCopyrightText: 2024 Kenta Hirachi
# SPDX-License=Identifier: BSD-3-Clause

import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'keyboard_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kenta Hirachi',
    maintainer_email='s23c1114qb@s.chibakodai.jp',
    description='A ROS 2 package for teleoperation using keyboard input.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop = keyboard_teleop.keyboard_teleop:main',
            'keyboard_receive = keyboard_teleop.keyboard_receive:main',
        ],
    },
)
