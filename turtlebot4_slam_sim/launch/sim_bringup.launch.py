# Copyright 2026 Sarup Arumugam Kumar
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Ignition Fortress bringup for TurtleBot 4: spawns the robot and its ros_gz bridge via
# turtlebot4_ignition_bringup's own turtlebot4_ignition.launch.py (rviz:=false — RViz
# comes from turtlebot4_slam_desktop instead, so there's exactly one RViz config shared
# by both sim and real modes, not two that can drift apart).
#
# Sets IGN_PARTITION/IGN_IP as a second line of defence behind the shell's tb4-sim
# profile (scripts/aliases.sh): Ignition Transport does its own peer discovery, entirely
# separate from ROS 2/DDS, so a correct ROS_DOMAIN_ID alone does not stop a sim instance
# finding another one on the Wi-Fi. Note the env var names: this is Ignition Fortress
# (Humble's simulator), which predates the Gazebo rename — it's IGN_PARTITION/IGN_IP
# here, not GZ_PARTITION/GZ_IP (that naming only applies from Gazebo Garden/Harmonic
# onward, e.g. a Jazzy-targeted sibling project). Confirmed by reading
# turtlebot4_ignition_bringup/launch/ignition.launch.py, which itself only sets
# IGN_GAZEBO_RESOURCE_PATH / IGN_GUI_PLUGIN_PATH — no GZ_-prefixed vars exist in this
# distro's simulator at all.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, \
    SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Ignition world (warehouse, depot, or maze — see '
                                      'turtlebot4_ignition_bringup/worlds; drop custom '
                                      "worlds in this package's own worlds/ directory)"),
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='TurtleBot 4 model to spawn'),
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the spawn pose'))


def generate_launch_description():
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')

    turtlebot4_ignition_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'turtlebot4_ignition.launch.py'])

    # Scope this Ignition instance to the current user and pin it to loopback so it
    # can't be discovered by (or discover) another instance on the network — see
    # module docstring above.
    ign_partition = SetEnvironmentVariable(
        name='IGN_PARTITION', value='tb4slam_' + os.environ.get('USER', 'ros'))
    ign_ip = SetEnvironmentVariable(name='IGN_IP', value='127.0.0.1')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([turtlebot4_ignition_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('rviz', 'false'),
            ('world', LaunchConfiguration('world')),
            ('model', LaunchConfiguration('model')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw')),
        ])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ign_partition)
    ld.add_action(ign_ip)
    ld.add_action(sim)
    # NOTE for anyone adding a custom world file to this package's worlds/ directory:
    # turtlebot4_ignition_bringup's own ignition.launch.py *sets* (not appends)
    # IGN_GAZEBO_RESOURCE_PATH — see its SetEnvironmentVariable call. Any
    # AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ...) adding this package's
    # worlds/ dir MUST be added to this LaunchDescription *after* `sim` above, or
    # ignition.launch.py's own Set silently wins and your world directory is never
    # found — same pitfall as the Jazzy sibling project's GZ_SIM_RESOURCE_PATH.
    return ld
