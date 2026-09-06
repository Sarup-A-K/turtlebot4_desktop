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
# Offboard SLAM entry point for the laptop: includes turtlebot4_navigation's
# slam.launch.py against a network-tuned params file, brings up RViz with a
# mapping-focused display config, and optionally starts joystick teleop.
# Keyboard teleop (teleop_twist_keyboard) is intentionally NOT launched here —
# it reads the terminal's stdin, which a node started by `ros2 launch` does not
# own. Run it as `ros2 run teleop_twist_keyboard teleop_twist_keyboard` in a
# second terminal instead.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('sync', default_value='false',
                          choices=['true', 'false'],
                          description='Use synchronous SLAM. Default false (async): '
                                      'async slam_toolbox tolerates the irregular scan '
                                      'arrival that WiFi introduces, where the sync node '
                                      'blocks on scans that arrive late or out of order.'),
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Launch RViz with the mapping display config'),
    DeclareLaunchArgument('joy_teleop', default_value='false',
                          choices=['true', 'false'],
                          description='Launch teleop_twist_joy for gamepad control. '
                                      'Keyboard teleop is never launched here — run '
                                      'teleop_twist_keyboard by hand in its own terminal.'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
]


def generate_launch_description():
    pkg_turtlebot4_navigation = get_package_share_directory('turtlebot4_navigation')
    pkg_turtlebot4_slam_desktop = get_package_share_directory('turtlebot4_slam_desktop')
    pkg_teleop_twist_joy = get_package_share_directory('teleop_twist_joy')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_params_arg = DeclareLaunchArgument(
        'params',
        default_value=PathJoinSubstitution(
            [pkg_turtlebot4_slam_desktop, 'config', 'slam_offboard.yaml']),
        description='slam_toolbox params file')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_turtlebot4_navigation, 'launch', 'slam.launch.py'])),
        launch_arguments=[
            ('namespace', namespace),
            ('sync', LaunchConfiguration('sync')),
            ('params', LaunchConfiguration('params')),
            ('use_sim_time', use_sim_time),
        ])

    rviz2_config = PathJoinSubstitution(
        [pkg_turtlebot4_slam_desktop, 'rviz', 'slam.rviz'])

    rviz = GroupAction([
        PushRosNamespace(namespace),

        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', rviz2_config],
             parameters=[{'use_sim_time': use_sim_time}],
             remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
             ],
             output='screen',
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])

    joy_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_teleop_twist_joy, 'launch', 'teleop-launch.py'])),
        condition=IfCondition(LaunchConfiguration('joy_teleop')))

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(slam_params_arg)
    ld.add_action(slam_launch)
    ld.add_action(rviz)
    ld.add_action(joy_teleop)
    return ld
