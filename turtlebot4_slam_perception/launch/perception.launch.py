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
# Config-only /scan filtering, shared by sim and real hardware. Runs laser_filters'
# scan_to_scan_filter_chain node against config/scan_filter.yaml: /scan in,
# /scan_filtered out (both are the node's own defaults — no remapping needed here).
# turtlebot4_slam_bringup points slam_toolbox at scan_filtered by remapping around
# this, not by editing the SLAM params file — see
# turtlebot4_slam_bringup/launch/tb4_slam.launch.py for why.
#
# robot_localization's ekf_node is launched here too, but only when use_ekf:=true
# (default false) — an opt-in fallback, not the normal path. See config/ekf.yaml for
# when it's actually the right tool vs. a symptom of an unrelated RMW problem.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace


ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('use_ekf', default_value='false',
                          choices=['true', 'false'],
                          description='Fuse /odom + /imu via robot_localization '
                                      'ekf_node to broadcast odom->base_link on '
                                      '/tf. Only turn this on if the base itself '
                                      "isn't already doing so — see config/ekf.yaml."),
]


def generate_launch_description():
    pkg_turtlebot4_slam_perception = get_package_share_directory(
        'turtlebot4_slam_perception')

    scan_filter_params = PathJoinSubstitution(
        [pkg_turtlebot4_slam_perception, 'config', 'scan_filter.yaml'])
    ekf_params = PathJoinSubstitution(
        [pkg_turtlebot4_slam_perception, 'config', 'ekf.yaml'])

    perception = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),

        Node(package='laser_filters',
             executable='scan_to_scan_filter_chain',
             name='scan_to_scan_filter_chain',
             parameters=[
                scan_filter_params,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
             ],
             output='screen'),

        Node(package='robot_localization',
             executable='ekf_node',
             name='ekf_filter_node',
             parameters=[
                ekf_params,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
             ],
             output='screen',
             condition=IfCondition(LaunchConfiguration('use_ekf'))),
    ])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(perception)
    return ld
