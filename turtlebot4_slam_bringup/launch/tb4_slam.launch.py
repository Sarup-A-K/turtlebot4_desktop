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
# Unified sim/real entry point for TurtleBot 4 offboard SLAM (ROS 2 Humble). `mode:=sim`
# spawns the robot in Ignition Fortress (turtlebot4_slam_sim) and runs SLAM against it;
# `mode:=real` runs the exact same SLAM/RViz layer against the physical robot, launching
# no robot-side nodes at all — the Pi's stock bringup already provides /scan, /odom,
# /tf, and /cmd_vel (see turtlebot4_slam_desktop's README: "the laptop thinks, the
# robot executes"). `mode:=view` is RViz only — for the (hopefully unnecessary, on this
# distro-matched setup — see the top-level README) case where slam_toolbox needs to run
# on the robot itself instead of the laptop. use_sim_time is derived from `mode`, not
# set alongside it, so the two can't be toggled out of sync.
#
# Built on OpaqueFunction rather than IfCondition branches: `mode` is looked up in a
# small dict to pick the SLAM params file and use_sim_time default, which (a) fails
# loudly on an unknown mode instead of silently launching nothing, and (b) reads as a
# table instead of a wall of conditionals. Ported from this project's Jazzy sibling
# (tb4_slam/turtlebot4_slam_bringup), where this design — including the OpaqueFunction
# choice specifically to dodge a LaunchConfiguration namespace collision on
# use_sim_time — was worked out and verified; carried over here unchanged since the
# same collision risk applies to any launch tree that nests multiple files each
# declaring their own use_sim_time with restricted choices.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, \
    OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace, SetRemap


# mode -> (use_sim_time default, package + file the SLAM params come from).
# 'real' deliberately points at turtlebot4_slam_desktop's existing tuned
# slam_offboard.yaml rather than a copy — one file, one place to tune WiFi params.
# 'view' runs no SLAM at all (RViz only), so it has no params entry.
MODES = {
    'sim': {
        'use_sim_time': 'true',
        'params_pkg': 'turtlebot4_slam_bringup',
        'params_file': 'slam_sim.yaml',
    },
    'real': {
        'use_sim_time': 'false',
        'params_pkg': 'turtlebot4_slam_desktop',
        'params_file': 'slam_offboard.yaml',
    },
    'view': {
        'use_sim_time': 'false',
    },
}

ARGUMENTS = [
    DeclareLaunchArgument('mode', default_value='sim', choices=['sim', 'real', 'view'],
                          description='sim: spawn TurtleBot 4 in Ignition Fortress and '
                                      'run SLAM against it. real: run SLAM on the '
                                      'laptop against the physical robot — no sim '
                                      "nodes are launched, the Pi's stock bringup "
                                      'already provides the sensor/base topics. view: '
                                      'RViz ONLY, no SLAM on the laptop — for the '
                                      'Pi-side-SLAM fallback case; every other '
                                      'argument except namespace is ignored.'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Launch RViz with the mapping display config'),
    DeclareLaunchArgument('sync', default_value='false',
                          choices=['true', 'false'],
                          description='Use synchronous SLAM. Default false (async) for '
                                      'the same reason turtlebot4_slam_desktop defaults '
                                      'it false: async tolerates irregular scan arrival, '
                                      'whether from WiFi jitter (real) or from the '
                                      'simulator falling behind real time under load '
                                      '(sim).'),
    DeclareLaunchArgument('joy_teleop', default_value='false',
                          choices=['true', 'false'],
                          description='Launch teleop_twist_joy for gamepad control'),
    DeclareLaunchArgument('filter_scan', default_value='true',
                          choices=['true', 'false'],
                          description='Run the turtlebot4_slam_perception laser_filters '
                                      'chain and point slam_toolbox at its output '
                                      '(scan_filtered) instead of the raw /scan. Same '
                                      'chain in both modes.'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Ignition world (warehouse, depot, or maze — see '
                                      'turtlebot4_ignition_bringup/worlds). Ignored '
                                      'outside mode:=sim.'),
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='TurtleBot 4 model to spawn. Ignored outside '
                                      'mode:=sim.'),
    DeclareLaunchArgument('use_ekf', default_value='false',
                          choices=['true', 'false'],
                          description='Fuse /odom + /imu via robot_localization '
                                      'ekf_node to broadcast odom->base_link on /tf. '
                                      'Leave false unless you have confirmed the base '
                                      "itself isn't already broadcasting that "
                                      'transform (ros2 topic echo /tf shows no '
                                      'base_link child frame) — see '
                                      'turtlebot4_slam_perception/config/ekf.yaml.'),
    DeclareLaunchArgument('tf_relay', default_value='false',
                          choices=['true', 'false'],
                          description='mode:=view only. Read TF from /tf_relay (the '
                                      "Pi-hosted relay of the robot's /tf, started by "
                                      'tb4-pi-slam-start) instead of /tf. Required when '
                                      'SLAM runs on the Pi: the Create 3 publishes '
                                      'odom->base_link over its USB link to the Pi only, '
                                      "so that transform never reaches this machine — "
                                      'measured directly, including with the base as an '
                                      'explicit CycloneDDS unicast peer. Without the '
                                      'relay RViz renders the map but reports "two or '
                                      'more unconnected trees" and draws no robot. '
                                      'Leave false if the relay is not running: RViz '
                                      'would then get no TF at all.'),
]
# use_sim_time is deliberately NOT a launch argument here — it is derived from `mode`
# only (see MODES above). See the module docstring: this dodges a real, previously-hit
# LaunchConfiguration namespace collision (the Jazzy sibling project's ARGUMENTS list
# has the full story of how it was found).


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    cfg = MODES[mode]
    use_sim_time = cfg['use_sim_time']

    namespace = LaunchConfiguration('namespace')

    if mode == 'view':
        # Viewer only: RViz with the shared mapping display config, nothing else. No
        # slam_toolbox, no scan filter, no sim — SLAM is expected to be running on the
        # robot (tb4-pi-slam-start). Mirrors slam_desktop.launch.py's RViz node so the
        # display config, namespace handling and /tf remaps stay identical; it just
        # doesn't drag slam_toolbox along with it, which would start a second publisher
        # of map->odom on this machine and fight the robot's.
        #
        # tf_relay: read the Pi-hosted relay of the robot's /tf instead of /tf itself.
        # /tf_static is deliberately left alone — it's published by the Pi's own
        # robot_state_publisher (TRANSIENT_LOCAL) and this machine receives it directly;
        # only the Create 3's odom->base_link, which never leaves the robot's USB link,
        # needs relaying. See the tf_relay argument's description.
        tf_relay = LaunchConfiguration('tf_relay').perform(context) == 'true'
        pkg_desktop = get_package_share_directory('turtlebot4_slam_desktop')
        rviz2_config = PathJoinSubstitution([pkg_desktop, 'rviz', 'slam.rviz'])
        return [GroupAction([
            PushRosNamespace(namespace),
            Node(package='rviz2',
                 executable='rviz2',
                 name='rviz2',
                 arguments=['-d', rviz2_config],
                 parameters=[{'use_sim_time': use_sim_time == 'true'}],
                 remappings=[
                    ('/tf', 'tf_relay' if tf_relay else 'tf'),
                    ('/tf_static', 'tf_static'),
                 ],
                 output='screen'),
        ])]

    filter_scan = LaunchConfiguration('filter_scan').perform(context) == 'true'
    use_ekf = LaunchConfiguration('use_ekf').perform(context) == 'true'

    namespace_str = namespace.perform(context)
    if namespace_str and not namespace_str.startswith('/'):
        namespace_str = '/' + namespace_str

    pkg_sim = get_package_share_directory('turtlebot4_slam_sim')
    pkg_perception = get_package_share_directory('turtlebot4_slam_perception')
    pkg_desktop = get_package_share_directory('turtlebot4_slam_desktop')

    slam_params = PathJoinSubstitution(
        [get_package_share_directory(cfg['params_pkg']), 'config', cfg['params_file']])

    actions = []

    if mode == 'sim':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_sim, 'launch', 'sim_bringup.launch.py'])),
            launch_arguments=[
                ('namespace', namespace),
                ('world', LaunchConfiguration('world')),
                ('model', LaunchConfiguration('model')),
            ]))

    # filter_scan and use_ekf are independent switches on the same package — include
    # it whenever either is wanted, not just when filter_scan is (otherwise use_ekf on
    # its own, with filter_scan:=false, would never actually launch ekf_node).
    if filter_scan or use_ekf:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_perception, 'launch', 'perception.launch.py'])),
            launch_arguments=[
                ('namespace', namespace),
                ('use_sim_time', use_sim_time),
                ('use_ekf', 'true' if use_ekf else 'false'),
            ]))

    slam_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_desktop, 'launch', 'slam_desktop.launch.py'])),
        launch_arguments=[
            ('namespace', namespace),
            ('params', slam_params),
            ('use_sim_time', use_sim_time),
            ('sync', LaunchConfiguration('sync')),
            ('rviz', LaunchConfiguration('rviz')),
            ('joy_teleop', LaunchConfiguration('joy_teleop')),
        ])

    if filter_scan:
        # slam_toolbox reads its subscribe topic from the scan_topic parameter, but
        # turtlebot4_navigation's own slam.launch.py additionally remaps /scan -> scan
        # unconditionally (its own remappings list, not ours) — so pointing scan_topic
        # at scan_filtered in our params file alone isn't reliable across how that
        # internal remap resolves. Remapping the fully-resolved topic name here instead
        # survives that: it acts on the name slam_toolbox actually ends up connected
        # to, regardless of which internal mechanism produced it. Absolute names,
        # mirroring how turtlebot4_navigation remaps /tf and /tf_static, so this is
        # correct with or without a namespace.
        actions.append(GroupAction([
            SetRemap(namespace_str + '/scan', namespace_str + '/scan_filtered'),
            slam_include,
        ]))
    else:
        actions.append(slam_include)

    return actions


def generate_launch_description():
    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=launch_setup)])
