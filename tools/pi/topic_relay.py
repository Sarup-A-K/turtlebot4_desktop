#!/usr/bin/env python3
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
"""Minimal topic relay: re-publish everything from IN_TOPIC on OUT_TOPIC.

Runs on the TurtleBot 4's Raspberry Pi (ROS 2 Humble), not the laptop. It is NOT a
ROS package — it's a single file, deployed with `tb4-pi-deploy` (scp) and started by
`tb4-pi-slam-start` (see .devcontainer/aliases.sh).

Why it exists: over WiFi the laptop cannot hold a stable DDS view of the Create 3's
own participant (its odom->base_link on /tf, and its /cmd_vel subscriber, come and go),
but it discovers the Pi's participants reliably. Relaying the Create 3's /tf through
a Pi-hosted node gives the laptop's RViz a dependable odom->base_link — the transform
the robot model and the scan pose need — without touching the Create 3 at all.
`topic_tools relay` would do the same job, but it isn't installed on the robot image.

NEVER relay a topic onto itself (e.g. /tf -> /tf): every message this node publishes
would be received by its own subscription and re-published — an infinite feedback loop
that also floods every other subscriber on the Pi. Relay to a *distinct* topic and
remap the consumer (RViz: `mode:=view tf_relay:=true`) onto it.

Usage:
  python3 topic_relay.py IN_TOPIC OUT_TOPIC MSG_TYPE [--depth N] [--ros-args ...]
  python3 topic_relay.py /tf /tf_relay tf2_msgs/msg/TFMessage

QoS is RELIABLE / KEEP_LAST(depth) on both sides — matching every /tf publisher on the
robot (Create 3, robot_state_publisher, slam_toolbox's broadcaster all use RELIABLE).
A BEST_EFFORT publisher would not match this subscription; adjust --depth, not the
reliability, if you ever relay one of those.
"""
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message


class TopicRelay(Node):

    def __init__(self, in_topic, out_topic, msg_type, depth):
        super().__init__('topic_relay_' + out_topic.strip('/').replace('/', '_'))
        msg_cls = get_message(msg_type)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST,
                         depth=depth)
        self._pub = self.create_publisher(msg_cls, out_topic, qos)
        # The publisher's publish() is the whole callback: forward as-is, untouched.
        self._sub = self.create_subscription(msg_cls, in_topic, self._pub.publish, qos)
        self.get_logger().info(f'relaying {in_topic} -> {out_topic} ({msg_type})')


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('in_topic')
    parser.add_argument('out_topic')
    parser.add_argument('msg_type', help='e.g. tf2_msgs/msg/TFMessage')
    parser.add_argument('--depth', type=int, default=100,
                        help='QoS history depth, both sides (default: 100, like tf2)')
    args, ros_args = parser.parse_known_args()
    if args.in_topic == args.out_topic:
        parser.error('in_topic and out_topic must differ — relaying a topic onto '
                     'itself loops forever (see module docstring)')

    rclpy.init(args=ros_args)
    node = TopicRelay(args.in_topic, args.out_topic, args.msg_type, args.depth)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
