# turtlebot4_desktop (fork)

Packages for interacting with TurtleBot 4 from a PC — forked from
[turtlebot/turtlebot4_desktop](https://github.com/turtlebot/turtlebot4_desktop) to add custom
algorithm implementations (SLAM and others) on top of the official packages.

For the full official reference, see the
[TurtleBot 4 User Manual](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_packages.html).

## What's in this repo

- `turtlebot4_desktop/` — desktop tools for interacting with the robot
- `turtlebot4_viz/` — RViz visualization configs and launch files
- Custom algorithm implementations (SLAM, etc.) — **in progress**, added on top of the above

## Prerequisites

- Ubuntu 22.04 with ROS 2 Humble installed
- A TurtleBot 4 (or simulation) reachable on the same network / domain
- `colcon` and the standard ROS 2 dev tools

## Setting up the workspace

```bash
mkdir -p ~/turtlebot4_ws/src
cd ~/turtlebot4_ws/src
git clone -b humble https://github.com/Sarup-A-K/turtlebot4_desktop.git

cd ~/turtlebot4_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Connecting to a real TurtleBot 4

1. Make sure your PC and the robot are on the same network.
2. Set up matching DDS middleware and domain ID (see below) — this is what lets `ros2`
   on your PC actually see the robot's nodes/topics.
3. Verify discovery is working:
   ```bash
   ros2 topic list
   ```
   You should see the robot's topics (e.g. `/odom`, `/scan`, `/battery_state`).
4. Visualize the robot:
   ```bash
   ros2 launch turtlebot4_viz view_robot.launch.py
   ```

See the [User Manual](https://turtlebot.github.io/turtlebot4-user-manual/) for full
network/discovery-server setup if you're running into connectivity issues.

### Middleware setup: Cyclone DDS + ROS_DOMAIN_ID

TurtleBot 4 ships with **Cyclone DDS** and a specific `ROS_DOMAIN_ID` set on the robot
(check the robot's `~/.bashrc` — look for `RMW_IMPLEMENTATION` and `ROS_DOMAIN_ID`, e.g.
via `ssh ubuntu@<robot-ip>`). Your PC needs to match both for reliable discovery.

1. Install the Cyclone DDS RMW on your PC:
   ```bash
   sudo apt update
   sudo apt install -y ros-humble-rmw-cyclonedds-cpp
   ```
2. Add these to your PC's `~/.bashrc` (matching whatever `ROS_DOMAIN_ID` the robot uses —
   `0` is the common default):
   ```bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   export ROS_DOMAIN_ID=0
   ```
3. Re-source and verify:
   ```bash
   source ~/.bashrc
   ros2 topic list                          # robot topics should appear
   ros2 topic echo /battery_state --once    # confirms live data, not just discovery
   ```

Basic pub/sub discovery can sometimes work even with mismatched RMW implementations
(both are DDS under the hood), but matching them avoids subtler QoS/discovery issues —
always set both sides to Cyclone DDS explicitly rather than relying on defaults.

## Custom algorithms (this fork)

This fork adds custom algorithm implementations on top of the stock packages —
starting with a SLAM implementation. Each addition will get its own section here
(what it does, how to launch it, what topics/params it uses) as it lands.

## Contributing

Pull requests and issues on this fork are welcome. This is a personal fork for
algorithm development; for issues with the official packages themselves, use the
[upstream repo](https://github.com/turtlebot/turtlebot4_desktop).
