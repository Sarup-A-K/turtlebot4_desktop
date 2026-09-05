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
2. Set a matching `ROS_DOMAIN_ID` on both the PC and the robot (defaults to `0`; check your
   robot's configuration if it's been changed).
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

## Custom algorithms (this fork)

This fork adds custom algorithm implementations on top of the stock packages —
starting with a SLAM implementation. Each addition will get its own section here
(what it does, how to launch it, what topics/params it uses) as it lands.

## Contributing

Pull requests and issues on this fork are welcome. This is a personal fork for
algorithm development; for issues with the official packages themselves, use the
[upstream repo](https://github.com/turtlebot/turtlebot4_desktop).
