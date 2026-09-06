# turtlebot4_desktop (fork)

Packages for interacting with TurtleBot 4 from a PC — forked from
[turtlebot/turtlebot4_desktop](https://github.com/turtlebot/turtlebot4_desktop) to add custom
algorithm implementations (SLAM and others) on top of the official packages.

For the full official reference, see the
[TurtleBot 4 User Manual](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_packages.html).

## What's in this repo

- `turtlebot4_desktop/` — desktop tools for interacting with the robot
- `turtlebot4_viz/` — RViz visualization configs and launch files
- `turtlebot4_slam_desktop/` — offboard SLAM: runs `slam_toolbox` and RViz on your PC
  against a networked robot (see [Offboard SLAM](#offboard-slam) below)
- More custom algorithm implementations, added on top of the above, land here over time

## Architecture: the PC thinks, the robot executes

This fork moves everything that *decides* onto your PC and leaves the robot's
Raspberry Pi 4 doing only what it does out of the box — sensor drivers in, motor
commands out. Nothing runs on the Pi that isn't part of stock TurtleBot 4 bringup.

- **Robot (Raspberry Pi 4)** — stock bringup only. Publishes `/scan` (RPLIDAR), `/odom`
  and `/tf` (Create 3 base); subscribes to `/cmd_vel` and turns it into wheel motion.
  It runs no SLAM and originates no motion commands of its own.
- **PC** — everything else. `slam_toolbox` (via `turtlebot4_slam_desktop`) consumes
  `/scan` and `/tf` and publishes the `map` → `odom` transform. RViz visualizes it.
  **Teleop also runs here** — a keyboard or joystick node on the PC is what actually
  generates `/cmd_vel`.

What crosses the network: `/scan`, `/odom`, `/tf` flow PC-ward from the robot;
`/cmd_vel` flows robot-ward from the PC. Both directions ride plain ROS 2/DDS discovery
over a matching `ROS_DOMAIN_ID` and RMW implementation — no bridge, no extra
infrastructure. It's one ROS graph that happens to span two machines.

**On the robot**, this means: don't launch SLAM, and don't launch teleop there either.
Two `/cmd_vel` publishers in one graph will fight each other, and the symptom
(stuttering, ignored commands) reads like a network problem rather than a duplicate
publisher — so the rule is simply "no teleop, no SLAM, on the Pi, ever."

## Prerequisites

- Ubuntu 22.04 with ROS 2 Humble installed
- A TurtleBot 4 (or simulation) reachable on the same network / domain
- `colcon` and the standard ROS 2 dev tools

## Setting up the workspace

```bash
mkdir -p ~/turtlebot4_ws/src
cd ~/turtlebot4_ws/src
git clone -b feature/slam-impl https://github.com/Sarup-A-K/turtlebot4_desktop.git

cd ~/turtlebot4_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Add `source ~/turtlebot4_ws/install/setup.bash` to your `~/.bashrc`, next to the
`RMW_IMPLEMENTATION` / `ROS_DOMAIN_ID` exports below — otherwise `ros2 launch` can't
find these packages in any new shell.

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

### Clock synchronization

`slam_toolbox` on your PC stamps incoming scans against your PC's clock while `/tf`
arrives stamped with the robot's. If the two clocks drift apart, every transform lookup
fails with `extrapolation into the future` and the map never builds — with no obvious
hint that the clock is the actual cause.

Check both machines are NTP-synced:

```bash
timedatectl status | grep "synchronized"   # should say "System clock synchronized: yes"
```

If the robot has no reliable time source, install `chrony` on it (`sudo apt install
chrony`) and confirm it syncs. **Treat any TF extrapolation error during bring-up as a
clock problem first** — it is one of the most common false leads when offboard SLAM
looks broken but isn't.

## Offboard SLAM

`turtlebot4_slam_desktop` runs `slam_toolbox` and RViz on your PC against a robot that
is only publishing sensor data and executing `/cmd_vel` — see
[Architecture](#architecture-the-pc-thinks-the-robot-executes) above for the full split.

It wraps `turtlebot4_navigation`'s stock `slam.launch.py` with a network-tuned params
file (`config/slam_offboard.yaml`) instead of the default one. Four values differ from
stock, each because the link between your PC and the robot is WiFi, not a wired LAN:

| Param | Stock | Offboard | Why |
|---|---|---|---|
| `transform_timeout` | `0.2` | `0.5` | WiFi latency on `/tf` can exceed 200 ms under load. |
| `map_update_interval` | `0.5` | `2.0` | `/map` is a full OccupancyGrid; republishing it at 2 Hz is the single biggest bandwidth consumer here. |
| `minimum_time_interval` | `0.25` | `0.5` | Fewer, better-spaced scans processed; less sensitive to WiFi jitter. |
| `tf_buffer_duration` | `30.` | `30.` | Unchanged — already generous enough. |

### Launching

```bash
# Terminal 1 — mapping + viz, on your PC
ros2 topic hz /scan                                 # confirm live data first
ros2 launch turtlebot4_slam_desktop slam_desktop.launch.py

# Terminal 2 — control, also on your PC
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Keyboard teleop is deliberately **not** part of the launch file — `teleop_twist_keyboard`
reads the terminal's stdin, and a node started by `ros2 launch` doesn't own a TTY. Run it
by hand in its own terminal, as above. If you use a gamepad instead, pass
`joy_teleop:=true` to the launch file and skip the second terminal.

Arguments: `namespace` (default `''`), `sync` (default `false` — async slam_toolbox
tolerates WiFi's irregular scan arrival better than sync), `rviz` (default `true`),
`joy_teleop` (default `false`), `params` (default `config/slam_offboard.yaml`),
`use_sim_time` (default `false`).

### Prerequisites on the robot

1. Robot's `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` match your PC's — this is what lets
   `/cmd_vel` reach the robot at all (see [Middleware setup](#middleware-setup-cyclone-dds--ros_domain_id)).
2. Leave standard bringup running; don't start SLAM or teleop on the robot itself
   (see [Architecture](#architecture-the-pc-thinks-the-robot-executes)).
3. Optional but the biggest compute/bandwidth win: disable the OAK-D via
   `turtlebot4-setup` if you're not using it. SLAM here only needs `/scan`, `/odom`,
   and `/tf`.

## Contributing

Pull requests and issues on this fork are welcome. This is a personal fork for
algorithm development; for issues with the official packages themselves, use the
[upstream repo](https://github.com/turtlebot/turtlebot4_desktop).
