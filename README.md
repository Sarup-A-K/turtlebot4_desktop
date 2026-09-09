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
- `turtlebot4_slam_sim/` — Ignition Fortress simulation bringup (see
  [Simulation mode](#simulation-mode))
- `turtlebot4_slam_perception/` — shared `/scan` filtering + optional EKF, used by
  both sim and real modes
- `turtlebot4_slam_bringup/` — unified `mode:=sim`/`real`/`view` entry point tying the
  above together
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

This split is unaffected by simulation mode below — `mode:=sim` replaces the *robot*,
not the PC. `slam_toolbox` and RViz are the same nodes, reading the same `/scan`/`/tf`,
whether those topics come from the Pi or from Ignition.

## Simulation mode

`turtlebot4_slam_bringup` is a single entry point that runs the exact same SLAM + RViz
stack against either an Ignition Fortress simulation or the physical robot, selected
with one argument:

```bash
tb4-sim   # sets up the sim network profile — see "Sim/real network isolation" below
ros2 launch turtlebot4_slam_bringup tb4_slam.launch.py mode:=sim
```

```bash
tb4-real  # sets up the real-robot network profile
ros2 launch turtlebot4_slam_bringup tb4_slam.launch.py mode:=real
```

`mode:=sim` spawns the TurtleBot 4 in Ignition Fortress (`turtlebot4_slam_sim`, built on
`turtlebot4_ignition_bringup`) inside a `warehouse`/`depot`/`maze` world (`world:=`),
runs the shared `/scan` filter chain, and launches `turtlebot4_slam_desktop`'s SLAM +
RViz layer with `use_sim_time:=true` and a stock (untuned) SLAM params file — there's no
network hop to tune for. `mode:=real` launches no robot-side nodes at all — same
architecture as [above](#architecture-the-pc-thinks-the-robot-executes) — and uses the
existing WiFi-tuned `slam_offboard.yaml`. `use_sim_time` is derived from `mode`, not a
separate argument, so the two can't be set inconsistently — see the comment above
`MODES` in `turtlebot4_slam_bringup/launch/tb4_slam.launch.py` for the (previously hit,
now-avoided) `LaunchConfiguration` bug this dodges.

**Give the controllers ~10 s after launch before driving.** `diffdrive_controller` is
loaded and activated by a spawner a few seconds after the world comes up; `/cmd_vel`
sent before that is silently ignored (the topic has a subscriber, the robot just doesn't
move — confirmed while testing). If teleop seems dead right after launch, wait, or check
`ros2 control list_controllers` shows `diffdrive_controller ... active`.

`turtlebot4_slam_desktop`/`tb4-launch` (the original hardware-only entry point) is
untouched and still works exactly as documented below — `tb4_slam.launch.py` is
additive, not a replacement.

**Two Gazebo/Ignition slowness patterns to know about, so neither reads as a hang:**

1. **First launch of `world:=warehouse`** references several dozen furniture props
   hosted on Gazebo Fuel, downloaded and cached under `~/.gz/fuel/` the first time
   they're used. Expect "Requesting list of world names. The server may be busy
   downloading resources" for a few minutes on a cold cache. `world:=maze` and
   `world:=depot` reference no Fuel content at all (verified: `grep fuel.gazebosim.org
   maze.sdf` returns nothing) and don't hit this.
2. **The very first `ign gazebo` launch in a session can also sit at that same message
   for several minutes — even on `world:=maze`.** Observed directly while building this:
   identical launches that hung for 2+ minutes on a first attempt started ticking
   within seconds on later attempts, and in one case the slow run coincided with an
   entirely unrelated Gazebo-based simulation (a different project, same machine)
   competing for the same GPU. If a launch seems stuck, check `ros2 topic echo /clock
   --once` before assuming something is broken — a real hang shows no output at all; a
   slow-but-working sim returns a tick with a nonzero `sec` field. Also check nothing
   else on the machine is contending for the GPU.

### Sim/real network isolation

Two shell functions in `scripts/aliases.sh` are what actually keep a simulation's
traffic separate from the real robot's — **run one of them in every shell that touches
ROS**, including a second teleop terminal, since a fresh shell has no discovery scoping
at all by default:

| | `tb4-sim` | `tb4-real` |
|---|---|---|
| `ROS_DOMAIN_ID` | `42` | `0` (must match the robot) |
| `ROS_LOCALHOST_ONLY` | `1` | unset |
| Ignition Transport | `IGN_PARTITION`/`IGN_IP` pinned to loopback | — |

Note the env var names if you've read the Jazzy/Docker sibling project's equivalent
(`tb4_slam`): Humble's simulator is **Ignition Fortress**, which predates the Gazebo
rename, so it's `IGN_PARTITION`/`IGN_IP` here — `GZ_PARTITION`/`GZ_IP` is Harmonic-era
naming and does nothing on this distro. Likewise Humble has no
`ROS_AUTOMATIC_DISCOVERY_RANGE` (that's Iron+), so isolation uses the older
`ROS_LOCALHOST_ONLY` instead — both confirmed by reading what
`turtlebot4_ignition_bringup` itself actually sets.

Both functions run `ros2 daemon stop`, because the ROS 2 daemon caches discovery per
domain: without restarting it, `ros2 topic list` etc. keep reporting the *previous*
profile's topics after switching, which reads like a network fault rather than a stale
daemon. `tb4-check` prints the active profile first for exactly this reason.

### GPU rendering on hybrid Intel/NVIDIA laptops

Ignition's lidar is a **GPU** sensor — it raycasts through an OGRE render context. On a
laptop with both an Intel iGPU and an NVIDIA dGPU (this project's dev machine: Intel +
RTX 5050, Wayland), a plain `ros2 launch` puts that context on the Intel/Mesa path, EGL
fails with `libEGL warning: egl: failed to create dri2 screen`, and the GPU lidar
returns `range_min` for **every** beam. `slam_toolbox` then does exactly the right thing
with 640 sub-minimum points per scan — discards them all — so you get a map stuck at
its 7×7 initial size, the robot driving around fine, and **no warning anywhere**. It is
one of the quietest failures in this whole stack.

`tb4-sim` fixes it by exporting `__NV_PRIME_RENDER_OFFLOAD=1` and
`__GLX_VENDOR_LIBRARY_NAME=nvidia`, which pins the render context to the NVIDIA GPU.
Harmless on single-GPU machines. Two ways to confirm it took, in order of speed:

```bash
nvidia-smi | grep gazebo          # 'ign gazebo server' must be listed — if absent, this is your problem
ros2 topic echo /scan --once --full-length | grep -c "^- 0.16"   # ~0 healthy; ~640 = all beams at range_min
```

If a `mode:=sim` map never grows but the robot moves and nothing logs an error, check
this before anything else.

### CycloneDDS participant limit

`mode:=sim` launches 30+ ROS nodes on one domain — `ros_gz_bridge` spawns one
`parameter_bridge` per sensor/topic (lidar, camera, every IR/cliff sensor, HMI, docking,
etc.), plus `robot_state_publisher` ×2, `controller_manager`, `slam_toolbox`, the scan
filter, and more. CycloneDDS's default `MaxAutoParticipantIndex` is too low for that —
confirmed directly while building this: past it, every `parameter_bridge` (and several
other nodes) logged `Failed to find a free participant index for domain 0` and then died
with SIGABRT within seconds of launch. `tb4-sim` fixes this by pointing `CYCLONEDDS_URI`
at `scripts/cyclonedds.xml`, which raises the limit to 200 — already wired up, no action
needed as long as you run `tb4-sim` first. `mode:=real` (~10 nodes) stays under the
default and is unaffected, which is why this never surfaced before simulation mode
existed.

## Prerequisites

- Ubuntu 22.04 with ROS 2 Humble installed
- A TurtleBot 4 (or simulation) reachable on the same network / domain
- `colcon` and the standard ROS 2 dev tools
- Two packages this repo depends on but does **not** install for you (see
  [Dependencies](#dependencies)):
  ```bash
  sudo apt install ros-humble-laser-filters ros-humble-robot-localization
  ```

## Dependencies

**Installing ROS 2 itself:** [docs.ros.org/en/humble/Installation](https://docs.ros.org/en/humble/Installation.html)
(reference only — if you're reading this you likely already have it, since this repo
targets a specific existing TurtleBot 4 setup rather than bootstrapping one from
nothing).

Everything else is one `apt install ros-humble-<name>` per row. Most of these come
with a standard TurtleBot 4 setup already; the reference links go to each package's
ROS index page (source repo, maintainer, every supported distro):

| Package | What it's for | Already installed? |
|---|---|---|
| [`turtlebot4-navigation`](https://index.ros.org/p/turtlebot4_navigation/) | `slam.launch.py`, stock `slam.yaml` — this repo includes/tunes both | Standard TB4 setup |
| [`turtlebot4-description`](https://index.ros.org/p/turtlebot4_description/) | Robot model for RViz | Standard TB4 setup |
| [`turtlebot4-viz`](https://index.ros.org/p/turtlebot4_viz/) | `navigation.rviz`/`robot.rviz`, source of this repo's `slam.rviz` | Standard TB4 setup |
| [`turtlebot4-simulator`](https://index.ros.org/p/turtlebot4_simulator/) | `turtlebot4_ignition_bringup` — Ignition Fortress spawn/bridge, simulation mode only | Standard TB4 setup |
| [`slam-toolbox`](https://index.ros.org/p/slam_toolbox/) | The actual SLAM implementation everything here wraps | Standard TB4 setup |
| [`rmw-cyclonedds-cpp`](https://index.ros.org/p/rmw_cyclonedds_cpp/) | The RMW this whole project's network setup assumes | Required, see [Middleware setup](#middleware-setup-cyclone-dds--ros_domain_id) |
| [`teleop-twist-keyboard`](https://index.ros.org/p/teleop_twist_keyboard/) / [`teleop-twist-joy`](https://index.ros.org/p/teleop_twist_joy/) | Driving the robot | Standard TB4 setup |
| [`laser-filters`](https://index.ros.org/p/laser_filters/) | `turtlebot4_slam_perception`'s `/scan` → `/scan_filtered` chain | **No — install yourself, see Prerequisites** |
| [`robot-localization`](https://index.ros.org/p/robot_localization/) | Opt-in EKF (`use_ekf:=true`) — see `turtlebot4_slam_perception/config/ekf.yaml` | **No — install yourself, see Prerequisites** |

Exact versions aren't pinned here on purpose — `apt install ros-humble-<name>` always
resolves whatever's current in your configured ROS 2 apt repository; run `apt list
--installed | grep ros-humble` on your own machine for the source of truth.

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

Add these two lines to your `~/.bashrc`, next to the `RMW_IMPLEMENTATION` /
`ROS_DOMAIN_ID` exports below — otherwise `ros2 launch` can't find these packages in
any new shell, and the `tb4-*` shortcuts (see [Shortcuts](#shortcuts)) won't exist:

```bash
source ~/turtlebot4_ws/install/setup.bash
source ~/turtlebot4_ws/src/turtlebot4_desktop/scripts/aliases.sh
```

## Shortcuts

Loaded by the `scripts/aliases.sh` line above, in every new shell:

| Command | Does |
|---|---|
| `cb` | `cd` to the workspace root, `colcon build --symlink-install`, source the result. The one to reach for after editing anything, from any directory. |
| `tb4-sim` | Set the sim network profile in this shell — see [Sim/real network isolation](#simreal-network-isolation). Run before anything sim-related. |
| `tb4-real` | Set the real-robot network profile in this shell. Run before anything robot-related. |
| `tb4-sim-launch` | `ros2 launch turtlebot4_slam_bringup tb4_slam.launch.py mode:=sim` |
| `tb4-real-launch` | `ros2 launch turtlebot4_slam_bringup tb4_slam.launch.py mode:=real` |
| `tb4-view` | `… mode:=view` — RViz only, no SLAM on this machine, for the Pi-side-SLAM fallback case (see the Jazzy sibling project if you ever need it here). |
| `tb4-launch` | Original hardware-only entry point: `ros2 launch turtlebot4_slam_desktop slam_desktop.launch.py` (with RViz) |
| `tb4-launch-headless` | Same, `rviz:=false` |
| `tb4-teleop` | `ros2 run teleop_twist_keyboard teleop_twist_keyboard` |
| `tb4-check` | Self-check for the PC/robot split, prints the active network profile first — see [Offboard SLAM](#offboard-slam) |

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
