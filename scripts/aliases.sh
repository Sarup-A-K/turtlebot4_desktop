# turtlebot4_desktop shortcuts (ROS 2 Humble, native — no Docker). Source this from
# ~/.bashrc, after sourcing install/setup.bash:
#   source ~/turtlebot4_ws/install/setup.bash
#   source ~/turtlebot4_ws/src/turtlebot4_desktop/scripts/aliases.sh
#
# Ported from this project's Jazzy/Docker sibling (tb4_slam/.devcontainer/aliases.sh).
# Two things differ deliberately, not by oversight:
#   - No container, so no wsb/wss build+source shortcut in quite the same form — build
#     with `cb` below, which just wraps colcon build from the workspace root regardless
#     of your current directory.
#   - Ignition Fortress (Humble's simulator, not Gazebo Harmonic) uses IGN_PARTITION/
#     IGN_IP, not GZ_PARTITION/GZ_IP — and Humble has no ROS_AUTOMATIC_DISCOVERY_RANGE,
#     so sim/real isolation uses ROS_LOCALHOST_ONLY instead. See tb4-sim below and
#     turtlebot4_slam_sim/launch/sim_bringup.launch.py's module docstring.

TB4_WS=~/turtlebot4_ws
TB4_DESKTOP_SRC="$TB4_WS/src/turtlebot4_desktop"

alias cb="cd $TB4_WS && colcon build --symlink-install && source $TB4_WS/install/setup.bash"

# Original hardware-only entry point. Still works unchanged — it launches
# turtlebot4_slam_desktop directly and knows nothing about sim mode.
alias tb4-launch='ros2 launch turtlebot4_slam_desktop slam_desktop.launch.py'
alias tb4-launch-headless='ros2 launch turtlebot4_slam_desktop slam_desktop.launch.py rviz:=false'
alias tb4-teleop='ros2 run teleop_twist_keyboard teleop_twist_keyboard'

# Unified sim/real entry point (turtlebot4_slam_bringup). Prefer these for new work —
# tb4-sim/tb4-real below must be run first, in *this* shell, so the discovery env vars
# are set before ros2 launch reads them.
alias tb4-sim-launch='ros2 launch turtlebot4_slam_bringup tb4_slam.launch.py mode:=sim'
alias tb4-real-launch='ros2 launch turtlebot4_slam_bringup tb4_slam.launch.py mode:=real'

# Laptop-side VIEWER ONLY — RViz, no slam_toolbox, no scan filter. Use this when SLAM
# is running on the robot itself (tb4-pi-slam-start below). Running tb4-real-launch or
# tb4-launch at the same time would start a second slam_toolbox here and the two would
# fight over the map->odom transform.
#
# tf_relay:=true reads TF from the Pi-hosted /tf_relay that tb4-pi-slam-start runs —
# without it RViz draws the map but not the robot ("two or more unconnected trees"),
# because the Create 3 publishes odom->base_link only over its USB link to the Pi and
# that transform never reaches this machine (measured, including with the base as an
# explicit CycloneDDS unicast peer).
alias tb4-view='ros2 launch turtlebot4_slam_bringup tb4_slam.launch.py mode:=view tf_relay:=true'

# ── SLAM on the robot (Pi-side) ──────────────────────────────────────────────
# Why this exists: the Create 3's DDS is bound to its USB link to the Pi, so the laptop
# never sees /odom, /imu, or the base's odom->base_link — the Pi sees all of them
# perfectly. Running slam_toolbox ON the Pi keeps every time-critical input (/scan,
# /odom, /tf) local to the robot; only the small, low-rate /map and the map->odom
# transform cross Wi-Fi to RViz here.
#
# Robot IP comes from TB4_ROBOT_IP. These use SSH key auth (run `ssh-copy-id
# ubuntu@$TB4_ROBOT_IP` once) — no password is stored in this file.
: "${TB4_ROBOT_IP:=10.85.219.141}"

# The env every Pi-side ROS command needs. Sourcing /etc/turtlebot4/setup.bash is what
# sets CYCLONEDDS_URI to the robot's own profile; without it the Pi can't see the
# Create 3 over usb0 either.
TB4_PI_ENV='source /opt/ros/humble/setup.bash; source /etc/turtlebot4/setup.bash 2>/dev/null'

# One-time: copy the relay script to the robot. Re-run after editing it.
tb4-pi-deploy() {
  scp "$TB4_DESKTOP_SRC/tools/pi/topic_relay.py" ubuntu@"$TB4_ROBOT_IP":/home/ubuntu/topic_relay.py \
    && echo "deployed topic_relay.py to the robot"
}

# Restart the robot's bringup. Needed after plugging the lidar in: the rplidar driver
# only probes /dev/ttyUSB0 at startup, so a lidar connected afterwards stays dead with
# /scan at 0 publishers and diagnostics reporting "Lidar: Error".
tb4-pi-lidar-restart() {
  ssh -t ubuntu@"$TB4_ROBOT_IP" "sudo systemctl restart turtlebot4.service && echo restarted"
  echo "waiting 20s for bringup..."; sleep 20
  ssh ubuntu@"$TB4_ROBOT_IP" "bash -lc '$TB4_PI_ENV; timeout 8 ros2 topic info /scan --verbose | grep -E \"Publisher count\"'"
}

# Starts two things on the robot, each pidfile-guarded so re-running is safe:
#   1. slam_toolbox (turtlebot4_navigation's stock slam.launch.py, async).
#   2. topic_relay.py republishing /tf as /tf_relay — needs tb4-pi-deploy once.
# Stock slam.yaml on purpose: this project's slam_offboard.yaml tuning compensates for a
# Wi-Fi hop between sensor and SLAM, and that hop doesn't exist when SLAM runs on the robot.
tb4-pi-slam-start() {
  ssh ubuntu@"$TB4_ROBOT_IP" "bash -lc '
    $TB4_PI_ENV
    if [ -f /tmp/slam_pi.pid ] && kill -0 \$(cat /tmp/slam_pi.pid) 2>/dev/null; then
      echo \"slam_toolbox already running on the robot (pid \$(cat /tmp/slam_pi.pid))\"
    else
      nohup ros2 launch turtlebot4_navigation slam.launch.py sync:=false > /tmp/slam_pi.log 2>&1 < /dev/null &
      echo \$! > /tmp/slam_pi.pid
      echo \"started slam_toolbox on the robot (pid \$!)\"
    fi
    if [ ! -f /home/ubuntu/topic_relay.py ]; then
      echo \"topic_relay.py is not on the robot — run tb4-pi-deploy once, then tb4-pi-slam-start again\"
      exit 0
    fi
    if [ -f /tmp/tf_relay.pid ] && kill -0 \$(cat /tmp/tf_relay.pid) 2>/dev/null; then
      echo \"/tf relay already running (pid \$(cat /tmp/tf_relay.pid))\"
    else
      nohup python3 /home/ubuntu/topic_relay.py /tf /tf_relay tf2_msgs/msg/TFMessage > /tmp/tf_relay.log 2>&1 < /dev/null &
      echo \$! > /tmp/tf_relay.pid
      echo \"started /tf -> /tf_relay relay (pid \$!)\"
    fi
    echo \"logs: tb4-pi-slam-log\"
  '"
}

tb4-pi-slam-stop() {
  ssh ubuntu@"$TB4_ROBOT_IP" "bash -c '
    for name in slam_pi tf_relay; do
      if [ -f /tmp/\$name.pid ] && kill -0 \$(cat /tmp/\$name.pid) 2>/dev/null; then
        kill -INT \$(cat /tmp/\$name.pid) && echo \"stopping \$name on the robot\"
      else
        echo \"\$name not running\"
      fi
      rm -f /tmp/\$name.pid
    done
  '"
}

tb4-pi-slam-log() {
  ssh ubuntu@"$TB4_ROBOT_IP" 'tail -n 20 /tmp/slam_pi.log /tmp/tf_relay.log'
}

# Keyboard teleop running ON the robot, over ssh -t (a real TTY, so keys register).
# This is the one that actually drives it: the Create 3's /cmd_vel subscriber lives
# behind the USB link, so teleop from this machine publishes into the void
# (tb4-check shows Subscription count: 0).
tb4-pi-teleop() {
  ssh -t ubuntu@"$TB4_ROBOT_IP" "bash -lc '
    $TB4_PI_ENV
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
  '"
}

# ── Sim/real network profiles ───────────────────────────────────────────────
# Run exactly one of these in every shell that will touch ROS, including a second/
# teleop terminal — a fresh shell has no discovery scoping at all by default, which
# on this network means a "sim" run's traffic is visible to (and from) the real robot.
#
# `ros2 daemon stop` matters here: the ROS 2 daemon caches discovery per domain, so
# without restarting it, `ros2 topic list` etc. keep reporting the PREVIOUS profile's
# topics after switching — which reads exactly like a network fault rather than a
# stale daemon.
tb4-sim() {
  export ROS_DOMAIN_ID=42
  export ROS_LOCALHOST_ONLY=1
  # Ignition Transport does its own peer discovery, entirely separate from ROS 2/DDS —
  # a correct ROS_DOMAIN_ID alone does not stop a sim instance finding another one on
  # the network. IGN_PARTITION scopes it to this user, IGN_IP pins it to loopback.
  export IGN_PARTITION="tb4slam_${USER}"
  export IGN_IP=127.0.0.1
  # Hybrid Intel-iGPU + NVIDIA laptop (this dev machine): without PRIME offload the
  # Ignition server's sensor render context lands on the Intel/Mesa path, EGL fails
  # ("failed to create dri2 screen"), and the GPU lidar returns range_min for every
  # beam. slam_toolbox then silently discards all 640 points per scan — no map, no
  # warning. Confirmed directly: nvidia-smi showed no gazebo process before, and the
  # lidar read 0.164 m in all directions; with these two vars the server appears on
  # the GPU and the scan reads real geometry (1–12 m). Harmless on single-GPU boxes.
  export __NV_PRIME_RENDER_OFFLOAD=1
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
  # Raises CycloneDDS's MaxAutoParticipantIndex — mode:=sim launches 30+ nodes on one
  # domain and, without this, hits "Failed to find a free participant index" and a
  # node dies (SIGABRT) — confirmed directly while building this. mode:=real (~10
  # nodes) stays under the default and is unaffected.
  export CYCLONEDDS_URI="file://$TB4_DESKTOP_SRC/scripts/cyclonedds.xml"
  ros2 daemon stop >/dev/null 2>&1
  echo "profile: SIM   domain=$ROS_DOMAIN_ID  localhost_only=1  ign_partition=$IGN_PARTITION"
}

tb4-real() {
  # Must match the robot's ROS_DOMAIN_ID (see the README's Middleware setup section).
  export ROS_DOMAIN_ID=0
  unset ROS_LOCALHOST_ONLY IGN_PARTITION IGN_IP CYCLONEDDS_URI
  unset __NV_PRIME_RENDER_OFFLOAD __GLX_VENDOR_LIBRARY_NAME
  ros2 daemon stop >/dev/null 2>&1
  echo "profile: REAL  domain=$ROS_DOMAIN_ID"
}

# Self-check for the "PC thinks, robot executes" split — run this with the robot
# powered on and slam_desktop.launch.py (or tb4-real-launch) already running.
#
# What it actually proves: which nodes are running on THIS machine, and which node
# publishes/subscribes /cmd_vel. It does NOT and CANNOT prove what's running on the
# Pi — ros2 CLI has no concept of "which machine" a node lives on. Separately SSH to
# the robot and run `ros2 node list` there: it must show no slam_toolbox and no
# teleop node.
tb4-check() {
  echo "── Nodes running on THIS machine ──"
  ros2 node list
  echo
  echo "── /cmd_vel: who publishes, who subscribes ──"
  ros2 topic info /cmd_vel --verbose
  echo
  echo "── /scan liveness (confirms the robot is actually reachable) ──"
  timeout 3 ros2 topic hz /scan || echo "(no /scan seen in 3s — robot not on the network, or discovery isn't up yet)"
  echo
  echo "This only checked this machine. Now SSH to the robot and confirm"
  echo "'ros2 node list' there shows no slam_toolbox and no teleop node."
}
