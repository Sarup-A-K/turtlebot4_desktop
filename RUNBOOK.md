# Runbook — mapping a real room with the TurtleBot 4

Step-by-step operating instructions. For *why* the system is built this way, see
[README.md](README.md); this file is just the sequence to follow.

Architecture in one line: **SLAM runs on the robot, you view it on the laptop.** The
Create 3 binds its DDS to the USB link to the Raspberry Pi, so the laptop can never
receive the base's odometry or drive it directly. See the README's
[Real robot](README.md#real-robot-slam-on-the-pi-viewing-on-the-pc) section.

---

## 0. Before you start

| Check | Why |
|---|---|
| Robot powered on and off the dock | Undocked gives cleaner initial odometry |
| **Lidar plugged in before boot** | The driver probes `/dev/ttyUSB0` once at startup |
| Laptop on the same WiFi as the robot | Both must reach `10.85.219.0/24` |
| `ssh ubuntu@10.85.219.141` works without a password | Run `ssh-copy-id ubuntu@10.85.219.141` once if not |

If you plugged the lidar in *after* powering up, fix it now:

```bash
tb4-pi-lidar-restart
```

It should report `Publisher count: 1`. Anything else means the lidar still isn't up.

---

## 1. Every terminal starts the same way

```bash
cd ~/turtlebot4_ws
source install/setup.bash
source src/turtlebot4_desktop/scripts/aliases.sh
tb4-real
```

Put the two `source` lines in your `~/.bashrc` and you only need `tb4-real` afterwards.

`tb4-real` sets `ROS_DOMAIN_ID=0` and restarts the ROS 2 daemon. **Run it in every
terminal that touches ROS.** A fresh shell has no profile set, and the daemon caches
discovery per domain, so skipping it produces stale topic lists that look exactly like a
network fault.

---

## 2. Start SLAM on the robot — terminal 1

```bash
tb4-pi-deploy        # first time only, or after editing tools/pi/topic_relay.py
tb4-pi-slam-start
```

Expected output:

```
started slam_toolbox on the robot (pid ....)
started /tf -> /tf_relay relay (pid ....)
```

Both run in the background on the Pi, so you get your prompt back. If it says
`topic_relay.py is not on the robot`, run `tb4-pi-deploy` and start again.

---

## 3. Open the viewer — terminal 2

```bash
tb4-view
```

RViz opens after a few seconds. You should see **black** cells for obstacles, **white**
for free space, and the robot model sitting inside the map.

If the map appears but the robot does not, the `/tf` relay is not running. Check
`tb4-pi-slam-log`.

---

## 4. Drive — terminal 3

```bash
tb4-pi-teleop
```

This runs teleop **on the robot** over `ssh -t`. Teleop from the laptop will not move
it: the base's `/cmd_vel` subscriber is behind the USB link and is unreachable from here.

| Key | Action |
|---|---|
| `i` | forward |
| `,` | backward |
| `j` / `l` | rotate left / right |
| `k` | stop |
| `q` / `z` | increase / decrease speed |

Press `k` once or twice first to confirm it is responding before driving.

**Driving for a good map:** go slowly, drive the perimeter of the room first, then
through the middle, and revisit a few places you have already been so loop closure can
tighten the result. Rotating in place at corners helps fill in the walls.

---

## 5. Save the map — before stopping anything

The map exists only in memory. Stopping SLAM without saving loses it.

```bash
ssh ubuntu@10.85.219.141
source /opt/ros/humble/setup.bash
source /etc/turtlebot4/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
exit
```

That writes `my_map.pgm` and `my_map.yaml` on the Pi. Copy them back:

```bash
scp ubuntu@10.85.219.141:~/my_map.* ~/turtlebot4_ws/
```

Run the saver on the Pi rather than the laptop: it needs a reliable `/map`
subscription, and on the Pi that is a local topic.

---

## 6. Shut down

```bash
tb4-pi-slam-stop
```

Then `Ctrl+C` the RViz and teleop terminals.

---

## Troubleshooting

`tb4-pi-slam-log` tails both robot-side logs and is always the first thing to check.

| Symptom | Cause | Fix |
|---|---|---|
| Map renders, **no robot** | `/tf` relay not running | `tb4-pi-slam-log`, then `tb4-pi-slam-start` |
| No map at all, `/scan` has 0 publishers | Lidar driver did not start | `tb4-pi-lidar-restart` |
| Teleop keys do nothing | Running teleop on the laptop instead of the robot | Use `tb4-pi-teleop` |
| `ros2 topic list` shows topics but nothing works | Forgot `tb4-real` in this shell | Run it, it restarts the daemon |
| RViz logs `extrapolation into the future` | Relay's WiFi hop, roughly 100 ms | Cosmetic. SLAM runs on the Pi where TF is local, so map quality is unaffected |
| Two RViz windows or duplicate maps | A previous session still running | `tb4-pi-slam-stop`, close old RViz, start again |

### A note on `ros2 topic list`

It is a poor health check here. A topic is listed if **anyone** publishes *or*
subscribes to it. The base's topics appear on the laptop because the Pi's nodes
subscribe to them, while `ros2 topic info <topic> --verbose` reports
`Publisher count: 0`. Always check publisher counts, not topic names.
