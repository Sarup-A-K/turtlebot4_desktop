# Path planning — global and local

**Status:** design. Nothing here can be validated until the three maps exist, so every
number below is either measured from the shipped configuration or explicitly marked as
something to measure later.

Companion to [MULTI_MAP_NAV.md](MULTI_MAP_NAV.md). That document covers crossing between
maps; this one covers planning *within* one map, which is what Nav2 does on every leg.

---

## 0. Scope: configure and select, do not reimplement

Nav2 ships several well-tested global planners and controllers, all of them present on
this system. Writing an A\* from scratch would produce something worse than
`nav2_navfn_planner` while consuming the whole project budget, and it runs against the
stated preference for integration and architecture over low-level algorithms.

So the work here is **selection, tuning, and measurement**: choosing the right plugins
for a differential-drive robot in a doorway-heavy building, on a Raspberry Pi that is
already loaded, and knowing *why* each was chosen.

> If a custom planner plugin is wanted later as a learning exercise, the pluginlib
> interface makes it a self-contained addition. It should not be on the critical path to
> a working system.

---

## 1. What ships today, measured

From `turtlebot4_navigation/config/nav2.yaml` on this machine:

| Setting | Value | Note |
|---|---|---|
| Global planner | `nav2_navfn_planner/NavfnPlanner` | `use_astar: false`, so full Dijkstra |
| Planner tolerance | 0.5 m | |
| `allow_unknown` | true | Important, see section 5 |
| Controller | `dwb_core::DWBLocalPlanner` | DWA, seven critics |
| Controller frequency | 20 Hz | |
| `robot_radius` | 0.175 m | 35 cm diameter |
| Costmap resolution | 0.06 m | Map itself is 0.05 m |
| `inflation_radius` | 0.45 m | |
| `cost_scaling_factor` | 4.0 | |
| `max_vel_x` | 0.26 m/s | |
| `max_vel_theta` | 1.0 rad/s | |

One thing to watch: this file sets `use_sim_time: True`. The launch files override it
from the `use_sim_time` argument, so it is normally harmless, but if navigation behaves
as though time has stopped, check this first.

---

## 2. The doorway question, answered with arithmetic rather than folklore

Every map transition in this project happens at a doorway, so it is worth knowing
precisely whether the inflation settings allow the robot through. The usual advice is
"inflation blocks doorways", and with these particular numbers that turns out to be
**false**. Worth checking rather than assuming.

Nav2's inflation layer assigns, for a cell at distance `d` from the nearest obstacle:

```
d <= robot_radius                  ->  253   (inscribed, planner refuses)
robot_radius < d < inflation_radius ->  252 · exp( −cost_scaling_factor · (d − robot_radius) )
d >= inflation_radius              ->  0
```

With `robot_radius = 0.175` and `cost_scaling_factor = 4.0`, at the centre line of a
doorway of width `W`, the distance to each jamb is `W/2`:

| Doorway width | Distance to jamb | Cost at centre | Passable? |
|---|---|---|---|
| 0.90 m | 0.45 m | 71 | Yes, comfortably |
| 0.85 m | 0.425 m | 93 | Yes |
| 0.76 m | 0.38 m | 111 | Yes |
| 0.60 m | 0.30 m | 155 | Yes, but expensive |
| 0.35 m | 0.175 m | 253 | No, and the robot physically does not fit either |

The planner only refuses at 253, which needs the centre line within `robot_radius` of a
wall. That happens at 0.35 m width, which is also the robot's own diameter. **So the
planner will path through any door the robot physically fits through.** Inflation makes
doorways expensive, not forbidden, and since a doorway is the only exit from a room, the
cost does not change the chosen route.

The real doorway difficulty is not the planner. It is the controller, section 4.

---

## 3. Global planner

### 3.1 Recommendation: start on NavFn, evaluate Theta\* once it works

**Keep `NavfnPlanner` for first bring-up.** It is the TurtleBot 4 default, it is what
every tutorial and forum answer assumes, and when something misbehaves early you want the
component that is least likely to be the cause.

**Then try `ThetaStarPlanner`**, which is installed. NavFn paths are constrained to the
costmap grid, so a straight corridor produces a stepped, faintly zigzagging path that the
controller then has to smooth away. Theta\* is any-angle: it produces genuinely straight
segments with far fewer waypoints. In a building that is mostly long corridors, this is
visible in the robot's motion, not just on the screen.

`SmacPlanner2D` is the third option, with a better A\* and a built-in smoother. Worth
trying if planning time turns out to be the constraint.

### 3.2 Dijkstra or A\*

The shipped config sets `use_astar: false`, so NavFn explores the entire reachable
costmap on every plan. A\* would direct the search toward the goal and visit far fewer
cells.

This connects directly to the multi-map rationale. Planning latency is the honest
justification for splitting the maps, and an undirected Dijkstra over a large map is a
large part of that cost. It is worth measuring both before concluding anything:

```bash
ros2 param set /planner_server GridBased.use_astar true
```

NavFn's A\* has a mediocre reputation compared to its Dijkstra, so if A\* produces
visibly worse paths, that is expected. Take the measurement, then decide.

### 3.3 Parameters worth changing

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5          # metres; how close a plan must get if the exact goal is blocked
      use_astar: false        # measure both, see 3.2
      allow_unknown: true     # keep true, see section 5
```

---

## 4. Local controller — the change that matters most

### 4.1 Replace DWB with RotationShim wrapping Regulated Pure Pursuit

This is the highest-value recommendation in this document, and it is specifically because
of the doorways.

DWB samples a velocity space and scores each candidate trajectory with critics. The
shipped configuration uses seven, with `PathAlign` and `PathDist` weighted at 32 and
`BaseObstacle` at only 0.02. Two consequences follow in a doorway:

1. The robot approaches the door at an angle, because a differential-drive robot arcs
   into a path rather than turning to face it. DWB must then find a trajectory that both
   aligns with the path and clears both jambs, and in a 0.8 m gap that candidate set gets
   very thin. The result is hesitation, and sometimes oscillation in the opening.
2. The elevated inflation cost through the doorway, from section 2, feeds `BaseObstacle`
   and mildly penalises exactly the trajectories that are needed.

**Regulated Pure Pursuit** behaves differently. It follows the path by pursuing a
lookahead point, and regulates speed on curvature and on proximity to obstacles. It slows
down in a doorway instead of deliberating about it. It also has far fewer parameters,
which matters when tuning time is limited.

**RotationShim** wraps another controller and rotates the robot in place until it faces
the path before handing over. For a differential-drive robot entering a doorway this is
close to ideal: square up to the opening, then drive straight through. It is a small
wrapper with a large effect on exactly this project's hardest manoeuvre.

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      primary_controller: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      angular_dist_threshold: 0.785      # 45 deg: rotate in place if the path starts further off than this
      forward_sampling_distance: 0.5
      rotate_to_heading_angular_vel: 1.0
      max_angular_accel: 3.2
      simulate_ahead_time: 1.0

      # --- Regulated Pure Pursuit, the primary controller ---
      desired_linear_vel: 0.26           # matches the TurtleBot 4 limit
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      use_velocity_scaled_lookahead_dist: true
      transform_tolerance: 0.2

      # Slow on tight curvature — this is what carries it through doorways
      use_regulated_linear_velocity_scaling: true
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.25

      # Slow near obstacles rather than refusing to proceed
      use_cost_regulated_linear_velocity_scaling: true
      cost_scaling_dist: 0.3
      cost_scaling_gain: 1.0
      inflation_cost_scaling_factor: 4.0   # MUST match the costmap's cost_scaling_factor

      # Differential drive can rotate in place at the goal
      use_rotate_to_heading: true
      rotate_to_heading_min_angle: 0.785
      rotate_to_heading_angular_vel: 1.0
      allow_reversing: false
      max_allowed_time_to_collision_up_to_carrot: 1.0
```

`inflation_cost_scaling_factor` must equal the costmap's `cost_scaling_factor`, which is
4.0 here. Mismatching them makes the controller's idea of obstacle proximity disagree
with the costmap's, and the symptom is speed regulation that feels arbitrary.

### 4.2 Keep the DWB block, commented out

Retain the shipped DWB configuration in the file, commented. If RPP disappoints, the
comparison should be one edit away, and DWB's critic weights are a known-good starting
point that is tedious to reconstruct.

### 4.3 What to keep from the current config

The velocity and acceleration limits are the robot's real constraints and should not
change: `max_vel_x: 0.26`, `max_vel_theta: 1.0`, `acc_lim_x: 2.5`, `acc_lim_theta: 3.2`.

---

## 5. Costmap configuration

The shipped values are reasonable and section 2 shows they do not block doorways. Three
points deserve attention:

**`allow_unknown: true` must stay true.** Each map covers one room, and the overlap
regions near portals contain cells the other map never observed. With `allow_unknown:
false` the planner refuses to route through unknown space, and the failure appears
exactly at portals, which is the worst possible place for it.

**Costmap resolution 0.06 m against map resolution 0.05 m.** Harmless, but matching them
at 0.05 removes a resampling step and slightly sharpens doorway geometry. Costs a little
memory and CPU, which matters on the Pi. Try it only if doorway traversal proves
marginal.

**Clear both costmaps after every map switch.** Covered in
[MULTI_MAP_NAV.md](MULTI_MAP_NAV.md) section 4, repeated here because it is a planning
failure: without it, the obstacle layer retains marks from the room just left, and the
planner routes around obstacles that are no longer anywhere near the robot.

---

## 6. Interaction with the multi-map design

Three couplings between this document and the other one:

**`ComputePathToPose` is a planner call.** The portal-distance precomputation in
MULTI_MAP_NAV section 5.2 asks the global planner for true path lengths without driving
them. Whichever global planner is configured is the one that answers, so those distances
should be measured with the planner that will actually be used.

**Planning cost is the multi-map justification.** Section 3.2's Dijkstra-versus-A\*
measurement bears directly on it. If A\* alone makes planning fast enough on a single
large map, the memory argument was already wrong and the latency argument weakens too.
The honest position is that the *other* benefits, drift containment and localization
robustness, still stand on their own.

**Every leg replans from scratch.** After a map switch the global costmap is new and the
previous path is meaningless. Nothing carries across a portal except the robot's pose.

---

## 7. Measurement plan — before tuning anything

Tuning a planner by intuition wastes days. Once the maps exist, collect these first.

```bash
# 1. Global planning time, per plan, on a real map
ros2 topic echo /plan --field header.stamp    # cadence
# Better: time ComputePathToPose directly
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 1.0}}}}" --feedback

# 2. The same, with A* enabled, for the section 3.2 comparison
ros2 param set /planner_server GridBased.use_astar true

# 3. Pi load while navigating — compare against the 2.17 idle baseline
ssh ubuntu@10.85.219.141 uptime

# 4. Controller output rate; should hold near the 20 Hz controller_frequency
ros2 topic hz /cmd_vel

# 5. Actual costmap cost through each doorway, to check section 2 against reality
ros2 run nav2_costmap_2d costmap_2d_markers   # or inspect /global_costmap/costmap
```

Then tune in this order, because each step changes what the next one sees:

1. Confirm the robot traverses a doorway at all, by hand-setting a goal on the far side.
2. Global planner choice, using the measurements from 1 and 2 above.
3. Controller: RPP lookahead and the regulation parameters.
4. Inflation, and only if doorway traversal is genuinely marginal.

---

## 8. Where this file will live

These parameters belong in `turtlebot4_multimap_nav/config/nav2_tb4.yaml`, created with
that package, overriding `turtlebot4_navigation`'s shipped `nav2.yaml`. The launch file
passes it through the `params_file` argument that `nav2_bringup` already accepts, so no
upstream file is ever edited.

Everything in this document runs **on the Pi**, alongside AMCL and the map server, for
the reason given in MULTI_MAP_NAV section 6: the controller publishes `/cmd_vel` at
20 Hz, and that loop must not cross WiFi.
