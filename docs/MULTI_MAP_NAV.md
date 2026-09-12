# Dynamic Multi-Map Navigation — architecture proposal

**Status:** design only. No implementation yet, pending the open questions at the end.

Goal: navigate across several small maps instead of one large one, loading each map only
when needed. The robot drives to a boundary waypoint in Map A, the system swaps to Map B
via the Nav2 Map Server, localization re-seeds in B's frame, and navigation continues to
a goal in B.

Planning *within* a single map — global planner and local controller selection and
tuning — is covered separately in [PATH_PLANNING.md](PATH_PLANNING.md).

## 0. Agreed scope and rationale

**Scale: three maps, three portals, forming a cycle.** The corridor wraps around the lab
and is mapped as two halves, joined at one end.

```
      left_corridor ------------- right_corridor
            |            join           |
            |        +---------+        |
            +--------|   lab   |--------+
               west  +---------+  east
                        door        door
```

| Portal | Joins | Site |
|---|---|---|
| `lab__left` | lab, left_corridor | west doorway |
| `lab__right` | lab, right_corridor | east doorway |
| `left__right` | left_corridor, right_corridor | the corridor join |

This is a triangle, and it is the smallest graph that presents a genuine routing choice.
From the lab there are **two distinct routes to right_corridor**: straight out the east
door, or out the west door and the long way round through the join. Which one is shorter
depends on where in right_corridor the goal sits, so the router has real work to do. That
makes this a better demonstration of the architecture than a larger but purely
hierarchical layout would be.

It also unlocks a loop-closure consistency check that a chain cannot provide. See
section 3.3.

**A note on the `left__right` portal site.** Doorways make good portals because they are
geometrically distinctive, which constrains AMCL tightly. An arbitrary point in open
corridor is the opposite: featureless, with both sides of the overlap looking alike, and
that is exactly what makes a particle filter pick the wrong hypothesis. **Site this
portal at a corner or a junction if at all possible**, not in the middle of a straight
run. If the only option is a featureless stretch, expect slower convergence there and
keep the generous approach tolerance.

**Adjacent maps overlap at each portal**, each extending past the doorway into the
neighbouring room. This is what makes AMCL converge quickly after a swap, because the
lidar immediately sees geometry present in the newly loaded map.

**The robot may pause at the portal**, roughly one to three seconds, while the map loads
and localization settles. Switching only while stationary is the single biggest
simplifier in this design.

**Why multi-map, stated accurately.** Memory is *not* the motivation, despite being the
intuitive one. An occupancy grid costs one byte per cell, so the room already mapped here
at 254 × 287 cells is 73 KB, and even a 100 × 80 m floor reaches only about 3 MB, or
roughly 30 MB once costmaps and planner structures are counted, against 2.9 GB free on
the Pi. The real benefits are:

1. **Planning latency.** Global planners work over every cell, so planning cost scales
   with map area. This is the actual resource constraint on a Pi 4 already running at
   load 2.17 across 4 cores.
2. **SLAM drift containment.** A large map accumulates odometry drift across its extent,
   and a poor loop closure can warp geometry far from the error. Small maps stay
   internally consistent, and the inter-map transforms absorb what would otherwise be
   global distortion. This is why production systems use submaps.
3. **Localization robustness.** Restricting AMCL to one room removes whole classes of
   global ambiguity in repetitive spaces.
4. **Partial re-mapping.** Re-map one changed room instead of the whole building.
5. **Semantic structure.** Maps become rooms, enabling named goals and per-room
   parameters.
6. **Extends to multiple floors**, which a single flat map cannot represent at all.

---

## 1. Core decision: an orchestrator Action Server, not custom BT nodes

The brief mentioned either a custom Action Server or a Behavior Tree node. **Use an
Action Server, and leave Nav2's BT untouched.**

The reason is that a map swap invalidates everything a running `NavigateToPose` depends
on. Mid-action, the BT holds a goal expressed in Map A's frame, a global path planned
through Map A's costmap, and a TF history whose `map` frame means Map A. Swapping the
map underneath that leaves the BT operating on three stale things at once. Recovering
from that inside the tree means teaching the BT about a discontinuity it was never
designed to model.

The alternative is to make each navigation segment a complete, self-consistent
`NavigateToPose` within a single static map, and to switch maps only *between* segments,
when nothing is mid-path. That is an orchestration problem, and an action server is the
natural ROS 2 shape for it: introspectable, cancellable, and it publishes feedback.

Concretely, one new node — call it `multimap_navigator` — exposes
`NavigateAcrossMaps` and internally sequences **stock** `NavigateToPose` goals with map
switches between them. Nav2 itself needs no modification, no recompiled BT plugins, and
no custom XML.

> Custom BT nodes are still the right tool for behaviour *within* one map — a recovery
> that retries a doorway, say. They are the wrong tool for a global state change that
> invalidates the tree's own assumptions.

---

## 2. Data model

### 2.1 Qualified poses

A bare `PoseStamped` with `frame_id: "map"` is ambiguous once several maps exist: `(2.0,
3.0)` means different physical places in different maps. Every pose crossing a module
boundary carries a **map id** alongside it. Internally the orchestrator rewrites
`frame_id` to `map` before handing a goal to Nav2, because Nav2 only ever knows about
the one map currently loaded.

### 2.2 The map graph

One YAML file is the single source of truth for topology and calibration:

```yaml
maps:
  lab:            { yaml: "package://turtlebot4_multimap_nav/maps/lab.yaml" }
  left_corridor:  { yaml: "package://turtlebot4_multimap_nav/maps/left_corridor.yaml" }
  right_corridor: { yaml: "package://turtlebot4_multimap_nav/maps/right_corridor.yaml" }

portals:
  - id: lab__left
    from: lab
    to: left_corridor
    bidirectional: true
    # The SAME physical spot, recorded in each map's own frame.
    pose_in_from: { x: -2.40, y: 0.35, yaw: 3.1416 }
    pose_in_to:   { x:  4.10, y: 1.20, yaw: 0.0000 }
    approach_tolerance: { xy: 0.25, yaw: 0.20 }

  - id: lab__right
    from: lab
    to: right_corridor
    bidirectional: true
    pose_in_from: { x:  3.10, y: 0.45, yaw: 0.0000 }
    pose_in_to:   { x: -1.20, y: 2.80, yaw: 3.1416 }
    approach_tolerance: { xy: 0.25, yaw: 0.20 }

  - id: left__right
    from: left_corridor
    to: right_corridor
    bidirectional: true
    # Site this at a corner or junction if possible — see section 0.
    pose_in_from: { x: -6.80, y: 9.25, yaw: 1.5708 }
    pose_in_to:   { x:  7.30, y: 9.40, yaw: 1.5708 }
    approach_tolerance: { xy: 0.35, yaw: 0.30 }   # looser: less distinctive site

# True path lengths between portals sharing a map, measured once with
# ComputePathToPose (section 5.2). With a cycle these decide which way round
# the robot goes, so they are not optional.
portal_distances:
  left_corridor:
    lab__left <-> left__right: 11.4
  right_corridor:
    lab__right <-> left__right: 12.1
  lab:
    lab__left <-> lab__right: 5.6
```

`maps` gives the Map Server its `map_url`. Each portal does double duty: `pose_in_from`
is the navigation goal for the leg inside Map A, and the pose pair yields the inter-map
transform of section 3.

---

## 3. The inter-map transform

This is the part the brief flagged, and it is the crux of the design.

Each map's origin is arbitrary — wherever SLAM happened to start. Two maps of adjacent
rooms have unrelated origins *and* unrelated rotations. To re-seed the pose after a
swap, the system needs the rigid 2D transform between the frames.

### 3.1 Deriving it from one calibration pair

A 2D rigid transform has three degrees of freedom, and a single pose supplies three
constraints, so **one paired observation per portal fully determines it.** Park the
robot at the portal, record its pose in Map A, swap to Map B, localize, record its pose
in Map B. Those two poses are `pose_in_from` and `pose_in_to`.

Given `P_A = (x_A, y_A, θ_A)` and `P_B = (x_B, y_B, θ_B)` at the same physical spot:

```
dθ = normalize(θ_B − θ_A)
dx = x_B − ( x_A·cos(dθ) − y_A·sin(dθ) )
dy = y_B − ( x_A·sin(dθ) + y_A·cos(dθ) )
```

Then any Map-A pose converts to Map B:

```
x_B = dx + x_A·cos(dθ) − y_A·sin(dθ)
y_B = dy + x_A·sin(dθ) + y_A·cos(dθ)
θ_B = normalize(θ_A + dθ)
```

Store the pose pairs, not the derived `(dx, dy, dθ)`. The pairs are physically
meaningful, independently checkable on a map image, and re-derivable; a bare triple is
none of those. Averaging several paired observations per portal reduces calibration
noise, and disagreement between them is itself a useful warning.

### 3.2 Calibration is a real procedure, not an afterthought

Recording those pairs needs a small tool — a `record_portal` mode in the orchestrator
that captures the current AMCL pose on demand and appends it to the graph YAML. Budget
real time for it. Calibration error at a portal shows up later as a mysterious
localization failure one room away.

### 3.3 The cycle gives a loop-closure check, for free

The triangle topology makes a genuine consistency test possible, and it is the single
most valuable diagnostic in this design.

Compose the three transforms all the way around the loop:

```
T_total  =  T(lab → left)  ∘  T(left → right)  ∘  T(right → lab)
```

Starting in the lab frame and going all the way round returns you to the lab frame, so
**`T_total` must be the identity**. Whatever it actually comes out as is the accumulated
error across all three maps and all three calibrations, in one number. This is exactly
the loop-closure residual that SLAM back-ends minimise, and here it drops out of the
calibration data at no cost.

Composition of two 2D transforms, applying `T1` then `T2`:

```
dθ = normalize(dθ1 + dθ2)
dx = dx2 + dx1·cos(dθ2) − dy1·sin(dθ2)
dy = dy2 + dx1·sin(dθ2) + dy1·cos(dθ2)
```

Interpreting the residual:

| Loop residual | Meaning |
|---|---|
| Under ~5 cm, ~2° | All three maps mutually consistent. Proceed. |
| 10–30 cm | Real SLAM drift in one or more maps. Workable, but expect a visible AMCL correction after each swap. |
| Over ~50 cm | A calibration mistake, or one badly distorted map. Re-record each pair, and re-map the worst offender if it persists. |

A chain topology cannot do this, because nothing closes back on itself. Build the check
into the calibration tool from the start. It is also the most quotable result in the
project write-up, since it converts "the maps are good" into a measured number.

**Which transform to use at runtime:** always the one belonging to the portal actually
crossed. It is locally the most accurate, whatever the global residual, and AMCL absorbs
what remains through scan matching. Do not attempt to distribute the loop error across
all three portals unless localization proves inadequate without it; that is pose-graph
optimisation, and it is a much larger undertaking than this project needs.

### 3.4 The fallback: no calibration at all

If a portal has no calibration, the system can load Map B, call AMCL's
`/reinitialize_global_localization`, and rotate in place until the particle filter
converges. This needs no setup, but it is slow, and it fails outright in symmetric or
repetitive spaces where a corridor looks like every other corridor. Treat it as a
recovery path, not the primary mechanism.

---

## 4. The map switch sequence

Each step, with its exact interface. This runs only when the robot is stationary at a
portal and no `NavigateToPose` is active.

| # | Step | Interface |
|---|---|---|
| 1 | Confirm arrival at the portal within tolerance | TF lookup `map → base_link` |
| 2 | Compute the pose in B's frame | Section 3 math, in-process |
| 3 | Load Map B | `/map_server/load_map`, `nav2_msgs/srv/LoadMap` |
| 4 | Check the result code | `result == 0` (`RESULT_SUCCESS`) |
| 5 | Re-seed AMCL | publish `/initialpose`, `geometry_msgs/msg/PoseWithCovarianceStamped` |
| 6 | Clear stale obstacles | `/global_costmap/clear_entirely_global_costmap` and the local equivalent, `nav2_msgs/srv/ClearEntireCostmap` |
| 7 | Wait for AMCL to converge | watch `/amcl_pose` covariance against a threshold, with a timeout |
| 8 | Resume | next `NavigateToPose` in B's frame |

`LoadMap`'s `map_url` accepts `file:///abs/path.yaml` or
`package://pkg_name/maps/name.yaml`; the package form is preferable since it survives
deployment to a different machine.

Step 5 deserves a caution: `/initialpose` is a **topic**, so publishing is
fire-and-forget with no acknowledgement. Publish it, then verify convergence in step 7
rather than assuming it took. Set a generous initial covariance — the seed is
approximately right, and letting AMCL tighten it from scan data is more robust than
asserting false confidence.

### 4.1 Two configuration details that will silently break this

**`amcl.first_map_only` must be `false`.** If true, AMCL ignores every map after the
first and will keep localizing against Map A forever while the costmap shows Map B — a
confusing failure with no error message. I checked: it is *not* set in
`turtlebot4_navigation/config/localization.yaml`, so it falls back to AMCL's built-in
default of `false`, which is what we want. **Set it explicitly anyway** so a future
config edit cannot silently regress it.

**Clear the costmaps (step 6).** The global costmap's static layer accepts the new map,
but obstacle layers retain marks from Map A. Without a clear, the robot plans around
obstacles that exist in a room it has left.

---

## 5. Routing to a goal in an unloaded map

The orchestrator holds the whole graph in memory, including maps that are not loaded, so
routing does not need Map B present to plan *which maps to cross*.

### 5.1 Why the obvious formulation is wrong here

The intuitive graph puts **maps** at the nodes and portals on the edges. With two doors
between the same pair of maps that breaks down, because the cost of crossing a map
depends on *which door you entered by and which you leave by*. A single edge weight
cannot express that.

**Put the portals at the nodes instead.** The graph becomes:

- One node per portal, plus a `START` node at the robot's current pose and a `GOAL` node
  at the requested pose.
- An edge between two portals that share a map, weighted by the distance between them
  *within that map*.
- An edge `START → portal` for every portal in the robot's current map, weighted by
  distance.
- An edge `portal → GOAL` for every portal in the goal's map, weighted by distance.
- A small fixed penalty per map switch, since each costs one to three seconds. Roughly
  five metres' equivalent discourages pointlessly crossing maps.

Dijkstra from `START` to `GOAL` then yields the optimal portal sequence, and parallel
portals fall out naturally with no special case.

For the two-map, two-door layout here this collapses to exactly what intuition suggests,
comparing

```
cost(north) = d(robot, north.pose_in_from) + d(north.pose_in_to, G)
cost(south) = d(robot, south.pose_in_from) + d(south.pose_in_to, G)
```

and taking the smaller. The general formulation costs only a few extra lines and keeps
working if a third map is added later.

### 5.2 Distance metric — straight-line will not do here

In a simple room, straight-line distance is a fine approximation. **Around a corridor
that wraps a building it is actively wrong**, because two points can be metres apart in a
straight line while the only route between them runs the long way round.

The case that decides real routes here: a goal in `right_corridor` sitting close to the
`left__right` join. Straight out the east door is the obvious choice geometrically, but
the actual path may be shorter out the west door and round through the join. Only true
path lengths reveal that, and it is precisely the decision this architecture exists to
make well.

**Precompute the distances once.** The building's geometry is static, so the real path
length between each pair of portals sharing a map is measured once with Nav2's
`ComputePathToPose` action, which plans a path without driving it, and stored as
`portal_distances` in the map graph. With three maps and three portals there are only
three such pairs, so this is a short exercise. It is exact, costs nothing at runtime, and
belongs in the same calibration pass that records the pose pairs.

Computing them live is the alternative. Always current, but it spends a planning call per
candidate on a Pi that is already loaded, and the geometry does not change anyway.

Straight-line distance remains acceptable for the first and last hop, from the robot's
current pose to a portal within one map, and from a portal to the goal, since a single
room or corridor segment has no comparable detour structure.

### 5.3 Execution

Expand the chosen portal sequence into legs that alternate navigation and switching:

- `NavigateToPose` to the chosen portal's `pose_in_from`, in `lab`
- switch `lab → corridor` using **that portal's** transform
- `NavigateToPose` to `G`, in `corridor`

Execute sequentially, publishing feedback (`current_map`, `phase`, `legs_remaining`) and
aborting the whole action if any leg fails.

Metric planning stays entirely inside Nav2, one map at a time. The orchestrator only
does symbolic routing between maps. That separation is what keeps it simple.

---

## 6. Where each piece runs

This robot has a hard constraint established earlier: the Create 3 binds its DDS to the
USB link to the Pi, so the laptop cannot receive base odometry or publish `/cmd_vel` to
it. See the [README](../README.md#real-robot-slam-on-the-pi-viewing-on-the-pc).

| Component | Host | Why |
|---|---|---|
| `map_server` | **Pi** | AMCL and the costmaps need it locally; map files deploy alongside |
| `amcl` | **Pi** | Needs `/scan` and `odom→base_link` at scan rate; publishes `map→odom` |
| `controller_server`, `behavior_server` | **Pi** | They publish `/cmd_vel`, and a 20 Hz control loop must not cross WiFi |
| `planner_server`, `bt_navigator` | **Pi** at first | Move to the laptop later only if the Pi is measurably short of CPU |
| **`multimap_navigator`** | **Laptop** | Action and service calls at roughly 1 Hz — entirely latency-tolerant |
| RViz | Laptop | Already working |

This lands nicely against the stated architectural preference: the multi-map
*intelligence* runs on the laptop, while the real-time stack stays on the robot where
its inputs are local. The laptop-to-Pi link is fully bidirectional — verified, the Pi
sees the laptop's nodes — so the orchestrator can call Pi-hosted actions and services
without any relay.

Map `.pgm`/`.yaml` files must be deployed to the Pi, since `map_server` reads them from
its own disk. An extension of the existing `tb4-pi-deploy` helper covers this.

---

## 7. Workspace changes

Two new packages, alongside the existing `turtlebot4_slam_*` set:

**`turtlebot4_multimap_msgs`** (`ament_cmake`) — interface definitions only. Separate
package because mixing `rosidl` generation with a Python node in one package causes
dependency-ordering pain.

```
NavigateAcrossMaps.action
  # Goal
  string target_map
  geometry_msgs/PoseStamped goal_pose
  ---
  # Result
  bool     success
  string   message
  uint16   maps_traversed
  ---
  # Feedback
  string                   current_map
  string                   phase           # NAVIGATING | SWITCHING | LOCALIZING
  geometry_msgs/PoseStamped current_pose
  uint16                   legs_remaining

SwitchMap.srv          # manual switching, for bring-up and testing
  string target_map
  ---
  bool   success
  string message
```

**`turtlebot4_multimap_nav`** (`ament_python`) — Python is right here: this is
orchestration at 1 Hz, not an inner loop.

```
turtlebot4_multimap_nav/
  multimap_navigator.py     # the action server
  map_graph.py              # YAML loader, graph search, transform math
  portal_recorder.py        # calibration tool (section 3.2)
config/map_graph.yaml
maps/                       # the .pgm / .yaml pairs
launch/
  multimap_localization.launch.py   # map_server + amcl, first_map_only explicitly false
  multimap_nav.launch.py            # the orchestrator
```

The existing `turtlebot4_slam_bringup/launch/tb4_slam.launch.py` gains a `nav` mode
following its current `MODES` dict pattern, reusing the `tf_relay` remap already there.
`scripts/aliases.sh` gains `tb4-pi-nav-start` / `-stop` / `-log`, mirroring the existing
`tb4-pi-slam-*` helpers.

---

## 8. Failure modes to design for from the start

| Failure | Detection | Response |
|---|---|---|
| AMCL diverges after a swap | `/amcl_pose` covariance above threshold after timeout | `/reinitialize_global_localization`, rotate in place, retry once, then abort |
| Map file missing on the Pi | `LoadMap` returns `RESULT_MAP_DOES_NOT_EXIST` | Abort the action with a clear message; do not silently continue on Map A |
| Switch triggered away from the portal | TF check in step 1 fails tolerance | Refuse to switch, re-navigate to the portal |
| Stale TF after the swap | `map→odom` jumps discontinuously | Only ever switch while stationary between legs; allow the TF buffer to age out before planning |
| Calibration drift at a portal | Pose after swap disagrees with scan match | Log it; re-record the portal pair |

---

## 9. Suggested build order

1. **Manual switch first.** `SwitchMap.srv` plus the map graph, driven by hand with
   `ros2 service call`, teleop for motion. This validates the transform math and the
   `first_map_only` behaviour before any autonomy exists.
2. **Calibration tool**, once switching is proven.
3. **Single-hop autonomy.** `NavigateAcrossMaps` across exactly one portal.
4. **Multi-hop routing** over the full graph.
5. **Failure handling** from section 8.

Step 1 is worth resisting the urge to skip. Nearly every hard bug in this design is a
frame-convention bug, and they are dramatically easier to find with a stationary robot
and a hand-issued service call than inside a running action server.
