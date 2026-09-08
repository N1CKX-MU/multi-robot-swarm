# Multi-Robot Swarm Coordination

Work in progress. ROS 2 Jazzy, Gazebo Harmonic, TurtleBot3, following a
7-day build plan (Day 1 of 7 complete).

## Status

**Day 1 — Multi-robot spawning and namespaced topics: done.**

`launch/spawn_swarm.launch.py` spawns a configurable number of TurtleBot3
Waffles into one Gazebo Harmonic world, each fully isolated:

- Gazebo-transport topics (`cmd_vel`, `odom`, `scan`) are rewritten per
  robot at spawn time, since they're hardcoded in the stock model's SDF
  and would otherwise collide across robots.
- `robot_state_publisher` runs per robot with `frame_prefix` set, so each
  robot's TF tree (`tb0/base_link`, `tb1/base_link`, ...) is distinct.
- `ros_gz_bridge` configs are generated per robot in code
  (`build_bridge_yaml`), since the topic names differ per robot and can't
  live in one static YAML file.

Verified with 4 robots spawned simultaneously: distinct `/tb{0..3}/cmd_vel`,
`/odom`, `/scan`, `/joint_states` topics, correct per-robot TF frame ids,
no cross-robot collisions.

**Day 1 — Namespaced Nav2 per robot: namespacing verified, full goal
execution blocked by a WSL2 environment quirk.**

`launch/swarm_nav2.launch.py` brings up an independent `nav2_bringup`
stack per robot. Getting here surfaced two real bugs, both fixed:

- `nav2_bringup`'s `navigation_launch.py` never actually applies the
  `namespace` argument it takes to its own nodes in the standalone
  (non-composed) code path -- it's expected to be namespaced by an
  *outer* `PushRosNamespace`, normally supplied by `bringup_launch.py`,
  which we bypass by calling `navigation_launch.py` directly. Without
  that wrapper every robot's Nav2 stack ran completely unnamespaced,
  which silently broke parameter loading for anything namespace-nested
  (surfaced as `nav2_mppi_controller`'s "No critics defined for
  FollowPath", since our params file expected a namespaced node that
  didn't exist). Fixed by wrapping our own
  `IncludeLaunchDescription(navigation_launch.py, ...)` in
  `GroupAction([PushRosNamespace(ns), ...])`.
- Nav2's own nodes apply the same `/tf` -> `tf` remap
  `robot_state_publisher` needed on Day 1, so TF ends up namespaced
  per-robot end to end (`/tb0/tf`, not the single shared bus Day 1 used
  before Nav2 entered the picture -- see `spawn_swarm.launch.py` for
  why that changed).
- Four robots' Nav2 stacks starting simultaneously (~40+ ROS nodes)
  overloads Gazebo/DDS on this hardware -- staggered via `TimerAction`
  (`STAGGER_SECONDS`), matching the build plan's own "Common pitfalls"
  warning.

Verified: all 4 robots' `controller_server`s load all 8 MPPI critics
correctly (32 "Critic loaded" log lines across 4 robots) with correct
`/tb{0..3}/...` node names throughout -- the namespacing bug is
genuinely fixed.

**Not yet resolved:** with all 4 stacks running, the local costmap
intermittently reports "TF has two or more unconnected trees" for the
placeholder `map -> {ns}/odom` static transform, and brand-new CLI
tools occasionally fail to attach to the graph ("rcl node's context is
invalid") once ~40+ nodes are up. Confirmed this is *not* the
namespacing logic (verified correct above); a FastDDS shared-memory
tuning profile made no difference, pointing to WSL2's SHM transport
being flaky under this much sustained load rather than a resolvable
resource limit. See `swarm_nav2.launch.py`'s docstring for the two
honest next steps (test with fewer robots, or just wait for Day 4's
real SLAM localization to replace this placeholder outright).

Not done yet: consensus rendezvous, formation control, cooperative
exploration, metrics, or the final polished README -- these follow over
the rest of the 7-day plan.

## Quick start

```bash
colcon build --symlink-install --packages-select swarm_coordination
source install/setup.bash

# terminal 1
ros2 launch swarm_coordination spawn_swarm.launch.py
# terminal 2, once the above has settled
ros2 launch swarm_coordination swarm_nav2.launch.py
```

Verify:

```bash
ros2 topic list | grep tb0
ros2 run tf2_tools view_frames
ros2 service call /tb0/controller_server/get_state lifecycle_msgs/srv/GetState
```

## Structure

```
src/swarm_coordination/
├── swarm_coordination/   # Python module (empty so far -- Day 2+)
├── config/
│   └── nav2_params.yaml  # Templated per robot at launch time
├── launch/
│   ├── spawn_swarm.launch.py
│   └── swarm_nav2.launch.py
└── worlds/
```
