# Multi-Robot Swarm Coordination

Work in progress. ROS 2 Jazzy, Gazebo Harmonic, TurtleBot3, following a
7-day build plan (Days 1-5 of 7 complete).

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

**Day 2 — Consensus rendezvous (plain numpy, `tiny/consensus_rendezvous.py`): done.**

Hand-derived the graph Laplacian and its eigenvalues for four topologies
(ring, complete, star, line) before writing any code, then verified the
numpy implementation reproduces those exact numbers:

| Topology | Fiedler value (hand-derived == code output) |
|---|---|
| Complete | 4 |
| Ring | 2 |
| Star | 1 |
| Line | 0.5858 |

`consensus_{ring,complete,star,line}.png` show each topology's
trajectories; `convergence_comparison.png` overlays all four -- the
empirical proof that more graph connectivity converges faster.

Also found something the plan doesn't state explicitly: the textbook
stability rule "epsilon < 1/max_degree" is a conservative sufficient
bound, not the real condition (`epsilon < 2/lambda_max`). Demonstrated
by running ring and star at the same epsilon=0.6 past that bound --
ring stayed stable anyway because the initial square-corner positions
have exactly zero overlap (dot product) with ring's one unstable
eigenvector, while star's initial positions have nonzero overlap with
its unstable eigenvector and diverge to ~1e7. Same danger eigenvalue on
both graphs (lambda_max=4), completely different outcome, because
stability under a fixed epsilon depends on the initial condition, not
just the graph.

**Day 3 — Formation control (plain numpy, `tiny/formation_control.py`): done.**

Leader-follower formation control: a scripted leader follows a sine-wave
path, three followers hold a rotating offset around it
(`goal = center + R(heading) @ offset`, heading taken from the leader's
velocity direction via `atan2`), with a hard formation-shape switch
(square -> diamond -> line) partway through. `formation_run.png` shows
the followers visibly swinging with the leader's changing heading
(bigger offsets swing more) and the discontinuous jump at each shape
switch.

**Day 4 — Cooperative exploration via Voronoi partitioning (plain numpy,
`tiny/cooperative_exploration.py`): done.**

Frontier detection (free cell adjacent to unknown) filtered by Voronoi
ownership (nearest-robot-wins, computed as a brute-force per-cell
distance comparison -- no `scipy.spatial.Voronoi`, which sidesteps that
library's known crash on collinear/degenerate robot positions, since no
explicit Voronoi diagram is ever constructed). `cooperative_exploration.png`
shows a fake occupancy grid split into 4 non-overlapping regions, with
each frontier cell colored by its owning robot.

**Day 5 — Metrics + exploration scaling experiment (plain numpy,
`tiny/exploration_scaling.py`): done.**

Extends Day 4 into a real time-stepped simulation: robots move toward
their nearest owned frontier and sense a radius around themselves each
step, run until 95% coverage, repeated over 5 trials per robot count
(1-5) to measure mean +/- stddev rather than trusting a single run.
Result: speedup climbs with diminishing returns (1.89x -> 4.35x going
from 2 to 5 robots) while redundant coverage overlap rises from ~0% to
~19% and plateaus -- the real cost/benefit curve of adding robots.

Caught and fixed two bugs in the experiment's own boilerplate before
trusting the result: every trial was coming back bit-identical (a
hardcoded RNG seed inside the map generator, and a global
`np.random.seed()` call that doesn't actually affect
`np.random.default_rng()` objects at all), and the 1-robot baseline was
hitting its step cap without finishing, which would have silently
deflated every speedup number. Written by Claude at Nick's request
(not hand-derived/hand-typed like Days 2-4) -- flagged here rather than
implied otherwise.

Not done yet: dynamic behaviors / collision avoidance, the ROS
integration of the consensus/formation/exploration controllers, or the
final polished README -- these follow over the rest of the 7-day plan.

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
