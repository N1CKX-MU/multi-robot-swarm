#!/usr/bin/env python3
"""
swarm_nav2.launch.py — Day 1 deliverable: an independent, namespaced
Nav2 stack per robot.

Prerequisite: spawn_swarm.launch.py must already be running (separate
terminal) — this file does NOT spawn robots, only Nav2.

Run:
    # terminal 1
    ros2 launch swarm_coordination spawn_swarm.launch.py
    # terminal 2, once the above has settled
    ros2 launch swarm_coordination swarm_nav2.launch.py

Send independent goals:
    ros2 action send_goal /tb0/navigate_to_pose nav2_msgs/action/NavigateToPose \
        "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 1.0}}}}"
    ros2 action send_goal /tb1/navigate_to_pose nav2_msgs/action/NavigateToPose \
        "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 3.0}}}}"

No SLAM yet (that's Day 4), so there is no real localization. Each
robot gets a STATIC map -> {ns}/odom transform published at its known
spawn pose instead — a deliberate placeholder, not real localization.
It lets "map"-frame goals line up with Gazebo world coordinates for
now; it will be replaced once SLAM Toolbox / AMCL enters the picture.

KNOWN ISSUE (as of Day 1, WSL2 + FastRTPS on this machine): with all 4
robots' Nav2 stacks running (~40+ ROS nodes total), the local costmap
sometimes logs "TF has two or more unconnected trees" for map -> {ns}/odom,
and separately, brand-new CLI tools (ros2 topic info, ros2 action
send_goal, etc.) intermittently fail with "rcl node's context is
invalid" when the graph is this busy. Verified NOT caused by our
namespacing logic: robot namespacing itself is correct end-to-end
(confirmed via `Critic loaded` messages appearing 8x per robot, 32x
total, and correct /tb0/... node names throughout) — a FastDDS
shared-memory transport tuning profile (increased segment size) made no
measurable difference, pointing to WSL2's SHM transport being flaky
under this much sustained multi-process load rather than a fixable
resource limit. Two honest paths forward, neither attempted yet:
  1. Test with fewer robots at a time (edit ROBOTS below) until this
     WSL2 install's real ceiling is found.
  2. This whole static-transform placeholder goes away once Day 4's
     SLAM Toolbox provides real map->odom localization anyway — worth
     revisiting there rather than over-investing in a placeholder now.
"""
import os

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

# Must match the ROBOTS list in spawn_swarm.launch.py — name AND spawn
# pose both matter here, since the static map->odom transform is set to
# each robot's actual spawn position.
ROBOTS = [
    {'name': 'tb0', 'x': 0.0, 'y': 0.0},
    {'name': 'tb1', 'x': 2.0, 'y': 0.0},
    {'name': 'tb2', 'x': 0.0, 'y': 2.0},
    {'name': 'tb3', 'x': 2.0, 'y': 2.0},
]

GENERATED_PARAMS_DIR = '/tmp/swarm_nav2_params'


def build_namespaced_nav2_params(ns: str) -> str:
    """
    Read config/nav2_params.yaml and rewrite it for one robot: every
    frame name that Nav2 will look up via TF gets the `ns/` prefix, and
    every ABSOLUTE topic reference (leading "/") gets namespaced too —
    same leading-slash-vs-relative rule as tb_bridge.yaml's design.
    Anything already relative (no leading "/") is left alone: it will
    resolve correctly on its own once RewrittenYaml nests these params
    under the robot's namespace.
    """
    my_share = get_package_share_directory('swarm_coordination')
    template_path = os.path.join(my_share, 'config', 'nav2_params.yaml')

    with open(template_path, 'r', encoding='utf-8') as f:
        params = f.read()

    # Frames (appear as literal parameter VALUES, not touched by ROS
    # namespace resolution at all — YAML text, not a topic/service name).
    params = params.replace('robot_base_frame: base_link', f'robot_base_frame: {ns}/base_link')
    params = params.replace('global_frame: odom', f'global_frame: {ns}/odom')
    params = params.replace('base_frame_id: "base_footprint"', f'base_frame_id: "{ns}/base_footprint"')
    params = params.replace('odom_frame_id: "odom"', f'odom_frame_id: "{ns}/odom"')
    params = params.replace('base_frame: "base_link"', f'base_frame: "{ns}/base_link"')
    params = params.replace('fixed_frame: "odom"', f'fixed_frame: "{ns}/odom"')
    # global_frame: map is intentionally NOT touched anywhere — shared.

    # Absolute topics — leading "/" means "escape any namespace", exactly
    # like an absolute gz_topic_name would. Relative ones (e.g. the
    # velocity_smoother's odom_topic: "odom", collision_monitor's
    # topic: "scan") are left alone on purpose: they'll resolve under
    # the robot's namespace automatically.
    params = params.replace('odom_topic: /odom', f'odom_topic: /{ns}/odom')
    params = params.replace('topic: /scan', f'topic: /{ns}/scan')
    params = params.replace(
        'footprint_topic: "/local_costmap/published_footprint"',
        f'footprint_topic: "/{ns}/local_costmap/published_footprint"'
    )

    return params


def write_nav2_params(ns: str) -> str:
    """Write build_namespaced_nav2_params(ns) to disk and return the path."""
    os.makedirs(GENERATED_PARAMS_DIR, exist_ok=True)
    path = os.path.join(GENERATED_PARAMS_DIR, f'{ns}_nav2_params.yaml')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(build_namespaced_nav2_params(ns))
    return path


# Seconds between each robot's Nav2 stack starting. Each stack is ~9
# heavy C++ nodes (2 costmaps each); starting all of them for every robot
# at once caused real crashes on this machine (RTPS shared-memory port
# contention -> "Node already added to an executor" -> SIGABRT) even
# though the generated params were verified correct. This is exactly the
# staggered-startup pitfall the build plan calls out under "Common
# pitfalls" for Day 1 — decided here rather than left implicit.
STAGGER_SECONDS = 6.0


def generate_launch_description():
    nav2_bringup = get_package_share_directory('nav2_bringup')
    actions = []

    for i, robot in enumerate(ROBOTS):
        ns = robot['name']
        params_path = write_nav2_params(ns)

        # Placeholder localization: static map -> {ns}/odom at the
        # robot's known spawn pose. Real localization (SLAM Toolbox)
        # replaces this on Day 4+.
        #
        # Like robot_state_publisher on Day 1, static_transform_publisher
        # publishes tf_static on the ABSOLUTE /tf_static topic by default.
        # Nav2's own nodes only listen on /{ns}/tf_static (their own
        # internal remap, applied under our PushRosNamespace(ns) below) —
        # without the same remap here, this transform lands on a topic
        # Nav2 never reads, and TF ends up as two disconnected trees.
        map_to_odom = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'map_to_odom_{ns}',
            namespace=ns,
            arguments=[
                '--x', str(robot['x']),
                '--y', str(robot['y']),
                '--z', '0',
                '--yaw', '0',
                '--frame-id', 'map',
                '--child-frame-id', f'{ns}/odom',
            ],
            remappings=[('/tf_static', 'tf_static')],
            output='screen',
        )

        # navigation_launch.py's own Node() calls never set namespace= and
        # never push one internally in the standalone (non-composed)
        # branch we use — the `namespace` argument it takes only feeds
        # its RewrittenYaml root_key and composable-container name. Actual
        # node namespacing is expected to come from an OUTER
        # PushRosNamespace, normally supplied by bringup_launch.py, which
        # we're bypassing by calling navigation_launch.py directly. Without
        # this wrapper the nodes come up completely unnamespaced
        # (/controller_server, not /tb0/controller_server) while our
        # params file is nested under a "tb0:" key expecting a namespaced
        # node — nothing matches, and Nav2 silently falls back to empty
        # defaults (surfaced as "No critics defined for FollowPath").
        nav2 = GroupAction([
            PushRosNamespace(ns),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'namespace': ns,
                    'use_sim_time': 'True',
                    'params_file': params_path,
                    'autostart': 'True',
                }.items(),
            ),
        ])

        actions.append(TimerAction(
            period=i * STAGGER_SECONDS,
            actions=[map_to_odom, nav2],
        ))

    return LaunchDescription(actions)
