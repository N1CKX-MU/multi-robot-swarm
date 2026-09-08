#!/usr/bin/env python3
"""
spawn_swarm.launch.py — Day 1, MULTI-ROBOT (loop) version.

Spawns every robot in ROBOTS, each in its own namespace, each with its
own rewritten SDF and its own dynamically-generated ros_gz_bridge config.

Run:
    ros2 launch swarm_coordination spawn_swarm.launch.py

Verify:
    ros2 topic list | grep tb1
    ros2 run tf2_tools view_frames
"""
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# ---------------------------------------------------------------------------
# Robot roster: name, spawn x, spawn y. Add/remove entries to change how
# many robots spawn — nothing else in this file needs to change.
# ---------------------------------------------------------------------------
ROBOTS = [
    {'name': 'tb0', 'x': 0.0, 'y': 0.0},
    {'name': 'tb1', 'x': 2.0, 'y': 0.0},
    {'name': 'tb2', 'x': 0.0, 'y': 2.0},
    {'name': 'tb3', 'x': 2.0, 'y': 2.0},
]

# Generated per-robot bridge configs land here. Regenerated fresh on
# every launch — nothing depends on them persisting between runs.
GENERATED_CONFIG_DIR = '/tmp/swarm_bridge_configs'


def build_namespaced_sdf(ns: str) -> str:
    """
    Read the stock turtlebot3_waffle model.sdf and return a modified SDF
    string where the DiffDrive plugin's Gazebo-transport topics, the
    lidar's topic, and the odometry TF frame ids are unique to `ns`.
    """
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    model_path = os.path.join(
        tb3_gazebo_share, 'models', 'turtlebot3_waffle', 'model.sdf'
    )

    with open(model_path, 'r', encoding='utf-8') as f:
        sdf = f.read()

    sdf = sdf.replace('<topic>cmd_vel</topic>', f'<topic>{ns}/cmd_vel</topic>')
    sdf = sdf.replace('<odom_topic>odom</odom_topic>', f'<odom_topic>{ns}/odom</odom_topic>')
    sdf = sdf.replace('<frame_id>odom</frame_id>', f'<frame_id>{ns}/odom</frame_id>')
    sdf = sdf.replace('<child_frame_id>base_footprint</child_frame_id>', f'<child_frame_id>{ns}/base_footprint</child_frame_id>')
    # Lidar sensor has an explicit <topic>, same collision risk as cmd_vel.
    sdf = sdf.replace('<topic>scan</topic>', f'<topic>{ns}/scan</topic>')
    # tf_topic: originally shared "/tf" across all robots (Day 1 design —
    # one bus, frame_id disambiguates). Nav2's navigation_launch.py forces
    # every node's TF onto a namespace-relative topic (hardcoded
    # remappings=[('/tf','tf'), ...]), so each robot needs its OWN tf
    # topic to match what Nav2 will actually listen to. See the
    # ros/geometry2#32 comment inside navigation_launch.py itself.
    sdf = sdf.replace('<tf_topic>/tf</tf_topic>', f'<tf_topic>{ns}/tf</tf_topic>')

    return sdf


def build_bridge_yaml(ns: str) -> str:
    """
    Return the full ros_gz_bridge YAML for one robot, with every
    gz_topic_name/ros_topic_name namespaced to `ns` (except clock, which
    is genuinely global). tf is namespaced per robot too — see the note
    in build_namespaced_sdf() for why this changed from Day 1's shared
    /tf design once Nav2 entered the picture.
    """
    return f"""\
- ros_topic_name: "clock"
  gz_topic_name: "clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS

- ros_topic_name: "/{ns}/cmd_vel"
  gz_topic_name: "{ns}/cmd_vel"
  ros_type_name: "geometry_msgs/msg/TwistStamped"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "/{ns}/odom"
  gz_topic_name: "{ns}/odom"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS

- ros_topic_name: "/{ns}/tf"
  gz_topic_name: "{ns}/tf"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: GZ_TO_ROS

- ros_topic_name: "/{ns}/joint_states"
  gz_topic_name: "joint_states"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS

- ros_topic_name: "/{ns}/scan"
  gz_topic_name: "{ns}/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS
"""


def write_bridge_config(ns: str) -> str:
    """Write build_bridge_yaml(ns) to disk and return the path."""
    os.makedirs(GENERATED_CONFIG_DIR, exist_ok=True)
    path = os.path.join(GENERATED_CONFIG_DIR, f'{ns}_bridge.yaml')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(build_bridge_yaml(ns))
    return path


def spawn_robots(context, *args, **kwargs):
    """Spawn every robot in ROBOTS with its own namespace and position."""
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    urdf_path = os.path.join(tb3_gazebo_share, 'urdf', 'turtlebot3_waffle.urdf')

    actions = []

    for robot in ROBOTS:
        ns = robot['name']
        sdf_string = build_namespaced_sdf(ns)
        bridge_config = write_bridge_config(ns)

        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            name=f'create_{ns}',
            arguments=[
                '-name', ns,
                '-string', sdf_string,
                '-x', str(robot['x']),
                '-y', str(robot['y']),
                '-z', '0.01',
            ],
            output='screen',
        )

        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=ns,
            name='robot_state_publisher',
            parameters=[{'use_sim_time': True, 'frame_prefix': f'{ns}/'}],
            arguments=[urdf_path],
            # robot_state_publisher publishes tf/tf_static on the ABSOLUTE
            # /tf, /tf_static topics by default, ignoring its own
            # namespace. Remap to the relative names first so namespace=ns
            # actually applies, landing this robot's static tree on
            # /tb0/tf — same fix Nav2's own navigation_launch.py applies
            # to its nodes, and required for the two to end up on the
            # same topic.
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            output='screen',
        )

        bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'ros_gz_bridge_{ns}',
            arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
            output='screen',
        )

        actions.extend([spawn_node, rsp_node, bridge_node])

    return actions


def generate_launch_description():
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    return LaunchDescription([
        gz_sim,
        OpaqueFunction(function=spawn_robots),
    ])
