import os
from xml.dom import Node as XmlNode

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _robot_description_from_xacro(use_sim: bool) -> str:
    def strip_comments(node):
        for child in list(node.childNodes):
            if child.nodeType == XmlNode.COMMENT_NODE:
                node.removeChild(child)
                child.unlink()
            else:
                strip_comments(child)

    pkg_share = get_package_share_directory("arm_4dof_5axis_description")
    xacro_file = os.path.join(pkg_share, "urdf", "arm_4dof_5axis.urdf.xacro")
    doc = xacro.process_file(
        xacro_file, mappings={"use_sim": "true" if use_sim else "false"}
    )
    strip_comments(doc)
    return doc.documentElement.toxml()


def generate_launch_description():
    safe_init = [0.0, -0.9, 0.8, 0.3, 0.018]
    init_pose_request = (
        "{model_name: 'arm_4dof_5axis', urdf_param_name: 'robot_description', "
        "joint_names: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'left_finger_joint'], "
        "joint_positions: [%s, %s, %s, %s, %s]}" % tuple(safe_init)
    )

    pkg_share = FindPackageShare("arm_4dof_5axis_description")

    world_file = PathJoinSubstitution([pkg_share, "worlds", "empty.world"])

    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
    )

    robot_description = ParameterValue(
        _robot_description_from_xacro(use_sim=True),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={"world": world_file}.items(),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "arm_4dof_5axis",
            "-topic",
            "robot_description",
            "-z",
            "0.02",
        ],
        output="screen",
    )

    set_init_pose = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/gazebo/set_model_configuration",
            "gazebo_msgs/srv/SetModelConfiguration",
            init_pose_request,
        ],
        output="screen",
    )

    jsb = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "controller_manager",
            "spawner",
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    arm = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "controller_manager",
            "spawner",
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    gripper = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "controller_manager",
            "spawner",
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo,
            rsp,
            spawn,
            RegisterEventHandler(
                OnProcessExit(target_action=spawn, on_exit=[set_init_pose])
            ),
            TimerAction(period=3.0, actions=[jsb]),
            TimerAction(period=5.0, actions=[arm]),
            TimerAction(period=6.0, actions=[gripper]),
        ]
    )
