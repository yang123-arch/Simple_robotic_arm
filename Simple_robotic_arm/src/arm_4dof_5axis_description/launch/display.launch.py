import os
from xml.dom import Node as XmlNode

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    safe_init = {
        "joint_1": 0.0,
        "joint_2": -0.9,
        "joint_3": 0.8,
        "joint_4": 0.3,
        "joint_5": 0.3,
        "left_finger_joint": 0.018,
        "right_finger_joint": 0.018,
    }

    robot_description = ParameterValue(
        _robot_description_from_xacro(use_sim=False),
        value_type=str,
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"rate": 50.0, "zeros": safe_init}],
    )

    return LaunchDescription([rsp, jsp])
