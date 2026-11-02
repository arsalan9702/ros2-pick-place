"""Launch the pick_place node with the Panda's MoveIt configuration.

demo.launch.py (moveit_resources_panda_moveit_config) does not publish the SRDF on a topic,
so the node needs robot_description, robot_description_semantic and kinematics as parameters.
Run demo.launch.py first, then:  ros2 launch pick_place_demo pick_place.launch.py cycles:=4
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    cycles_arg = DeclareLaunchArgument(
        "cycles", default_value="3", description="Number of pick-and-place cycles"
    )

    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .to_moveit_configs()
    )

    pick_place_node = Node(
        package="pick_place_demo",
        executable="pick_place",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"cycles": LaunchConfiguration("cycles")},
        ],
    )

    return LaunchDescription([cycles_arg, pick_place_node])
