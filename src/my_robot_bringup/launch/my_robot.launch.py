from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    turtle_sim_node = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

    turtle_controller_node = Node(
        package="my_robot_controller",
        executable="turtle_controller"
    )

    ld.add_action(turtle_sim_node)
    ld.add_action(turtle_controller_node)

    return ld