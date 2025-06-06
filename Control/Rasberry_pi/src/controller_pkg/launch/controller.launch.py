from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    joy_params = os.path.join(
        get_package_share_directory("controller_pkg"),
        "config",
        "joy.yaml",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[joy_params],
    )

    #ps4_controller_node = Node(
     #   package="controller_pkg",
      #  executable="ps4_to_uart",
       # name="ps4_controller_translator",
    #)

    visualisation_node = Node(
        package="controller_pkg",
        executable="visualisation",
    )

    ld.add_action(joy_node)
    #ld.add_action(ps4_controller_node)
    ld.add_action(visualisation_node)
    return ld