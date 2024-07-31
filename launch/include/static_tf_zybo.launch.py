from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    """
    This function is by default called when executing ros2 launch ...
    This function must return a LaunchDescription object created from a list
    of launch_ros.actions
    """

    launch_desc_list = [
        # whether we are running a simulation or on a physical vehicle is controlled
        # through this launch argument.
        DeclareLaunchArgument(
            "use_sim_time", default_value="False",
                            description="Use simulation clock if True"
        )
    ]

    launch_arg_dict = {"use_sim_time": LaunchConfiguration("use_sim_time")}

    urdf = os.path.join(get_package_share_directory("rallycar"), "urdf", "rallycar_zybo.urdf")
    robot_desc = None
    with open(urdf, "r") as f:
        robot_desc = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "publish_frequency": 1.0,
                "ignore_timestamp": False,
                "use_tf_static": True,
                "robot_description": robot_desc,
            },
            launch_arg_dict
        ],
    )

    launch_desc_list.append(robot_state_publisher)

    '''
    multi_odom_tf_node = Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        arguments = ["--x", "0.325", "--frame-id", "odom", "--child-frame-id", "odom_frame"]
    )

    launch_desc_list.append(multi_odom_tf_node)
    '''

    return LaunchDescription(launch_desc_list)
