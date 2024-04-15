import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

def get_share_file(package_name: str, *args: str) -> str:
    """Convert package-relative path to absolute path. Any additional args
    will be appended to the package_name, separated by '/'.

    Args:
        package_name (str): Package name.

    Returns:
        os.path: Absolute path.
    """
    return os.path.join(get_package_share_directory(package_name), *args)


def generate_launch_description():
    """
    This function is by default called when executing ros2 launch ...
    This function must return a LaunchDescription object created from a list
    of launch_ros.actions
    """
    hardware_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_share_file('rallycar', 'launch', 'rallycar_hardware.launch.py')
        )
    )

    # we can create nodes and put into the launch description as well
    scanmatching_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_share_file('rallycar', 'launch', 'scanmatching_slam.launch.py')
        )
    )

    print_usage_instructions = LogInfo(msg="To start a keyboard teleop session, "\
        "keep this session running, open a new terminal, enter this workspace, "\
        "and run:\n"\
        "\tsource ./install/setup.bash\n"\
        "\tros2 run rallycar rally_teleop_keyboard.py\n\n"\
        "To save the resultant map, keep this session running, open a new terminal "\
        "and run:\n"\
        "\tros2 run map_server map_saver_cli -f your_map_file_name\n")

    return LaunchDescription([
        hardware_driver_launch,
        scanmatching_slam_launch,
        print_usage_instructions,
    ])
