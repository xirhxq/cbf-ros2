from struct import pack
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def my_function(context):
    node_num = LaunchConfiguration('numbers').perform(context)

    num_list = range(int(node_num))

    ld = []

    for i in num_list:
        ld.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('mbzirc_ign'),
                        'launch',
                        'spawn.launch.py'
                    ])
                ]),
                launch_arguments={
                    'name': f'uav_{i + 1}',
                    'world': 'coast',
                    'model': 'mbzirc_quadrotor',
                    'x': str(-1490 - 2 * int(i)),
                    'y': '0',
                    'z': '4.3',
                    'R': '0',
                    'P': '0',
                    'Y': '0',
                    'slot0': 'mbzirc_hd_camera',
                    'slot0_rpy': '0 30 0',
                    'flightTime': '60',
                    'slot1': 'mbzirc_rf_long_range',
                    'slot1_rpy': '0 0 0'
                }.items()
            )
            
        )
    
    return ld


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "numbers", default_value=TextSubstitution(text="6")
        ),
        OpaqueFunction(function=my_function)
    ])
