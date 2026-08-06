"""Headless spawn: UAVs without camera/RF sensors.

Identical to spawn.launch.py but omits slot0 (mbzirc_hd_camera) and slot1
(mbzirc_rf_long_range). CBF control only consumes pose groundtruth and
publishes cmd_vel, so it does not need any rendering sensors. Running without
sensors avoids the Ignition Ogre2 render-engine initialization that segfaults
in a headless (no GPU/EGL) container.

Use with: ros2 launch cbf-ros2 spawn_headless.launch.py numbers:=$UAV_NUM
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def spawn_entities(context):
    node_num = LaunchConfiguration('numbers').perform(context)
    num_list = range(int(node_num))

    print(f"[spawn_headless] Spawning {node_num} UAVs (no sensors)")

    actions = []
    for i in num_list:
        actions.append(
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
                    # No slot0 (camera) and no slot1 (rfsensor): headless-friendly.
                    'flightTime': '60',
                }.items()
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "numbers", default_value=TextSubstitution(text="6")
        ),
        OpaqueFunction(function=spawn_entities)
    ])
