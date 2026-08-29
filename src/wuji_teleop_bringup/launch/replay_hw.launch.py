"""Hardware replay bring-up, teleop-container side (spec_1 stages A-F).

Starts, in the TELEOP container:
  - wujihand_driver x2       (the only processes that touch hand USB;
                              serials from the gitignored wujihand_ik.yaml,
                              namespaces left_hand/right_hand)
  - wujihand_controller x2   (input_source=q20_topic, HARDWARE profile:
                              require_feedback true, watchdogs armed)
  - replay_publisher         (gated; no --force-sim here, ever)
  - supervisor               (gates, barrier, Layer-3 monitors, bag,
                              run directories under ~/wuji_runs)

The G1 arm node runs in ITS OWN container (CLAUDE.md rule), as its own
terminal on the host, from docker/:

    # Stage A (read-only, 7A):
    docker compose run --rm --name g1-world-output g1_world_output \
        ros2 launch g1_world_output g1_world_output.launch.py \
        read_only:=true mode:=joint_replay arm_type:=G1_29

    # Stages B+ (writing):
    docker compose run --rm --name g1-world-output g1_world_output \
        ros2 launch g1_world_output g1_world_output.launch.py \
        mode:=joint_replay arm_type:=G1_29 control_rate:=250.0

Per-stage operator sequences: docs/spec/spec_1_bringup.md.

Launch arguments:
    enable_hand_driver:=true|false   false = hands not connected (arm-track
                                     work); the q20 controllers then hold
                                     with no feedback and refuse approach
    hands:=true|false                false = do not start the hand
                                     controllers at all (pure arm track)
    record_bag:=true|false           supervisor rosbag recording (default on)
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from wuji_teleop_bringup.hand_defaults import (
    LEFT_HAND_SERIAL, RIGHT_HAND_SERIAL,
    LEFT_HAND_NAME, RIGHT_HAND_NAME,
    DRIVER_PUBLISH_RATE, DRIVER_FILTER_CUTOFF_FREQ, DRIVER_DIAGNOSTICS_RATE,
)


def generate_launch_description() -> LaunchDescription:
    hand_cfg = str(Path(get_package_share_directory('wujihand_output'))
                   / 'config' / 'wujihand_ik_q20.yaml')

    spawn_driver = IfCondition(LaunchConfiguration('enable_hand_driver'))
    spawn_hands = IfCondition(LaunchConfiguration('hands'))

    driver_nodes = [
        Node(
            package='wujihand_driver',
            executable='wujihand_driver_node',
            name='wujihand_driver',
            namespace=ns,
            parameters=[{
                'serial_number': ParameterValue(serial, value_type=str),
                'publish_rate': DRIVER_PUBLISH_RATE,
                'filter_cutoff_freq': DRIVER_FILTER_CUTOFF_FREQ,
                'diagnostics_rate': DRIVER_DIAGNOSTICS_RATE,
            }],
            output='screen',
            emulate_tty=True,
            condition=spawn_driver,
        )
        for ns, serial in ((LEFT_HAND_NAME, LEFT_HAND_SERIAL),
                           (RIGHT_HAND_NAME, RIGHT_HAND_SERIAL))
    ]

    controller_nodes = [
        Node(
            package='controller',
            executable='wujihand_controller',
            name=f'wujihand_controller_{side}',
            output='screen',
            emulate_tty=True,
            arguments=['--side', side, '--hand-name', name, '-c', hand_cfg],
            condition=spawn_hands,
        )
        for side, name in (('left', LEFT_HAND_NAME), ('right', RIGHT_HAND_NAME))
    ]

    return LaunchDescription([
        DeclareLaunchArgument('enable_hand_driver', default_value='true'),
        DeclareLaunchArgument('hands', default_value='true'),
        DeclareLaunchArgument('record_bag', default_value='true'),
        *driver_nodes,
        *controller_nodes,
        Node(
            package='replay',
            executable='replay_publisher',
            name='replay_publisher',
            output='screen',
        ),
        Node(
            package='replay',
            executable='supervisor',
            name='replay_supervisor',
            output='screen',
            parameters=[{
                'record_bag': ParameterValue(
                    LaunchConfiguration('record_bag'), value_type=bool),
            }],
        ),
    ])
