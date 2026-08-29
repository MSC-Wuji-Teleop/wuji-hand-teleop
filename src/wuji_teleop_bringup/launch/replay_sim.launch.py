"""Stage 0 all-sim replay bring-up (spec_1): collapses the teleop-container
terminals of the old Flow 3 into one launch.

Starts, in the TELEOP container:
  - replay_publisher        (gated; nothing publishes until run_ctl load)
  - wujihand_controller x2  (input_source=q20_topic, sim config: no driver)
  - supervisor              (gates, barrier, Layer-3 monitors, run dirs)
  - mujoco_visualizer       (29-DoF composed model; viewer:=false for
                             headless smoke runs)

The G1 arm node CANNOT run in this container (Pinocchio/CasADi + Unitree
SDK live in its own image; CLAUDE.md rule). Start it as its own terminal:

    cd docker && docker compose run --rm --name g1-world-output \
        g1_world_output ros2 launch g1_world_output g1_world_output.launch.py \
        dry_run:=true mode:=joint_replay arm_type:=G1_29 control_rate:=250.0

Then drive the run from a second teleop-container terminal:

    ros2 run replay condition_clip --method-dir \
        RobotSTAR_demos/samples/<sample>/GT --out-dir ~/wuji_clips
    ros2 run replay run_ctl load ~/wuji_clips/<...>/conditioned_clip_v1.npz \
        --speed 1.0
    ros2 run replay run_ctl arm
    ros2 run replay run_ctl start

Launch arguments:
    viewer:=true|false        MuJoCo window (false = headless smoke)
    record_bag:=false|true    supervisor rosbag recording (sim default off)
    force_sim:=false|true     publisher AND supervisor bypass load gates
                              (fault-injection drills only; never with
                              hardware attached)
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    viewer = LaunchConfiguration('viewer')
    record_bag = LaunchConfiguration('record_bag')
    force_sim = LaunchConfiguration('force_sim')

    hand_cfg = str(Path(get_package_share_directory('wujihand_output'))
                   / 'config' / 'wujihand_ik_q20_sim.yaml')
    mjcf = str(Path(get_package_share_directory('g1_wuji2_description'))
               / 'g1_29_wuji2_fixed.xml')
    # scripts/ are not installed console scripts; resolve the source file
    # through the workspace layout (src/ bind-mount, symlink install).
    viz_script = str(Path(get_package_share_directory('g1_world_output'))
                     .parents[3] / 'src' / 'output_devices' / 'g1_world_output'
                     / 'scripts' / 'mujoco_visualizer.py')

    hand_nodes = [
        Node(
            package='controller',
            executable='wujihand_controller',
            name=f'wujihand_controller_{side}',
            output='screen',
            arguments=['--side', side, '-c', hand_cfg],
        )
        for side in ('left', 'right')
    ]

    return LaunchDescription([
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument('record_bag', default_value='false'),
        DeclareLaunchArgument('force_sim', default_value='false'),

        Node(
            package='replay',
            executable='replay_publisher',
            name='replay_publisher',
            output='screen',
            arguments=['--force-sim'],
            condition=IfCondition(force_sim),
        ),
        Node(
            package='replay',
            executable='replay_publisher',
            name='replay_publisher',
            output='screen',
            condition=UnlessCondition(force_sim),
        ),
        *hand_nodes,
        Node(
            package='replay',
            executable='supervisor',
            name='replay_supervisor',
            output='screen',
            parameters=[{'record_bag': record_bag,
                         # sim: no wujihand driver, so Layer 3 must not
                         # demand hand_diagnostics liveness
                         'expect_hand_diagnostics': False,
                         # one launch arg arms BOTH bypasses: the publisher
                         # gets --force-sim, the supervisor logs-and-bypasses
                         # its load gates (drill 6d)
                         'force_sim': force_sim}],
        ),
        ExecuteProcess(
            cmd=['python3', viz_script, '--mjcf', mjcf],
            output='screen',
            condition=IfCondition(viewer),
        ),
    ])
