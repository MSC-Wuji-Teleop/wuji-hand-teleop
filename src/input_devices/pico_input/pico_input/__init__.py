"""
pico_input - PICO VR teleoperation input device

Public API:
  pico_input_node: PicoInputNode -- ROS2 node (incremental control)
  data_source: DataSource, LiveDataSource, RecordedDataSource -- data source abstraction
  xrobotoolkit_client: XRoboToolkitClient -- PICO SDK wrapper

Coordinate transforms: pico_input.transform_utils (sole authoritative implementation)
Configuration loading: pico_input.config_loader (config/robot_frames.yaml)

Data flow:
  PICO Tracker --> pico_input_node --> /left_arm_target_pose, /right_arm_target_pose
                                   --> /left_arm_elbow_direction, /right_arm_elbow_direction
                                   --> TF: world --> head, pico_*_wrist, pico_*_arm
"""
