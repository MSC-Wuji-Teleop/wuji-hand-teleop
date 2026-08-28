cmake_minimum_required(VERSION 3.8)
project(wujihand-urdf)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Wuji Hand 2 (Beta 2) URDFs, official vendor files. Provenance and the
# joint-order contract are in README.md.
#
# The retarget configs reach these through a source-tree-relative
# optimizer.urdf_path, not through the share/ install below, because
# --symlink-install resolves a config's __yaml_dir back to its source
# directory. The install rules are here for ROS2 consumers
# (robot_state_publisher, RViz), which need the meshes as well.
install(
  FILES
    wujihand_left.urdf
  DESTINATION share/${PROJECT_NAME}/urdf
  RENAME left.urdf
)

install(
  FILES
    wujihand_right.urdf
  DESTINATION share/${PROJECT_NAME}/urdf
  RENAME right.urdf
)

ament_package()

