cmake_minimum_required(VERSION 3.10)
project(g1_wuji2_description)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY meshes
    DESTINATION share/${PROJECT_NAME}
)

install(
    FILES g1_wuji2.urdf g1_wuji2.xml g1_wuji2_fixed.xml scene_g1_wuji2.xml
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
