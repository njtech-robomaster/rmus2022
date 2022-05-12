cmake_minimum_required(VERSION 3.0.2)
project(rmus_cartographer)

find_package(catkin REQUIRED)

catkin_package()

install(
    DIRECTORY
        launch
        config
        rviz
        maps
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
    PROGRAMS
        scripts/write_map.sh
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
