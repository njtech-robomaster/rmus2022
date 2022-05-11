cmake_minimum_required(VERSION 3.0.2)
project(ep_driver)
find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(
    PROGRAMS
        scripts/ep_comm_node.py
        scripts/odom_pub.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY
        launch
        urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
