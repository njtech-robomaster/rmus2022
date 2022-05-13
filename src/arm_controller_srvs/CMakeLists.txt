cmake_minimum_required(VERSION 3.10)

project(arm_controller_srvs)

find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
)

add_action_files(
    DIRECTORY action
    FILES
        GraspPlace.action
)
generate_messages(
    DEPENDENCIES
        actionlib_msgs
        std_msgs
)

catkin_package(
    CATKIN_DEPENDS
        actionlib_msgs
)
