cmake_minimum_required(VERSION 3.10)

project(rmus_decision)

find_package(catkin REQUIRED COMPONENTS
	roscpp
    tf2_eigen
    tf2_ros
    arm_controller_srvs
    nav_msgs
    apriltag_msgs
    move_base_msgs
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

catkin_package()

file(GLOB_RECURSE CXX_SRC "src/*.hpp" "src/*.cpp")
add_executable(${PROJECT_NAME} ${CXX_SRC})
target_link_libraries(${PROJECT_NAME} PUBLIC
    ${catkin_LIBRARIES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
	${catkin_INCLUDE_DIRS}
)

add_executable(print_2d_pose samples/print_2d_pose.cpp)
target_link_libraries(print_2d_pose PUBLIC
    ${catkin_LIBRARIES}
)
target_include_directories(print_2d_pose PUBLIC
	${catkin_INCLUDE_DIRS}
    src
)

install(TARGETS ${PROJECT_NAME} print_2d_pose
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY config launch rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
