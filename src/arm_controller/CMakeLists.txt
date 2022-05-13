cmake_minimum_required(VERSION 3.10)

project(arm_controller)

find_package(catkin REQUIRED COMPONENTS
	roscpp
    tf2_eigen
    tf2_ros
    actionlib_msgs
    nav_msgs
    apriltag_msgs
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

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

file(GLOB_RECURSE CXX_SRC "src/*.hpp" "src/*.cpp")
add_executable(${PROJECT_NAME} ${CXX_SRC})
target_link_libraries(${PROJECT_NAME} PUBLIC
    ${catkin_LIBRARIES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
	${catkin_INCLUDE_DIRS}
)

add_executable(test_action samples/test_action.cpp)
target_link_libraries(test_action PUBLIC
    ${catkin_LIBRARIES}
)
target_include_directories(test_action PUBLIC
	${catkin_INCLUDE_DIRS}
)

install(TARGETS ${PROJECT_NAME} test_action
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY launch rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
