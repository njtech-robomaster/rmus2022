cmake_minimum_required(VERSION 3.10)

project(odom_republisher)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
    tf2_ros
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

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
