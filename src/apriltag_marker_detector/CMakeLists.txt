cmake_minimum_required(VERSION 3.10)

project(apriltag_marker_detector)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	roscpp
    sensor_msgs
    tf2_eigen
    cv_bridge
    apriltag_msgs
    image_transport
)
find_package(OpenCV REQUIRED
    COMPONENTS
        core
        calib3d
        highgui
)
find_package(apriltag REQUIRED)

catkin_package()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

file(GLOB_RECURSE C_SRC "src/*.h" "src/*.c")
file(GLOB_RECURSE CXX_SRC "src/*.hpp" "src/*.cpp")
add_executable(apriltag_marker_detector ${C_SRC} ${CXX_SRC})

target_link_libraries(apriltag_marker_detector PUBLIC
    Threads::Threads
    opencv_core
    opencv_calib3d
    opencv_highgui
    apriltag::apriltag
    ${catkin_LIBRARIES}
)

target_include_directories(apriltag_marker_detector PUBLIC
	${catkin_INCLUDE_DIRS}
)

install(TARGETS apriltag_marker_detector
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
