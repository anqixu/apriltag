cmake_minimum_required(VERSION 2.8.3)
project(apriltag_ros)

add_definitions("-std=c++11 -Wall")

find_package(catkin REQUIRED
    COMPONENTS roscpp image_transport
               cv_bridge std_msgs
               sensor_msgs geometry_msgs
               message_generation dynamic_reconfigure
    )
find_package(OpenCV)

add_message_files(DIRECTORY msg
    FILES Apriltag.msg ApriltagArray.msg
    )
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    CATKIN_DEPENDS message_runtime
    )

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

set(MIT_DIR ../apriltag_mit)
include_directories(${MIT_DIR})
link_directories(${MIT_DIR}/lib)

# pose_estimator
add_library(pose_estimator
    src/pose_estimator.cpp
    )
target_link_libraries(pose_estimator
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    apriltag_mit
    )

# node
add_executable(${PROJECT_NAME}_node
    src/${PROJECT_NAME}_node.cpp
    )
target_link_libraries(${PROJECT_NAME}_node
    pose_estimator
    )
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
