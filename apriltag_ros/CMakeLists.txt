cmake_minimum_required(VERSION 2.8.3)
project(apriltag_ros)

option(USE_OLD_YAML_INTERFACE "Use old yaml-cpp interface (for Ubuntu 12.04 / hydro compatibility)" OFF)

if(USE_OLD_YAML_INTERFACE)
add_definitions(-DUSE_OLD_YAML_INTERFACE)
else()
remove_definitions(-DUSE_OLD_YAML_INTERFACE)
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp image_transport cv_bridge camera_info_manager
    std_msgs sensor_msgs geometry_msgs message_generation
    apriltag_mit apriltag_umich)
find_package(OpenCV)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/../3rdparty/apriltag_mit/cmake_modules/)
find_package(Eigen3 REQUIRED)

add_message_files(DIRECTORY msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
CATKIN_DEPENDS roscpp image_transport cv_bridge camera_info_manager std_msgs sensor_msgs geometry_msgs message_runtime apriltag_mit apriltag_umich
)

add_definitions("-std=c++0x -Wall")

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# Apriltag node
add_executable(${PROJECT_NAME}_node
    src/apriltag_ros_node.cpp
    src/tag_yaml.cpp
    )
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    yaml-cpp
    )

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Map generator
add_executable(generate_map
  src/generate_map.cpp
  src/tag_yaml.cpp
)

target_link_libraries(generate_map
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)
