cmake_minimum_required(VERSION 2.8.3)
project(apriltag_ros)

add_definitions("-std=c++11 -Wall")

find_package(catkin REQUIRED
    COMPONENTS roscpp image_transport cv_bridge camera_info_manager
               std_msgs sensor_msgs geometry_msgs message_generation
               dynamic_reconfigure
    )
find_package(OpenCV)

#add_message_files(DIRECTORY msg)
#generate_messages(DEPENDENCIES std_msgs)

# Dynamic reconfigure option
generate_dynamic_reconfigure_options(cfg/UsbCameraDyn.cfg)

catkin_package()

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

set(UMICH_DIR ../apriltag_umich)
set(MIT_DIR ../apriltag_mit)

include_directories(${UMICH_DIR})
link_directories(${UMICH_DIR}/lib)

include_directories(${MIT_DIR})
link_directories(${MIT_DIR}/lib)

# camera
add_library(usb_camera
    src/usb_camera.cpp
    )
target_link_libraries(usb_camera
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )
add_executable(camera_node
    src/camera_node.cpp
    )
target_link_libraries(camera_node
    ${catkin_LIBRARIES}
    usb_camera
    )

# apriltag
#add_executable(${PROJECT_NAME}_node
#    src/apriltag_ros_node.cpp
#    )
#target_link_libraries(${PROJECT_NAME}_node
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBRARIES}
#    apriltag_mit
#    apriltag_umich
#    )

add_dependencies(camera_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
    )
#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
