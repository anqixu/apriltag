cmake_minimum_required(VERSION 2.8.3)
project(apriltag_mit)

set(USE_ROS TRUE)

set(CMAKE_CXX_FLAGS "-Wall -std=c++0x -fPIC")
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules/)
message(STATUS "Blarg: ${CMAKE_MODULE_PATH}")
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

if(USE_ROS)
    find_package(catkin REQUIRED COMPONENTS roscpp)
    catkin_package(
        DEPENDS Eigen OpenCV
        CATKIN_DEPENDS roscpp
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        )
else(USE_ROS)
    message(STATUS "Source:  " ${PROJECT_SOURCE_DIR})
    message(STATUS "Binary:  " ${PROJECT_BINARY_DIR})
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
    link_directories(${PROJECT_SOURCE_DIR}/lib)
endif(USE_ROS)

file(GLOB CC_FILES ${PROJECT_SOURCE_DIR}/src/*.cc)
add_library(${PROJECT_NAME} SHARED ${CC_FILES})
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
    )

add_executable(test_apriltag_mit src/test/test_apriltag_mit.cpp)
target_link_libraries(test_apriltag_mit
    ${OpenCV_LIBRARIES}
    ${PROJECT_NAME}
    )
