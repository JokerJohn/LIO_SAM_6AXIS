cmake_minimum_required(VERSION 2.8.3)
project(lio_sam_6axis)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")
find_package(Boost REQUIRED COMPONENTS timer)
find_package(catkin REQUIRED COMPONENTS
        tf
        roscpp
        rospy
        cv_bridge
        pcl_conversions
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        message_generation
        visualization_msgs
        rosbag
        )

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
find_package(libxml2 QUIET)
if (NOT LIBXML2_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(Y REQUIRED libxml-2.0)
endif ()

add_message_files(
        DIRECTORY msg
        FILES
        cloud_info.msg
)

add_service_files(
        DIRECTORY srv
        FILES
        save_map.srv
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
        nav_msgs
        sensor_msgs
)

add_subdirectory(./ThirdParty/GeographicLib/)

catkin_package(
        INCLUDE_DIRS include
        DEPENDS PCL GTSAM

        CATKIN_DEPENDS
        std_msgs
        nav_msgs
        geometry_msgs
        sensor_msgs
        message_runtime
        message_generation
        visualization_msgs
)

# include directories
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
        ${Y_INCLUDE_DIRS}
        ${LIBXML2_INCLUDE_DIRS}
        ./ThirdParty/GeographicLib/include/
)

# link directories
link_directories(
        include
        ${PCL_LIBRARY_DIRS}
        ${OpenCV_LIBRARY_DIRS}
        ${GTSAM_LIBRARY_DIRS}
        ${LIBXML2_LIBRARY_DIR}
        ${Y_LIBRARY_DIRS}
)

###########
## Build ##
###########


# Gps odometry
add_executable(${PROJECT_NAME}_gpsOdometry src/simpleGpsOdom.cpp)
target_link_libraries(${PROJECT_NAME}_gpsOdometry ${catkin_LIBRARIES} ${LIBXML2_LIBRARIES} ${Y_LIBRARIES} libGeographic)

# Range Image Projection
add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
add_dependencies(${PROJECT_NAME}_imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${LIBXML2_LIBRARIES} ${Y_LIBRARIES})

# Feature Association
add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
add_dependencies(${PROJECT_NAME}_featureExtraction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_featureExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${LIBXML2_LIBRARIES} ${Y_LIBRARIES})

# Mapping Optimization
add_executable(${PROJECT_NAME}_mapOptmization src/mapOptmizationGps.cpp src/dataSaver.cpp)
add_dependencies(${PROJECT_NAME}_mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapOptmization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam ${LIBXML2_LIBRARIES} ${Y_LIBRARIES} libGeographic)

# IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam ${LIBXML2_LIBRARIES} ${Y_LIBRARIES})
