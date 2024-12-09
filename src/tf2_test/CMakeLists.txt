cmake_minimum_required(VERSION 3.8)
project(tf2_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${camera_info_manager_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  /usr/include/gazebo-11/gazebo/msgs/
)

add_executable(tf2_broadcaster src/tf2_broadcaster.cpp)
add_executable(tf2_broadcaster_ass src/tf2_broadcaster_ass.cpp)

ament_target_dependencies(
    tf2_broadcaster
    geometry_msgs
    nav_msgs
    rclcpp
    tf2
    tf2_ros
)

ament_target_dependencies(
    tf2_broadcaster_ass
    geometry_msgs
    nav_msgs
    rclcpp
    tf2
    tf2_ros
)

install(
    TARGETS tf2_broadcaster tf2_broadcaster_ass
    DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
