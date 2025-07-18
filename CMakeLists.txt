cmake_minimum_required(VERSION 3.8)
project(bag_reading)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(CURL REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/bag_reader.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs rosbag2_transport cv_bridge)
target_link_libraries(${PROJECT_NAME}_node ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CURL_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
