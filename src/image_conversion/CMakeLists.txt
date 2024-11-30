cmake_minimum_required(VERSION 3.8)
project(image_conversion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)


add_executable(image_converter_node src/image_conversion.cpp) 
ament_target_dependencies(image_converter_node rclcpp sensor_msgs OpenCV cv_bridge example_interfaces)

install(TARGETS
  image_converter_node
  DESTINATION lib/${PROJECT_NAME})

  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )

ament_package()
