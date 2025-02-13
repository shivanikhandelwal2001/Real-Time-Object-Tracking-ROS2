cmake_minimum_required(VERSION 3.8)
project(object_tracking_viz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Eigen3 REQUIRED)

# include cpp directory
include_directories(include)

# create cpp executables
add_executable(image_publisher src/image_publisher.cpp)
ament_target_dependencies(image_publisher rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(object_tracking src/object_tracking.cpp)
ament_target_dependencies(object_tracking rclcpp std_msgs nlohmann_json Eigen3)

# install cpp executables
install(TARGETS
  image_publisher
  object_tracking
  DESTINATION lib/${PROJECT_NAME}
)

# install py modules
ament_python_install_package(${PROJECT_NAME})

# install py executables
install(PROGRAMS
  scripts/image_subscriber.py
  scripts/object_visualization.py
  DESTINATION lib/${PROJECT_NAME}
)

# add launch 
install (DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# add config 
install (DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
