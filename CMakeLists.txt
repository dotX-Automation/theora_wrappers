cmake_minimum_required(VERSION 3.8)
project(theora_wrappers)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fvisibility=hidden)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Theora publisher
add_library(theorapub SHARED
  src/publisher.cpp)
target_compile_features(theorapub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_definitions(theorapub PRIVATE "THEORA_WRAPPERS_BUILDING_LIBRARY")
target_include_directories(theorapub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  theorapub
  image_transport
  rclcpp
  sensor_msgs
  std_srvs)

# Theora subscriber
add_library(theorasub SHARED
  src/subscriber.cpp)
target_compile_features(theorasub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_definitions(theorasub PRIVATE "THEORA_WRAPPERS_BUILDING_LIBRARY")
target_include_directories(theorasub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  theorasub
  image_transport
  rclcpp
  sensor_msgs
  std_srvs)

# Libraries installation
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS theorapub theorasub
  EXPORT theora_wrappersTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

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

# Export all dependencies and library targets for this package
ament_export_targets(theora_wrappersTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  image_transport
  rclcpp
  sensor_msgs
  std_srvs)

ament_package()
