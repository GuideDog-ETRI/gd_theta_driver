cmake_minimum_required(VERSION 3.5)
project(theta_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED system thread timer)
find_package(libuvc REQUIRED)
find_package(pluginlib REQUIRED)

find_package(PkgConfig)
pkg_check_modules(GSTREAMER_1_0 gstreamer-1.0)
pkg_check_modules(GSTREAMER_1_0_APP gstreamer-app-1.0)

set(dependencies
    rclcpp
    sensor_msgs
    cv_bridge
    image_transport
    OpenCV
    pluginlib
    GSTREAMER_1_0
    GSTREAMER_1_0_APP
)

include_directories(
  include
  3rd/libuvc-theta-sample/gst
  ${GSTREAMER_1_0_INCLUDE_DIRS}
  ${GSTREAMER_1_0_APP_INCLUDE_DIRS}
)

add_executable(theta_driver_node src/theta_driver_node.cpp src/theta_driver_main.cpp 3rd/libuvc-theta-sample/gst/thetauvc.c)
target_link_libraries(theta_driver_node
  ${GSTREAMER_1_0_LIBRARIES}
  ${GSTREAMER_1_0_APP_LIBRARIES}
  LibUVC::UVCShared
)
ament_target_dependencies(theta_driver_node ${dependencies})

target_include_directories(theta_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  theta_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
