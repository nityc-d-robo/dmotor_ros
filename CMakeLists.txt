cmake_minimum_required(VERSION 3.8)
project(dmotor_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(drobo_interfaces REQUIRED)
find_package(usb_connect REQUIRED)
find_package(motor_lib REQUIRED)

include_directories(
  include
)

add_executable(dmotor_ros
  src/main.cpp
)

target_link_libraries(dmotor_ros
  usb_connect
  md_lib
  smd_lib
  sd_lib
  sr_lib
  sm_lib
)

ament_target_dependencies(dmotor_ros
  rclcpp
  std_msgs
  drobo_interfaces
)

install(TARGETS
  dmotor_ros
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
