cmake_minimum_required(VERSION 3.8)
project(bt_trees)

# make sure to include behaviortree_ros2 library
# this is a temporary fix where the library is installed using a bash script
# eventually we will want to implement this directly in CMake with FetchContent
execute_process(COMMAND ./include_behaviortrees_ros2.sh
                WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}) 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)

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

add_executable(bt_client src/add_two_ints_bt_service.cpp)
ament_target_dependencies(bt_client rclcpp behaviortree_cpp behaviortree_ros2 example_interfaces)

install(TARGETS
     bt_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()