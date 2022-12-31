cmake_minimum_required(VERSION 3.15)
project(unitree_go1_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(unitree_go1_simulator SHARED
    src/joint_trajectory_update_timestamp_node.cpp
)
rclcpp_components_register_node(unitree_go1_simulator
    PLUGIN "unitree_go1_simulator::JointTrajectoryUpdateTimestampNode"
    EXECUTABLE joint_trajectory_update_timestamp_node
)
install(
    TARGETS unitree_go1_simulator
    EXPORT unitree_go1_simulator
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY
        launch
        worlds
        config
        rviz
    DESTINATION
        "share/${PROJECT_NAME}"
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
