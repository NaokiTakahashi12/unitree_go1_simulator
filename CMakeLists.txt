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

if(NOT "$ENV{GZ_VERSION}" STREQUAL "")
    set(GAZEBO_DISTRO $ENV{GZ_VERSION})
elseif(NOT "$ENV{IGN_VERSION}" STREQUAL "")
    set(GAZEBO_DISTRO $ENV{IGN_VERSION})
else()
    message(FATAL_ERROR "Please set GZ_VERSION or IGN_VERSION enviroment variable")
endif()
message(STATUS "Set GAZEBO_DISTRO is ${GAZEBO_DISTRO}")

if("${GAZEBO_DISTRO}" STREQUAL "garden")
    set(GAZEBO_CLI_PREFIX gz)
else()
    set(GAZEBO_CLI_PREFIX ign)
endif()

add_custom_command(
    OUTPUT
        $ENV{HOME}/.gz/fuel/fuel.gazebosim.org/movai/models/pallet
    COMMAND
        ${GAZEBO_CLI_PREFIX} fuel download -u https://fuel.gazebosim.org/1.0/MovAi/models/pallet
    VERBATIM
)
add_custom_command(
    OUTPUT
        $ENV{HOME}/.gz/fuel/fuel.gazebosim.org/movai/models/shelf
    COMMAND
        ${GAZEBO_CLI_PREFIX} fuel download -u https://fuel.gazebosim.org/1.0/MovAi/models/shelf
    VERBATIM
)
add_custom_command(
    OUTPUT
        $ENV{HOME}/.gz/fuel/fuel.gazebosim.org/movai/models/shelf_big
    COMMAND
        ${GAZEBO_CLI_PREFIX} fuel download -u https://fuel.gazebosim.org/1.0/MovAi/models/shelf_big
    VERBATIM
)
add_custom_command(
    OUTPUT
        $ENV{HOME}/.gz/fuel/fuel.gazebosim.org/openrobotics/models/warehouse
    COMMAND
        ${GAZEBO_CLI_PREFIX} fuel download -u https://fuel.gazebosim.org/1.0/OpenRobotics/models/Warehouse
    VERBATIM
)
add_custom_command(
    OUTPUT
        $ENV{HOME}/.gz/fuel/fuel.gazebosim.org/mingfei/models/actor
    COMMAND
        ${GAZEBO_CLI_PREFIX} fuel download -u https://fuel.gazebosim.org/1.0/Mingfei/models/actor
    VERBATIM
)
add_custom_target(fuel_download_target
    DEPENDS
        $ENV{HOME}/.gz/fuel/fuel.gazebosim.org/movai/models/pallet
        $ENV{HOME}/.gz/fuel/fuel.gazebosim.org/movai/models/shelf
        $ENV{HOME}/.gz/fuel/fuel.gazebosim.org/movai/models/shelf_big
        $ENV{HOME}/.gz/fuel/fuel.gazebosim.org/openrobotics/models/warehouse
        $ENV{HOME}/.gz/fuel/fuel.gazebosim.org/mingfei/models/actor
)
add_dependencies(unitree_go1_simulator
    fuel_download_target
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
