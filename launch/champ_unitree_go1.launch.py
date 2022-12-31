#!/usr/bin/env -S python3

import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros

def generate_launch_description():
    return launch.LaunchDescription(
        generate_declare_launch_arguments()
        + generate_launch_nodes()
    )

def generate_declare_launch_arguments():
    this_pkg_share_dir = get_package_share_directory('unitree_go1_simulator')

    return [
        launch.actions.DeclareLaunchArgument(
            'namespace',
            default_value = [''],
            description = 'Namespace (string)'
        ),
        launch.actions.DeclareLaunchArgument(
            'robot_model_file',
            default_value = ['unitree_go1.urdf.xacro'],
            description = 'Robot model file (string)'
        ),
        launch.actions.DeclareLaunchArgument(
            'robot_model_path',
            default_value = [
                os.path.join(
                    get_package_share_directory('unitree_go1_description'),
                    'urdf'
                )
            ],
            description = 'Robot model file path (string)'
        ),
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value = ['true'],
            description = 'Use simulation time (boolean)'
        ),
        launch.actions.DeclareLaunchArgument(
            'ros2_control_config_file',
            default_value = [
                os.path.join(
                    get_package_share_directory('unitree_go1_description'),
                    'config',
                    'ros2_controllers.yaml'
                )
            ],
            description = 'ros2 controller config file path (string)'
        ),
        launch.actions.DeclareLaunchArgument(
            'use_real_hardware',
            default_value = ['false'],
            description = 'Using real hardware control (boolean)',
        ),
        launch.actions.DeclareLaunchArgument(
            'ignition_gazebo',
            default_value = ['true'],
            description = 'Using ignition gazebo (boolean)',
        ),
        launch.actions.DeclareLaunchArgument(
            'champ_config_file_path',
            default_value = [
                os.path.join(
                    this_pkg_share_dir,
                    'config',
                    'unitree_go1_champ.yaml'
                )
            ],
            description = 'champ configuration file path (string)'
        )
    ]

def generate_launch_nodes():
    output = 'screen'

    urdf_file = launch.substitutions.PathJoinSubstitution([
        launch.substitutions.LaunchConfiguration('robot_model_path'),
        launch.substitutions.LaunchConfiguration('robot_model_file')
    ])
    robot_description = launch.substitutions.Command([
            'xacro ',
            ' use_real_hardware:=',
            launch.substitutions.LaunchConfiguration('use_real_hardware'),
            ' ignition_gazebo:=',
            launch.substitutions.LaunchConfiguration('ignition_gazebo'),
            ' ros2_control_config_file:=',
            launch.substitutions.LaunchConfiguration('ros2_control_config_file'),
            ' ',
            urdf_file
    ])
    urdf_parameter = {
        'urdf': robot_description
    }

    use_sim_time = {'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')}

    return [
        launch_ros.actions.Node(
            package = 'champ_base',
            executable = 'quadruped_controller_node',
            name = 'quadruped_controller_node',
            output = output,
            parameters = [
                use_sim_time,
                urdf_parameter,
                launch.substitutions.LaunchConfiguration('champ_config_file_path')
            ],
            remappings = [
                ('cmd_vel/smooth', 'cmd_vel'),
                ('joint_states', '~/joint_states')
            ]
        ),
        launch_ros.actions.Node(
            package = 'unitree_go1_simulator',
            executable = 'joint_trajectory_update_timestamp_node',
            name = 'joint_trajectory_update_timestamp',
            output = output,
            parameters = [
                use_sim_time,
            ]
        )
    ]

