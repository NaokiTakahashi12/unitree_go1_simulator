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
    return [
        launch.actions.DeclareLaunchArgument(
            'namespace',
            default_value = [''],
            description = 'Namespace (string)'
        ),
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value = ['true'],
            description = 'Use simulation time (boolean)'
        ),
        launch.actions.DeclareLaunchArgument(
            'world_name',
            default_value = ['checker_ground_plane_with_obstacle'],
            description = 'Simulation world name (string)'
        ),
        launch.actions.DeclareLaunchArgument(
            'debug',
            default_value=['false'],
            description='Enable debug output (boolean)'
        )
    ]

def generate_launch_nodes():
    output = 'screen'

    this_pkg_share_dir = get_package_share_directory('unitree_go1_simulator')
    go1_description_share_dir = get_package_share_directory('unitree_go1_description')
    ros_gz_utils_share_dir = get_package_share_directory('ros_gz_utils')

    simulator_namespace = 'simulator'
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    world_name = launch.substitutions.LaunchConfiguration('world_name')

    # Mesh file
    resource_package_name = go1_description_share_dir
    # URDF file
    model_resource_path = os.path.join(
        go1_description_share_dir,
        'urdf'
    )
    # this package world direcotry
    world_path = os.path.join(this_pkg_share_dir, 'worlds', 'ignition')

    return [
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.AnyLaunchDescriptionSource(
                os.path.join(
                    go1_description_share_dir,
                    'launch',
                    'unitree_go1.launch.py'
                )
            ),
            launch_arguments = {
                'namespace': launch.substitutions.LaunchConfiguration('namespace'),
                'joint_state_publisher_config_file': [
                    os.path.join(
                        this_pkg_share_dir,
                        'config',
                        'joint_state.yaml'
                    )
                ],
                'use_sim_time': use_sim_time,
                'enable_ros2_control': 'true',
                'use_real_hardware': 'false',
                'ignition_gazebo': 'true'
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.AnyLaunchDescriptionSource(
                os.path.join(
                    ros_gz_utils_share_dir,
                    'launch',
                    'ignition_gazebo.launch.py'
                )
            ),
            launch_arguments = {
                'namespace': simulator_namespace,
                'use_sim_time': use_sim_time,
                'world_name': world_name,
                'resource_package_name': resource_package_name,
                'model_resource_path': model_resource_path,
                'world_path': world_path,
                'debug': launch.substitutions.LaunchConfiguration('debug')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.AnyLaunchDescriptionSource(
                os.path.join(
                    ros_gz_utils_share_dir,
                    'launch',
                    'ignition_spawn.launch.py'
                )
            ),
            launch_arguments = {
                'namespace': simulator_namespace,
                'use_sim_time': use_sim_time,
                'world_name': world_name,
                'robot_model_from_topic': 'true',
                'robot_description_topic': '/robot_description',
                'spawn_position_z': '0.75',
                'spawn_orientation_p': '-0.01'
            }.items()
        ),
        launch.actions.GroupAction(actions = [
            launch_ros.actions.PushRosNamespace(
                namespace = simulator_namespace
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    os.path.join(
                        ros_gz_utils_share_dir,
                        'launch',
                        'ignition_bridge.launch.py'
                    )
                ),
                launch_arguments = {
                    'with_stf': 'false',
                    'ign_topic': ['/world/', world_name, '/model/unitree_go1/joint_state'],
                    'ros_topic': 'joint_states',
                    'convert_args': 'sensor_msgs/msg/JointState[ignition.msgs.Model',
                    'bridge_node_name': 'unitree_go1_joint_state_bridge'
                }.items()
            )
        ]),
        launch.actions.GroupAction(actions = [
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    os.path.join(
                        ros_gz_utils_share_dir,
                        'launch',
                        'ignition_bridge.launch.py'
                    )
                ),
                launch_arguments = {
                    'with_stf': 'true',
                    'ign_frame_id': ['unitree_go1/base_link/camera_face_camera'],
                    'ros_frame_id': 'camera_face',
                    'ign_topic': [
                        '/world/', world_name, '/model/unitree_go1',
                        '/link/base_link/sensor/camera_face_camera/camera_info'
                    ],
                    'ros_topic': 'face_camera/camera_info',
                    'convert_args': 'sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'
                }.items()
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    os.path.join(
                        ros_gz_utils_share_dir,
                        'launch',
                        'ignition_image_bridge.launch.py'
                    )
                ),
                launch_arguments = {
                    'with_stf': 'false',
                    'ign_topic': [
                        '/world/', world_name, '/model/unitree_go1',
                        '/link/base_link/sensor/camera_face_camera/depth_image'
                    ],
                    'ros_topic': 'face_camera/depth_image'
                }.items()
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    os.path.join(
                        ros_gz_utils_share_dir,
                        'launch',
                        'ignition_bridge.launch.py'
                    )
                ),
                launch_arguments = {
                    'with_stf': 'false',
                    'ign_topic': [
                        '/world/', world_name, '/model/unitree_go1',
                        '/link/base_link/sensor/camera_face_camera/depth_image/points'
                    ],
                    'ros_topic': 'face_camera/points',
                    'convert_args': 'sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
                }.items()
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    os.path.join(
                        ros_gz_utils_share_dir,
                        'launch',
                        'ignition_bridge.launch.py'
                    )
                ),
                launch_arguments = {
                    'with_stf': 'true',
                    'ign_frame_id': ['unitree_go1/base_link/camera_left_camera'],
                    'ros_frame_id': 'camera_left',
                    'ign_topic': [
                        '/world/', world_name, '/model/unitree_go1',
                        '/link/base_link/sensor/camera_left_camera/camera_info'
                    ],
                    'ros_topic': 'left_camera/camera_info',
                    'convert_args': 'sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'
                }.items()
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    os.path.join(
                        ros_gz_utils_share_dir,
                        'launch',
                        'ignition_image_bridge.launch.py'
                    )
                ),
                launch_arguments = {
                    'with_stf': 'false',
                    'ign_topic': [
                        '/world/', world_name, '/model/unitree_go1',
                        '/link/base_link/sensor/camera_left_camera/depth_image'
                    ],
                    'ros_topic': 'left_camera/depth_image'
                }.items()
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    os.path.join(
                        ros_gz_utils_share_dir,
                        'launch',
                        'ignition_bridge.launch.py'
                    )
                ),
                launch_arguments = {
                    'with_stf': 'false',
                    'ign_topic': [
                        '/world/', world_name, '/model/unitree_go1',
                        '/link/base_link/sensor/camera_left_camera/depth_image/points'
                    ],
                    'ros_topic': 'left_camera/points',
                    'convert_args': 'sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
                }.items()
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    os.path.join(
                        ros_gz_utils_share_dir,
                        'launch',
                        'ignition_bridge.launch.py'
                    )
                ),
                launch_arguments = {
                    'with_stf': 'true',
                    'ign_frame_id': ['unitree_go1/base_link/camera_right_camera'],
                    'ros_frame_id': 'camera_right',
                    'ign_topic': [
                        '/world/', world_name, '/model/unitree_go1',
                        '/link/base_link/sensor/camera_right_camera/camera_info'
                    ],
                    'ros_topic': 'right_camera/camera_info',
                    'convert_args': 'sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'
                }.items()
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    os.path.join(
                        ros_gz_utils_share_dir,
                        'launch',
                        'ignition_image_bridge.launch.py'
                    )
                ),
                launch_arguments = {
                    'with_stf': 'false',
                    'ign_topic': [
                        '/world/', world_name, '/model/unitree_go1',
                        '/link/base_link/sensor/camera_right_camera/depth_image'
                    ],
                    'ros_topic': 'right_camera/depth_image'
                }.items()
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    os.path.join(
                        ros_gz_utils_share_dir,
                        'launch',
                        'ignition_bridge.launch.py'
                    )
                ),
                launch_arguments = {
                    'with_stf': 'false',
                    'ign_topic': [
                        '/world/', world_name, '/model/unitree_go1',
                        '/link/base_link/sensor/camera_right_camera/depth_image/points'
                    ],
                    'ros_topic': 'right_camera/points',
                    'convert_args': 'sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
                }.items()
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    os.path.join(
                        ros_gz_utils_share_dir,
                        'launch',
                        'ignition_bridge.launch.py'
                    )
                ),
                launch_arguments = {
                    'with_stf': 'true',
                    'ign_frame_id': ['unitree_go1/base_link/camera_chin_camera'],
                    'ros_frame_id': 'camera_chin',
                    'ign_topic': [
                        '/world/', world_name, '/model/unitree_go1',
                        '/link/base_link/sensor/camera_chin_camera/camera_info'
                    ],
                    'ros_topic': 'chin_camera/camera_info',
                    'convert_args': 'sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'
                }.items()
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    os.path.join(
                        ros_gz_utils_share_dir,
                        'launch',
                        'ignition_image_bridge.launch.py'
                    )
                ),
                launch_arguments = {
                    'with_stf': 'false',
                    'ign_topic': [
                        '/world/', world_name, '/model/unitree_go1',
                        '/link/base_link/sensor/camera_chin_camera/depth_image'
                    ],
                    'ros_topic': 'chin_camera/depth_image'
                }.items()
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    os.path.join(
                        ros_gz_utils_share_dir,
                        'launch',
                        'ignition_bridge.launch.py'
                    )
                ),
                launch_arguments = {
                    'with_stf': 'false',
                    'ign_topic': [
                        '/world/', world_name, '/model/unitree_go1',
                        '/link/base_link/sensor/camera_chin_camera/depth_image/points'
                    ],
                    'ros_topic': 'chin_camera/points',
                    'convert_args': 'sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
                }.items()
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    os.path.join(
                        ros_gz_utils_share_dir,
                        'launch',
                        'ignition_bridge.launch.py'
                    )
                ),
                launch_arguments = {
                    'with_stf': 'true',
                    'ign_frame_id': ['unitree_go1/base_link/camera_rearDown_camera'],
                    'ros_frame_id': 'camera_rearDown',
                    'ign_topic': [
                        '/world/', world_name, '/model/unitree_go1',
                        '/link/base_link/sensor/camera_rearDown_camera/camera_info'
                    ],
                    'ros_topic': 'rearDown_camera/camera_info',
                    'convert_args': 'sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'
                }.items()
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    os.path.join(
                        ros_gz_utils_share_dir,
                        'launch',
                        'ignition_image_bridge.launch.py'
                    )
                ),
                launch_arguments = {
                    'with_stf': 'false',
                    'ign_topic': [
                        '/world/', world_name, '/model/unitree_go1',
                        '/link/base_link/sensor/camera_rearDown_camera/depth_image'
                    ],
                    'ros_topic': 'rearDown_camera/depth_image'
                }.items()
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    os.path.join(
                        ros_gz_utils_share_dir,
                        'launch',
                        'ignition_bridge.launch.py'
                    )
                ),
                launch_arguments = {
                    'with_stf': 'false',
                    'ign_topic': [
                        '/world/', world_name, '/model/unitree_go1',
                        '/link/base_link/sensor/camera_rearDown_camera/depth_image/points'
                    ],
                    'ros_topic': 'rearDown_camera/points',
                    'convert_args': 'sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
                }.items()
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    os.path.join(
                        ros_gz_utils_share_dir,
                        'launch',
                        'ignition_bridge.launch.py'
                    )
                ),
                launch_arguments = {
                    'with_stf': 'true',
                    'ign_frame_id': 'unitree_go1/base_link/imu_sensor',
                    'ros_frame_id': 'imu_link',
                    'ign_topic': [
                        '/world/', world_name, '/model/unitree_go1',
                        '/link/base_link/sensor/imu_sensor/imu'
                    ],
                    'ros_topic': 'imu/data_raw',
                    'convert_args': 'sensor_msgs/msg/Imu[ignition.msgs.IMU',
                }.items()
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    os.path.join(
                        ros_gz_utils_share_dir,
                        'launch',
                        'ignition_bridge.launch.py'
                    )
                ),
                launch_arguments = {
                    'with_stf': 'true',
                    'ign_frame_id': 'unitree_go1/base_link/ultrasound_face_sensor',
                    'ros_frame_id': 'ultraSound_face',
                    'ign_topic': [
                        '/world/', world_name, '/model/unitree_go1',
                        '/link/base_link/sensor/ultrasound_face_sensor/scan/points'
                    ],
                    'ros_topic': 'face_ultrasound/points',
                    'convert_args': 'sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                }.items()
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    os.path.join(
                        ros_gz_utils_share_dir,
                        'launch',
                        'ignition_bridge.launch.py'
                    )
                ),
                launch_arguments = {
                    'with_stf': 'true',
                    'ign_frame_id': 'unitree_go1/base_link/ultrasound_left_sensor',
                    'ros_frame_id': 'ultraSound_left',
                    'ign_topic': [
                        '/world/', world_name, '/model/unitree_go1',
                        '/link/base_link/sensor/ultrasound_left_sensor/scan/points'
                    ],
                    'ros_topic': 'left_ultrasound/points',
                    'convert_args': 'sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                }.items()
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    os.path.join(
                        ros_gz_utils_share_dir,
                        'launch',
                        'ignition_bridge.launch.py'
                    )
                ),
                launch_arguments = {
                    'with_stf': 'true',
                    'ign_frame_id': 'unitree_go1/base_link/ultrasound_right_sensor',
                    'ros_frame_id': 'ultraSound_right',
                    'ign_topic': [
                        '/world/', world_name, '/model/unitree_go1',
                        '/link/base_link/sensor/ultrasound_right_sensor/scan/points'
                    ],
                    'ros_topic': 'right_ultrasound/points',
                    'convert_args': 'sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                }.items()
            ),
        ])
    ]

