#
#   Copyright (c)
#
#   The Verifiable & Control-Theoretic Robotics (VECTR) Lab
#   University of California, Los Angeles
#
#   Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez
#   Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    current_pkg = FindPackageShare('gicp_localization')

    # Set default arguments
    rviz = LaunchConfiguration('rviz', default='false')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic', default='/luminar_front/points')
    imu_topic = LaunchConfiguration('imu_topic', default='/gps_bot/imu')
    odom_topic = LaunchConfiguration('odom_topic', default='/odom')
    imu_only = LaunchConfiguration('imu_only', default='false')
    # Don't set default for map_path - let it come from config file

    # Static TF parameters (base_link -> lidar/sensor frame)
    # Identity transform: axis correction (Y flip) is handled in code via localization/flip_y param
    tf_x = LaunchConfiguration('tf_x', default='0.0')
    tf_y = LaunchConfiguration('tf_y', default='0.0')
    tf_z = LaunchConfiguration('tf_z', default='0.0')
    tf_qx = LaunchConfiguration('tf_qx', default='0.0')
    tf_qy = LaunchConfiguration('tf_qy', default='0.0')
    tf_qz = LaunchConfiguration('tf_qz', default='0.0')
    tf_qw = LaunchConfiguration('tf_qw', default='1.0')
    parent_frame = LaunchConfiguration('parent_frame', default='base_link')
    child_frame = LaunchConfiguration('child_frame', default='luminar_front')

    # Define arguments
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value=rviz,
        description='Launch RViz'
    )
    declare_pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value=pointcloud_topic,
        description='Pointcloud topic name'
    )
    declare_imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value=imu_topic,
        description='IMU topic name (for deskewing)'
    )
    declare_odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value=odom_topic,
        description='Odometry topic name (for initialization)'
    )
    declare_imu_only_arg = DeclareLaunchArgument(
        'imu_only',
        default_value=imu_only,
        description='If true, disable GICP and run IMU-only propagation'
    )
    declare_tf_x_arg = DeclareLaunchArgument(
        'tf_x', default_value=tf_x,
        description='Static TF translation X'
    )
    declare_tf_y_arg = DeclareLaunchArgument(
        'tf_y', default_value=tf_y,
        description='Static TF translation Y'
    )
    declare_tf_z_arg = DeclareLaunchArgument(
        'tf_z', default_value=tf_z,
        description='Static TF translation Z'
    )
    declare_tf_qx_arg = DeclareLaunchArgument(
        'tf_qx', default_value=tf_qx,
        description='Static TF rotation quaternion X'
    )
    declare_tf_qy_arg = DeclareLaunchArgument(
        'tf_qy', default_value=tf_qy,
        description='Static TF rotation quaternion Y'
    )
    declare_tf_qz_arg = DeclareLaunchArgument(
        'tf_qz', default_value=tf_qz,
        description='Static TF rotation quaternion Z'
    )
    declare_tf_qw_arg = DeclareLaunchArgument(
        'tf_qw', default_value=tf_qw,
        description='Static TF rotation quaternion W'
    )
    declare_parent_frame_arg = DeclareLaunchArgument(
        'parent_frame', default_value=parent_frame,
        description='Parent frame for static TF'
    )
    declare_child_frame_arg = DeclareLaunchArgument(
        'child_frame', default_value=child_frame,
        description='Child frame for static TF (sensor frame)'
    )
    declare_map_path_arg = DeclareLaunchArgument(
        'map_path', default_value='/media/terramaster/lc_three_lidar_map.pcd',
        description='Path to PCD map file for localization'
    )

    # Load parameters
    localization_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', 'localization.yaml'])

    # Static Transform Publisher (base_link -> lidar frame)
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_publisher',
        arguments=[
            tf_x, tf_y, tf_z,
            tf_qx, tf_qy, tf_qz, tf_qw,
            parent_frame, child_frame
        ],
        output='screen'
    )

    # Static Transform Publisher (base_link -> gps_bot/IMU frame)
    # From URDF gps_bottom_joint: xyz="1.63574 -0.075 -1.0075"
    # imu_bottom is co-located with gps_bottom (zero offset joint)
    static_tf_imu_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_gps_bot_publisher',
        arguments=[
            '1.63574', '-0.075', '-1.0075',  # Translation from base_link to gps_bot
            '0', '0', '0', '1',              # Identity rotation
            'base_link', 'gps_bot'
        ],
        output='screen'
    )

    # GICP Localization Node
    # map_path arg overrides the value in localization.yaml
    def make_localization_node(context):
        map_path_value = LaunchConfiguration('map_path').perform(context).strip()
        child_frame_value = LaunchConfiguration('child_frame').perform(context).strip()
        params = [
            localization_yaml_path,
            {'localization/lidar_frame': child_frame_value},
            {'localization/imu_only': LaunchConfiguration('imu_only')},
            {'localization/map_path': map_path_value},
        ]

        node = Node(
            package='gicp_localization',
            executable='gicp_localization_node',
            output='screen',
            parameters=params,
            remappings=[
                ('pointcloud', pointcloud_topic),  # Localization node transforms internally
                ('imu', imu_topic),
                ('odom', odom_topic),
                ('localized_pose', 'gicp/localization/pose'),
                ('localized_odom', 'gicp/localization/odom'),
                ('localized_path', 'gicp/localization/path'),
                ('aligned_cloud', 'gicp/localization/aligned_cloud'),
                ('map', 'gicp/localization/map'),
            ],
        )
        return [node]

    # RViz node
    rviz_config_path = PathJoinSubstitution([current_pkg, 'launch', 'localization.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='gicp_localization_rviz',
        arguments=['-d', rviz_config_path],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        declare_rviz_arg,
        declare_pointcloud_topic_arg,
        declare_imu_topic_arg,
        declare_odom_topic_arg,
        declare_imu_only_arg,
        declare_map_path_arg,
        declare_tf_x_arg,
        declare_tf_y_arg,
        declare_tf_z_arg,
        declare_tf_qx_arg,
        declare_tf_qy_arg,
        declare_tf_qz_arg,
        declare_tf_qw_arg,
        declare_parent_frame_arg,
        declare_child_frame_arg,
        static_tf_publisher,
        static_tf_imu_publisher,
        OpaqueFunction(function=make_localization_node),
        rviz_node
    ])
