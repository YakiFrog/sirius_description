#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # パッケージのパスを取得
    pkg_share = FindPackageShare('sirius_description')
    
    # 環境SDFファイルのパスを設定
    world_sdf_file = PathJoinSubstitution([
        pkg_share,
        'worlds',
        'sirius_world.sdf'
    ])
    
    # ロボットSDFファイルのパスを設定
    robot_sdf_file = PathJoinSubstitution([
        pkg_share,
        'sdf',
        'sirius3.sdf'
    ])
    
    # URDFファイルのパスを設定
    urdf_file = PathJoinSubstitution([
        pkg_share,
        'urdf',
        'sirius3.urdf.xacro'
    ])
    
    # Rvizファイルのパスを設定
    rviz_config = PathJoinSubstitution([
        pkg_share,
        'rviz',
        'sirius_robot.rviz'
    ])
    
    # Launch引数の定義
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # 1. 環境（world）のGazeboシミュレーション起動
    gazebo_world = ExecuteProcess(
        cmd=['gz', 'sim', world_sdf_file],
        output='screen',
        shell=False
    )
    
    # 2. ロボットをスポーン（ros_gz_simのcreateノード使用）
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', robot_sdf_file,
            '-name', 'sirius3',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.0825',
            '-Y', '1.5708'  # 左に90度回転（ラジアンで指定、1.5708 ≒ π/2）
        ],
        output='screen'
    )
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': Command(['xacro ', urdf_file, ' prefix:=sirius3/'])
            }
        ],
        output='screen'
    )
    
    # TFのros_gz_bridge
    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/sirius3/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
        ],
        remappings=[
            ('/model/sirius3/tf', '/tf')
        ],
        output='screen'
    )
    
    # Odometryのros_gz_bridge
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/sirius3/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ],
        remappings=[
            ('/model/sirius3/odom', '/odom')
        ],
        output='screen'
    )
    
    # Twistのros_gz_bridge
    twist_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen'
    )
    
    # Joint Stateのros_gz_bridge
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'
        ],
        output='screen'
    )
    
    # LiDARのros_gz_bridge
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )

    # LiDAR2のros_gz_bridge
    lidar2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/hokuyo_scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )

    # LiDAR (Velodyne)用のTF Static Publisher
    lidar_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.0', '0.0', '0.85',  # x, y, z (位置)
            '0', '0', '0',     # roll, pitch, yaw (姿勢)
            'sirius3/base_link',  # 親フレーム
            'sirius3/base_link/lidar_link'      # 子フレーム
        ],
        output='screen'
    )

    # Hokuyo LiDAR用のTF Static Publisher
    hokuyo_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.25', '0.0', '0.17',  # x, y, z (位置)
            '0', '0', '0',     # roll, pitch, yaw (姿勢)
            'sirius3/base_link',  # 親フレーム
            'sirius3/base_link/lidar_link2'      # 子フレーム
        ],
        output='screen'
    )

    # Teleopキーボードコントロール
    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e'
    )
    
    # Rviz2の起動
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch引数
        use_sim_time_arg,
        
        # 1. 環境（ワールド）を最初に起動
        gazebo_world,

        # 2. 3秒後にロボットをスポーン
        TimerAction(
            period=3.0,
            actions=[spawn_robot]
        ),

        # 3. 5秒後にブリッジ類を開始
        TimerAction(
            period=5.0,
            actions=[
                tf_bridge,
                odom_bridge,
                twist_bridge,
                robot_state_publisher,
                joint_state_bridge,
                lidar_bridge,
                lidar2_bridge,
                lidar_tf_publisher,
                hokuyo_tf_publisher
            ]
        ),
        
        # 4. 7秒後にteleopを起動
        TimerAction(
            period=7.0,
            actions=[
                teleop_keyboard
            ]
        ),

        # 5. 8秒後にRVizを起動
        TimerAction(
            period=8.0,
            actions=[
                rviz2
            ]
        )
    ])
