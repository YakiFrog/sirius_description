#!/usr/bin/env python3

import os
import tempfile
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def convert_sdf_to_urdf(sdf_file_path, pkg_path):
    """SDFファイルをURDFに変換し、メッシュパスを修正"""
    try:
        # gz sdfコマンドでSDFを処理（モデル情報を抽出）
        result = subprocess.run(
            ['gz', 'sdf', '-p', sdf_file_path],
            capture_output=True,
            text=True,
            check=True
        )
        
        # 変換されたURDFを取得
        urdf_content = result.stdout
        
        # 相対パス（../urdf/meshes/）を絶対パスに置換
        urdf_content = urdf_content.replace(
            '../urdf/meshes/',
            f'file://{pkg_path}/urdf/meshes/'
        )
        
        # package:// 形式に変換（推奨）
        urdf_content = urdf_content.replace(
            f'file://{pkg_path}/urdf/meshes/',
            'package://sirius_description/urdf/meshes/'
        )
        
        return urdf_content
    except subprocess.CalledProcessError as e:
        print(f"Error converting SDF to URDF: {e}")
        return ""


def launch_setup(context, *args, **kwargs):
    # Get the path to the package
    pkg_path = get_package_share_directory('sirius_description')
    
    # Path to the SDF file
    sdf_file_path = os.path.join(pkg_path, 'sdf', 'sirius3.sdf')
    
    # Path to the RViz config file
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'sirius_robot.rviz')
    
    # SDFをURDFに変換（メッシュパスを修正）
    robot_description_content = convert_sdf_to_urdf(sdf_file_path, pkg_path)
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )
    
    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    return [
        robot_state_publisher,
        joint_state_publisher,
        rviz2
    ]


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        OpaqueFunction(function=launch_setup)
    ])
