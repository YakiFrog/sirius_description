#!/usr/bin/env python3

import os
import subprocess
import shutil
import logging
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the path to the package
    pkg_path = get_package_share_directory('sirius_description')
    
    # Path to the SDF file (Main source of truth)
    robot_sdf_file_abs = os.path.join(pkg_path, 'sdf', 'sirius3.sdf')

    log = logging.getLogger('display_launch')

    def convert_sdf_to_urdf(sdf_file_path, pkg_path):
        """Convert an SDF file to URDF using available sdftool."""
        # prefer `gz sdf -p`
        cmd = ['gz', 'sdf', '-p', sdf_file_path]
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            urdf_content = result.stdout

            # update mesh paths
            urdf_content = urdf_content.replace('../urdf/meshes/', f'file://{pkg_path}/urdf/meshes/')
            urdf_content = urdf_content.replace(f'file://{pkg_path}/urdf/meshes/', 'package://sirius_description/urdf/meshes/')
            urdf_content = urdf_content.replace('../dae/', f'file://{pkg_path}/dae/')
            urdf_content = urdf_content.replace(f'file://{pkg_path}/dae/', 'package://sirius_description/dae/')

            return urdf_content
        except Exception as e:
            log.error('Failed to convert SDF to URDF: %s', e)
            raise

    # Path to the RViz config file
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'sirius_robot.rviz')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # Obtain robot_description by converting SDF
    if os.path.exists(robot_sdf_file_abs):
        robot_description_value = convert_sdf_to_urdf(robot_sdf_file_abs, pkg_path)
    else:
        raise FileNotFoundError('No SDF file found in package: {}'.format(pkg_path))

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description_value, value_type=str),
            'frame_prefix': 'sirius3/',
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Joint State Publisher (for manual control of joints in RViz)
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
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher,
        joint_state_publisher,
        rviz2
    ])
