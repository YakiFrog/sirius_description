#!/usr/bin/env python3

import os
import sys
import subprocess
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                               QHBoxLayout, QCheckBox, QPushButton, QComboBox, 
                               QLabel, QGroupBox)
from PySide6.QtCore import Qt

class LaunchConfigUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.config = {
            'world_file': 'sirius_world.sdf',
            'spawn_robot': True,
            'robot_state_publisher': True,
            'tf_bridge': True,
            'odom_bridge': True,
            'twist_bridge': True,
            'joint_state_bridge': True,
            'lidar_bridge': True,
            'lidar2_bridge': True,
            'imu_bridge': True,
            'velodyne_bridge': True,
            'clock_bridge': True,
            'teleop': True,
            'rviz': True
        }
        self.launched = False
        self.init_ui()
        
    def init_ui(self):
        self.setWindowTitle('Sirius シミュレーション起動設定')
        self.setGeometry(100, 100, 500, 600)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # ワールド選択
        world_group = QGroupBox("ワールド選択")
        world_layout = QVBoxLayout()
        
        world_label = QLabel("起動するワールドファイル:")
        world_layout.addWidget(world_label)
        
        self.world_combo = QComboBox()
        # worldsディレクトリからSDFファイルを検索
        pkg_path = get_package_share_directory('sirius_description')
        worlds_path = os.path.join(pkg_path, 'worlds')
        if os.path.exists(worlds_path):
            sdf_files = [f for f in os.listdir(worlds_path) if f.endswith('.sdf')]
            self.world_combo.addItems(sdf_files)
        else:
            self.world_combo.addItem('sirius_world.sdf')
        
        world_layout.addWidget(self.world_combo)
        world_group.setLayout(world_layout)
        layout.addWidget(world_group)
        
        # コンポーネント選択
        components_group = QGroupBox("起動するコンポーネント")
        components_layout = QVBoxLayout()
        
        self.checkboxes = {}
        
        # ロボット関連
        robot_label = QLabel("ロボット:")
        robot_label.setStyleSheet("font-weight: bold;")
        components_layout.addWidget(robot_label)
        
        self.checkboxes['spawn_robot'] = QCheckBox("ロボットをスポーン")
        self.checkboxes['spawn_robot'].setChecked(True)
        components_layout.addWidget(self.checkboxes['spawn_robot'])
        
        self.checkboxes['robot_state_publisher'] = QCheckBox("Robot State Publisher")
        self.checkboxes['robot_state_publisher'].setChecked(True)
        components_layout.addWidget(self.checkboxes['robot_state_publisher'])
        
        # ブリッジ関連
        bridge_label = QLabel("\nROS-Gazebo ブリッジ:")
        bridge_label.setStyleSheet("font-weight: bold;")
        components_layout.addWidget(bridge_label)
        
        self.checkboxes['tf_bridge'] = QCheckBox("TF Bridge")
        self.checkboxes['tf_bridge'].setChecked(True)
        components_layout.addWidget(self.checkboxes['tf_bridge'])
        
        self.checkboxes['odom_bridge'] = QCheckBox("Odometry Bridge")
        self.checkboxes['odom_bridge'].setChecked(True)
        components_layout.addWidget(self.checkboxes['odom_bridge'])
        
        self.checkboxes['twist_bridge'] = QCheckBox("Twist Bridge (cmd_vel)")
        self.checkboxes['twist_bridge'].setChecked(True)
        components_layout.addWidget(self.checkboxes['twist_bridge'])
        
        self.checkboxes['joint_state_bridge'] = QCheckBox("Joint State Bridge")
        self.checkboxes['joint_state_bridge'].setChecked(True)
        components_layout.addWidget(self.checkboxes['joint_state_bridge'])
        
        self.checkboxes['clock_bridge'] = QCheckBox("Clock Bridge")
        self.checkboxes['clock_bridge'].setChecked(True)
        components_layout.addWidget(self.checkboxes['clock_bridge'])
        
        # センサー関連
        sensor_label = QLabel("\nセンサー:")
        sensor_label.setStyleSheet("font-weight: bold;")
        components_layout.addWidget(sensor_label)
        
        self.checkboxes['lidar_bridge'] = QCheckBox("LiDAR Bridge (/scan)")
        self.checkboxes['lidar_bridge'].setChecked(True)
        components_layout.addWidget(self.checkboxes['lidar_bridge'])
        
        self.checkboxes['lidar2_bridge'] = QCheckBox("LiDAR2 Bridge (/hokuyo_scan)")
        self.checkboxes['lidar2_bridge'].setChecked(True)
        components_layout.addWidget(self.checkboxes['lidar2_bridge'])
        
        self.checkboxes['imu_bridge'] = QCheckBox("IMU Bridge")
        self.checkboxes['imu_bridge'].setChecked(True)
        components_layout.addWidget(self.checkboxes['imu_bridge'])
        
        self.checkboxes['velodyne_bridge'] = QCheckBox("Velodyne Bridge (3D LiDAR)")
        self.checkboxes['velodyne_bridge'].setChecked(True)
        components_layout.addWidget(self.checkboxes['velodyne_bridge'])
        
        # ツール関連
        tools_label = QLabel("\nツール:")
        tools_label.setStyleSheet("font-weight: bold;")
        components_layout.addWidget(tools_label)
        
        self.checkboxes['teleop'] = QCheckBox("Teleopキーボードコントロール")
        self.checkboxes['teleop'].setChecked(True)
        components_layout.addWidget(self.checkboxes['teleop'])
        
        self.checkboxes['rviz'] = QCheckBox("RViz2")
        self.checkboxes['rviz'].setChecked(True)
        components_layout.addWidget(self.checkboxes['rviz'])
        
        components_group.setLayout(components_layout)
        layout.addWidget(components_group)
        
        # 起動ボタン
        button_layout = QHBoxLayout()
        
        self.launch_button = QPushButton("起動")
        self.launch_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-size: 16px;
                font-weight: bold;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
        """)
        self.launch_button.clicked.connect(self.on_launch)
        button_layout.addWidget(self.launch_button)
        
        layout.addLayout(button_layout)
        
    def on_launch(self):
        # 設定を保存
        self.config['world_file'] = self.world_combo.currentText()
        for key, checkbox in self.checkboxes.items():
            self.config[key] = checkbox.isChecked()
        
        self.launched = True
        self.close()
    
    def get_config(self):
        return self.config if self.launched else None


def show_ui():
    """UIを表示して設定を取得"""
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    
    ui = LaunchConfigUI()
    ui.show()
    app.exec()
    
    return ui.get_config()


def convert_sdf_to_urdf(sdf_file_path, pkg_path):
    """SDFファイルをURDFに変換し、メッシュパスを修正"""
    result = subprocess.run(
        ['gz', 'sdf', '-p', sdf_file_path],
        capture_output=True,
        text=True,
        check=True
    )
    
    urdf_content = result.stdout
    
    urdf_content = urdf_content.replace(
        '../urdf/meshes/',
        f'file://{pkg_path}/urdf/meshes/'
    )
    
    urdf_content = urdf_content.replace(
        f'file://{pkg_path}/urdf/meshes/',
        'package://sirius_description/urdf/meshes/'
    )
    
    return urdf_content


def generate_launch_description_with_config(context, *args, **kwargs):
    """設定に基づいてLaunchDescriptionを生成"""
    # UIから設定を取得
    config = show_ui()
    
    if config is None:
        print("起動がキャンセルされました")
        return []
    
    # パッケージのパスを取得
    pkg_share = FindPackageShare('sirius_description')
    pkg_path = get_package_share_directory('sirius_description')
    
    # 選択されたワールドファイルのパスを設定
    world_sdf_file = PathJoinSubstitution([
        pkg_share,
        'worlds',
        config['world_file']
    ])
    
    # ロボットSDFファイルのパスを設定
    robot_sdf_file = PathJoinSubstitution([
        pkg_share,
        'sdf',
        'sirius3.sdf'
    ])
    
    robot_sdf_file_abs = os.path.join(pkg_path, 'sdf', 'sirius3.sdf')
    
    # Rvizファイルのパスを設定
    rviz_config = PathJoinSubstitution([
        pkg_share,
        'rviz',
        'sirius_robot.rviz'
    ])
    
    actions = []
    
    # 1. 環境（world）のGazeboシミュレーション起動（常に起動）
    gazebo_world = ExecuteProcess(
        cmd=['gz', 'sim', world_sdf_file],
        output='screen',
        shell=False
    )
    actions.append(gazebo_world)
    
    # 2. Clock Bridge（選択された場合、最優先）
    if config['clock_bridge']:
        clock_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        )
        actions.append(TimerAction(period=1.0, actions=[clock_bridge]))
    
    # 3. ロボットをスポーン
    timer_actions_3sec = []
    if config['spawn_robot']:
        spawn_robot = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', robot_sdf_file,
                '-name', 'sirius3',
                '-x', '0.0',
                '-y', '0.0', 
                '-z', '0.0825',
                '-Y', '1.5708'
            ],
            output='screen'
        )
        timer_actions_3sec.append(spawn_robot)
    
    if timer_actions_3sec:
        actions.append(TimerAction(period=3.0, actions=timer_actions_3sec))
    
    # 4. ブリッジ類とrobot_state_publisher（5秒後）
    timer_actions_5sec = []
    
    if config['tf_bridge']:
        tf_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/model/sirius3/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'],
            remappings=[('/model/sirius3/tf', '/tf')],
            output='screen'
        )
        timer_actions_5sec.append(tf_bridge)
    
    if config['odom_bridge']:
        odom_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/model/sirius3/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
            remappings=[('/model/sirius3/odom', '/odom')],
            output='screen'
        )
        timer_actions_5sec.append(odom_bridge)
    
    if config['twist_bridge']:
        twist_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
            output='screen'
        )
        timer_actions_5sec.append(twist_bridge)
    
    if config['robot_state_publisher']:
        robot_description_content = convert_sdf_to_urdf(robot_sdf_file_abs, pkg_path)
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {
                    'use_sim_time': True,
                    'robot_description': robot_description_content,
                    'frame_prefix': 'sirius3/'
                }
            ],
            output='screen'
        )
        timer_actions_5sec.append(robot_state_publisher)
    
    if config['joint_state_bridge']:
        joint_state_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'],
            output='screen'
        )
        timer_actions_5sec.append(joint_state_bridge)
    
    if config['lidar_bridge']:
        lidar_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
            output='screen'
        )
        timer_actions_5sec.append(lidar_bridge)
    
    if config['lidar2_bridge']:
        lidar2_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/hokuyo_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
            output='screen'
        )
        timer_actions_5sec.append(lidar2_bridge)
    
    if config['imu_bridge']:
        imu_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
            output='screen'
        )
        timer_actions_5sec.append(imu_bridge)
    
    if config['velodyne_bridge']:
        velodyne_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/velodyne_points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
            remappings=[('/velodyne_points/points', '/velodyne_points')],
            output='screen'
        )
        timer_actions_5sec.append(velodyne_bridge)
    
    if timer_actions_5sec:
        actions.append(TimerAction(period=5.0, actions=timer_actions_5sec))
    
    # 5. Teleop（7秒後）
    if config['teleop']:
        teleop_keyboard = Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e'
        )
        actions.append(TimerAction(period=7.0, actions=[teleop_keyboard]))
    
    # 6. RViz（8秒後）
    if config['rviz']:
        rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
        actions.append(TimerAction(period=8.0, actions=[rviz2]))
    
    return actions


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        OpaqueFunction(function=generate_launch_description_with_config)
    ])
