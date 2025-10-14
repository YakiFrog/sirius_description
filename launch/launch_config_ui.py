#!/usr/bin/env python3
"""
Sirius シミュレーション起動設定UI
PySide6を使用したGUIで起動パラメータを設定
"""

import os
import sys
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                               QHBoxLayout, QCheckBox, QPushButton, QComboBox, 
                               QLabel, QGroupBox)
from PySide6.QtCore import Qt
from ament_index_python.packages import get_package_share_directory


class LaunchConfigUI(QMainWindow):
    """シミュレーション起動設定用のGUIウィンドウ"""
    
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
        """UIの初期化"""
        self.setWindowTitle('Sirius シミュレーション起動設定')
        self.setGeometry(100, 100, 500, 600)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # ワールド選択セクション
        layout.addWidget(self._create_world_selection_group())
        
        # コンポーネント選択セクション
        layout.addWidget(self._create_components_group())
        
        # 起動ボタン
        layout.addLayout(self._create_launch_button())
        
    def _create_world_selection_group(self):
        """ワールド選択グループの作成"""
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
        
        return world_group
    
    def _create_components_group(self):
        """コンポーネント選択グループの作成"""
        components_group = QGroupBox("起動するコンポーネント")
        components_layout = QVBoxLayout()
        
        self.checkboxes = {}
        
        # ロボット関連
        components_layout.addWidget(self._create_section_label("ロボット:"))
        self._add_checkbox(components_layout, 'spawn_robot', "ロボットをスポーン", True)
        self._add_checkbox(components_layout, 'robot_state_publisher', "Robot State Publisher", True)
        
        # ブリッジ関連
        components_layout.addWidget(self._create_section_label("\nROS-Gazebo ブリッジ:"))
        self._add_checkbox(components_layout, 'tf_bridge', "TF Bridge", True)
        self._add_checkbox(components_layout, 'odom_bridge', "Odometry Bridge", True)
        self._add_checkbox(components_layout, 'twist_bridge', "Twist Bridge (cmd_vel)", True)
        self._add_checkbox(components_layout, 'joint_state_bridge', "Joint State Bridge", True)
        self._add_checkbox(components_layout, 'clock_bridge', "Clock Bridge", True)
        
        # センサー関連
        components_layout.addWidget(self._create_section_label("\nセンサー:"))
        self._add_checkbox(components_layout, 'lidar_bridge', "LiDAR Bridge (/scan)", True)
        self._add_checkbox(components_layout, 'lidar2_bridge', "LiDAR2 Bridge (/hokuyo_scan)", True)
        self._add_checkbox(components_layout, 'imu_bridge', "IMU Bridge", True)
        self._add_checkbox(components_layout, 'velodyne_bridge', "Velodyne Bridge (3D LiDAR)", True)
        
        # ツール関連
        components_layout.addWidget(self._create_section_label("\nツール:"))
        self._add_checkbox(components_layout, 'teleop', "Teleopキーボードコントロール", True)
        self._add_checkbox(components_layout, 'rviz', "RViz2", True)
        
        components_group.setLayout(components_layout)
        return components_group
    
    def _create_section_label(self, text):
        """セクションラベルの作成"""
        label = QLabel(text)
        label.setStyleSheet("font-weight: bold;")
        return label
    
    def _add_checkbox(self, layout, key, text, checked):
        """チェックボックスを追加"""
        checkbox = QCheckBox(text)
        checkbox.setChecked(checked)
        self.checkboxes[key] = checkbox
        layout.addWidget(checkbox)
    
    def _create_launch_button(self):
        """起動ボタンレイアウトの作成"""
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
        
        return button_layout
    
    def on_launch(self):
        """起動ボタンがクリックされた時の処理"""
        # 設定を保存
        self.config['world_file'] = self.world_combo.currentText()
        for key, checkbox in self.checkboxes.items():
            self.config[key] = checkbox.isChecked()
        
        self.launched = True
        self.close()
    
    def get_config(self):
        """設定を取得（起動された場合のみ）"""
        return self.config if self.launched else None


def show_launch_config_ui():
    """
    UIを表示して設定を取得する関数
    
    Returns:
        dict or None: ユーザーが選択した設定、またはキャンセルされた場合はNone
    """
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    
    ui = LaunchConfigUI()
    ui.show()
    app.exec()
    
    return ui.get_config()
