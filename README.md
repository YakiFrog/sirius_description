# Sirius Robot Description

Sirius robot description package for RViz2 visualization and Gazebo simulation.

## Package Structure

```
sirius_description/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package metadata and dependencies
├── README.md              # This file
├── config/                # Configuration files
├── launch/                # Launch files
│   └── display.launch.py  # Launch file for RViz visualization
├── rviz/                  # RViz configuration files
│   └── sirius_robot.rviz  # RViz configuration for Sirius robot
├── sdf/                   # Gazebo SDF files
│   └── sirius_robot.sdf   # Gazebo model definition
├── urdf/                  # URDF files
│   └── sirius_robot.urdf.xacro  # Robot description in URDF format
└── worlds/                # Gazebo world files
```

## Robot Specifications

- **Base dimensions**: 0.6m x 0.4m x 0.15m
- **Weight**: 2.0kg (base)
- **Wheel radius**: 0.12m
- **Wheel width**: 0.06m
- **Drive type**: Differential drive
- **Caster wheels**: Front and rear for stability

## Usage

### 1. Build the package

```bash
cd ~/sirius_jazzy_ws
colcon build --packages-select sirius_description
source install/setup.bash
```

### 2. Launch RViz visualization

```bash
ros2 launch sirius_description display.launch.py
```

### 3. Launch with GUI for joint control

```bash
ros2 launch sirius_description display.launch.py use_gui:=true
```

### 4. Launch for simulation

```bash
ros2 launch sirius_description display.launch.py use_sim_time:=true
```

## Launch Parameters

- `use_sim_time`: Set to 'true' when using with Gazebo simulation (default: 'false')
- `use_gui`: Set to 'true' to launch joint state publisher GUI (default: 'true')

## Robot Frame Structure

```
sirius/base_footprint
└── sirius/base_link
    ├── sirius/left_wheel
    ├── sirius/right_wheel
    ├── sirius/front_caster
    └── sirius/rear_caster
```

## Dependencies

- `urdf`
- `xacro`
- `robot_state_publisher`
- `rviz2`
- `joint_state_publisher`
- `joint_state_publisher_gui`
- `ros_gz_bridge`
- `tf2_ros`
- `teleop_twist_keyboard`
- `launch`
- `launch_ros`

## Notes

- The robot uses a differential drive configuration with two main wheels and two caster wheels for support
- Frame prefix is set to "sirius/" to avoid naming conflicts
- The robot description is compatible with both RViz2 visualization and Gazebo simulation
