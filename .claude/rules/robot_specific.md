---
description: Robot-Specific Standards (URDF, TF2, Navigation)
---

# Robot-Specific Standards

## URDF/Xacro Best Practices

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="robot">

  <!-- Properties -->
  <xacro:property name="wheel_radius" value="0.05"/>
  <xacro:property name="wheel_width" value="0.02"/>

  <!-- Macros -->
  <xacro:macro name="wheel" params="prefix reflect">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="0 ${reflect * 0.1} 0" rpy="${-pi/2} 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry><box size="0.3 0.2 0.1"/></geometry>
    </visual>
  </link>

  <!-- Wheels -->
  <xacro:wheel prefix="left" reflect="1"/>
  <xacro:wheel prefix="right" reflect="-1"/>

</robot>
```

## TF2 Frame Tree

```
map
 └── odom
      └── base_footprint
           └── base_link
                ├── lidar_link
                ├── camera_link
                │    └── camera_optical
                ├── imu_link
                └── left_wheel
```

## Navigation Stack Configuration

```yaml
# nav2_params.yaml
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      plugin: dwb_core::DWBLocalPlanner
      max_vel_x: 0.5
      max_vel_theta: 1.0

planner_server:
  ros__parameters:
    GridBased:
      plugin: nav2_navfn_planner/NavfnPlanner
      tolerance: 0.5
      use_astar: true
```

## Sensor Integration

### Python Example

```python
# Sensor data publishing pattern
from sensor_msgs.msg import LaserScan, Imu, Image

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')

        # LiDAR publisher
        self.lidar_pub = self.create_publisher(
            LaserScan, 'scan',
            qos_profile_sensor_data
        )

        # Camera publisher
        self.camera_pub = self.create_publisher(
            Image, 'camera/image_raw',
            qos_profile_sensor_data
        )

        # IMU publisher
        self.imu_pub = self.create_publisher(
            Imu, 'imu/data',
            qos_profile_sensor_data
        )
```

## Motor Control Pattern

### Python Example

```python
from geometry_msgs.msg import Twist

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel',
            self.cmd_callback, 10
        )

        # Hardware interface
        self._left_motor = Motor(pin=17)
        self._right_motor = Motor(pin=18)

    def cmd_callback(self, msg: Twist):
        # Differential drive kinematics
        linear = msg.linear.x
        angular = msg.angular.z

        left_vel = linear - angular * self.wheel_base / 2
        right_vel = linear + angular * self.wheel_base / 2

        self._left_motor.set_velocity(left_vel)
        self._right_motor.set_velocity(right_vel)
```
