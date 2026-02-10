# WHEELTEC MID360 SLAM导航系统

基于ROS2 Humble的麦克纳姆轮机器人SLAM建图与导航系统，使用Livox MID360激光雷达。

## 系统要求

- Ubuntu 22.04
- ROS2 Humble
- Livox MID360激光雷达
- WHEELTEC麦克纳姆轮底盘

## 硬件连接

### 激光雷达网络配置
- MID360 IP地址: `192.168.2.148`
- 本机IP地址: `192.168.2.50`

确保本机网卡配置为静态IP `192.168.2.50`，与MID360在同一网段。

### 底盘串口连接
- 默认串口: `/dev/ttyUSB0`
- 波特率: `115200`

## 安装依赖

### 1. 安装ROS2包

```bash
# Nav2 和 SLAM
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-pointcloud-to-laserscan

# 机器人描述
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-xacro

# DDS配置（推荐使用Cyclone DDS）
sudo apt install ros-humble-rmw-cyclonedds-cpp

# 遥控
sudo apt install ros-humble-teleop-twist-keyboard
```

### 2. 安装Livox SDK2

```bash
cd ~/
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install
```

### 3. 安装livox_ros_driver2

```bash
cd ~/nav_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd livox_ros_driver2
./build.sh humble
```

### 4. 编译工作空间

```bash
cd ~/nav_ws
colcon build --symlink-install
source install/setup.bash
```

## 需要配置的参数

### ⚠️ 重要：机器人参数配置

在使用前，**必须**根据实际机器人修改以下参数：

#### 1. 机器人尺寸参数

**文件**: `wheeltec_driver/include/wheeltec_driver/robot_params.hpp`

| 参数 | 描述 | 默认值 | 单位 |
|-----|------|-------|-----|
| `ROBOT_WHEEL_SEPARATION` | 轮距（左右轮之间的距离） | 0.30 | 米 |
| `ROBOT_WHEEL_BASE` | 轴距（前后轮之间的距离） | 0.25 | 米 |
| `ROBOT_WHEEL_DIAMETER` | 轮子直径 | 0.10 | 米 |

或者通过YAML配置：

**文件**: `wheeltec_driver/config/driver_params.yaml`

```yaml
wheeltec_driver_node:
  ros__parameters:
    wheel_separation: 0.30   # 轮距 (米)
    wheel_base: 0.25         # 轴距 (米)
    wheel_diameter: 0.10     # 轮子直径 (米)
```

#### 2. 串口配置

**文件**: `wheeltec_driver/config/driver_params.yaml`

```yaml
wheeltec_driver_node:
  ros__parameters:
    serial_port: "/dev/ttyUSB0"  # 串口设备路径
    baudrate: 115200             # 波特率
```

#### 3. 激光雷达安装位置

**文件**: `wheeltec_description/urdf/robot.urdf.xacro`

```xml
<!-- 激光雷达安装位置（相对于base_link） -->
<xacro:property name="lidar_x" value="0.0"/>   <!-- X方向偏移 (米) -->
<xacro:property name="lidar_y" value="0.0"/>   <!-- Y方向偏移 (米) -->
<xacro:property name="lidar_z" value="0.20"/>  <!-- Z方向偏移 (米) -->
```

#### 4. IMU参数

**文件**: `wheeltec_driver/config/driver_params.yaml`

```yaml
wheeltec_driver_node:
  ros__parameters:
    # 根据实际IMU型号调整
    imu_accel_scale: 0.00239  # 加速度计比例因子
    imu_gyro_scale: 0.00106   # 陀螺仪比例因子
```

#### 5. 机器人尺寸（用于导航避障）

**文件**: `wheeltec_bringup/config/nav2_params.yaml`

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 0.22  # 机器人半径（用于碰撞避免）

global_costmap:
  global_costmap:
    ros__parameters:
      robot_radius: 0.22  # 机器人半径
```

## 使用说明

### 1. 配置环境

建议在 `~/.bashrc` 中添加：

```bash
source /opt/ros/humble/setup.bash
source ~/nav_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 2. 建图模式

启动完整系统进行建图：

```bash
# 启动建图模式
ros2 launch wheeltec_bringup bringup.launch.py mode:=slam

# 在另一个终端，使用键盘遥控小车移动建图
ros2 run wheeltec_bringup teleop_keyboard.py
```

键盘控制说明：
- `i`: 前进
- `k`: 停止
- `,`: 后退
- `j`: 左转
- `l`: 右转
- `u`: 左前
- `o`: 右前
- `m`: 左后
- `.`: 右后

保存地图（一键保存两种格式）：

```bash
# 在地图建好后，使用脚本一键保存（同时保存占据栅格地图和序列化地图）
~/nav_ws/src/wheeltec_bringup/scripts/save_map.sh my_home
```

> 也可以手动分步保存：
> ```bash
> ros2 run nav2_map_server map_saver_cli -f ~/nav_ws/maps/my_home
> ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/bigtruck/nav_ws/maps/my_home_serialized'}"
> ```

### 3. 导航模式

使用已建好的地图进行导航（slam_toolbox 自动定位）：

```bash
# 启动导航模式（指定占据栅格地图和序列化地图）
ros2 launch wheeltec_bringup bringup.launch.py mode:=nav \
  map:=~/nav_ws/maps/my_home.yaml \
  serialized_map:=~/nav_ws/maps/my_home_serialized
```

- `map` — 占据栅格地图（`.yaml`），提供给 costmap 使用
- `serialized_map` — slam_toolbox 序列化地图（不含扩展名），用于自动定位

系统启动后，slam_toolbox 通过 scan matching 自动确定机器人位置和朝向。
在 RViz 中点击 **2D Goal Pose** 按钮设置导航目标点，机器人将自动规划路径并导航。

### 4. 单独启动各模块

```bash
# 仅启动底盘驱动
ros2 launch wheeltec_driver driver.launch.py

# 仅启动激光雷达
ros2 launch wheeltec_bringup lidar.launch.py

# 仅启动机器人描述
ros2 launch wheeltec_description description.launch.py

# 仅启动SLAM
ros2 launch wheeltec_bringup slam.launch.py

# 仅启动导航
ros2 launch wheeltec_bringup navigation.launch.py map:=/path/to/map.yaml
```

## 话题列表

| 话题 | 类型 | 描述 |
|-----|------|------|
| `/cmd_vel` | geometry_msgs/Twist | 速度指令输入 |
| `/odom` | nav_msgs/Odometry | 里程计输出 |
| `/scan` | sensor_msgs/LaserScan | 2D激光扫描 |
| `/livox/lidar` | sensor_msgs/PointCloud2 | 3D点云 |
| `/imu/data_raw` | sensor_msgs/Imu | IMU原始数据 |
| `/battery_voltage` | std_msgs/Float32 | 电池电压 |
| `/map` | nav_msgs/OccupancyGrid | 地图 |
| `/global_costmap/costmap` | nav_msgs/OccupancyGrid | 全局代价地图 |
| `/local_costmap/costmap` | nav_msgs/OccupancyGrid | 局部代价地图 |
| `/plan` | nav_msgs/Path | 规划路径 |

## TF变换树

```
map
 └── odom (由slam_toolbox或amcl发布)
      └── base_link (由wheeltec_driver发布)
           ├── laser_frame (由robot_state_publisher发布)
           ├── imu_link (由robot_state_publisher发布)
           └── *_wheel_link (由robot_state_publisher发布)
```

## 故障排除

### 1. 串口权限问题

```bash
sudo chmod 666 /dev/ttyUSB0
# 或永久添加用户到dialout组
sudo usermod -a -G dialout $USER
# 需要重新登录
```

### 2. 无法连接激光雷达

检查网络配置：
```bash
# 检查本机IP
ip addr show

# ping激光雷达
ping 192.168.2.148
```

### 3. TF变换问题

```bash
# 查看TF树
ros2 run tf2_tools view_frames

# 检查特定变换
ros2 run tf2_ros tf2_echo map base_link
```

### 4. 话题检查

```bash
# 查看所有话题
ros2 topic list

# 检查话题数据
ros2 topic echo /scan
ros2 topic echo /odom
```

## 文件结构

```
nav_ws/src/
├── wheeltec_driver/          # 底盘驱动包
│   ├── include/
│   │   └── wheeltec_driver/
│   │       ├── serial_protocol.hpp  # 串口协议定义
│   │       ├── robot_params.hpp     # 机器人参数 ⚠️需配置
│   │       └── serial_port.hpp      # 串口类
│   ├── src/
│   │   ├── wheeltec_driver_node.cpp # 主驱动节点
│   │   └── serial_port.cpp          # 串口实现
│   ├── config/
│   │   └── driver_params.yaml       # 驱动参数 ⚠️需配置
│   └── launch/
│       └── driver.launch.py
│
├── wheeltec_description/     # 机器人描述包
│   ├── urdf/
│   │   └── robot.urdf.xacro         # 机器人模型 ⚠️需配置
│   └── launch/
│       └── description.launch.py
│
└── wheeltec_bringup/         # 启动配置包
    ├── config/
    │   ├── MID360_config.json           # 激光雷达配置
    │   ├── pointcloud_to_laserscan.yaml # 点云转激光扫描
    │   ├── slam_params.yaml             # SLAM建图参数
    │   ├── slam_localization_params.yaml # SLAM定位参数
    │   └── nav2_params.yaml             # 导航参数 ⚠️需配置robot_radius
    ├── launch/
    │   ├── bringup.launch.py            # 完整系统启动
    │   ├── lidar.launch.py              # 激光雷达启动
    │   ├── slam.launch.py               # SLAM建图启动
    │   └── navigation.launch.py         # 导航启动（slam_toolbox定位）
    ├── scripts/
    │   └── save_map.sh                  # 一键保存地图脚本
    ├── rviz/
    │   ├── slam.rviz                    # 建图RViz配置
    │   └── navigation.rviz              # 导航RViz配置
    └── maps/                            # 地图存储目录
```

## 许可证

Apache-2.0
