#!/bin/bash
# 保存地图脚本 - 同时保存占据栅格地图和 slam_toolbox 序列化地图
# 用法: ./save_map.sh <地图名称>
# 示例: ./save_map.sh my_home

if [ -z "$1" ]; then
    echo "用法: $0 <地图名称>"
    echo "示例: $0 my_home"
    exit 1
fi

MAP_DIR="$HOME/nav_ws/maps"
MAP_NAME="$1"

mkdir -p "$MAP_DIR"

echo "=== 保存地图: $MAP_NAME ==="

# 1. 保存占据栅格地图 (pgm + yaml)
echo "[1/2] 保存占据栅格地图..."
ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/$MAP_NAME"

# 2. 保存 slam_toolbox 序列化地图 (posegraph + data)
echo "[2/2] 保存 slam_toolbox 序列化地图..."
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$MAP_DIR/${MAP_NAME}_serialized'}"

echo ""
echo "=== 保存完成! ==="
echo "文件列表:"
ls -la "$MAP_DIR/${MAP_NAME}"* 2>/dev/null
echo ""
echo "启动导航命令:"
echo "ros2 launch wheeltec_bringup bringup.launch.py mode:=nav map:=~/nav_ws/maps/${MAP_NAME}.yaml serialized_map:=~/nav_ws/maps/${MAP_NAME}_serialized"
