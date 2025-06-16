#!/bin/bash
# 文件位置: /root/bridge_scripts/launch/stop_system.sh

echo "停止PPCom桥接系统..."

echo "停止ROS2客户端..."
docker exec ros_caric_drone bash -c "pkill -f ppcom_client.py" 2>/dev/null

echo "停止ROS2路由器..."
docker exec ros_caric_drone bash -c "pkill -f ppcom_router.py" 2>/dev/null

echo "停止ROS2转换器..."
docker exec ros_caric_drone bash -c "pkill -f ros2_converter.py" 2>/dev/null

echo "停止域桥接..."
docker exec ros_caric_bridge bash -c "pkill -f domain_bridge" 2>/dev/null

echo "停止ROS1-ROS2桥接..."
docker exec ros_caric_bridge bash -c "pkill -f parameter_bridge" 2>/dev/null

echo "停止ROS1转换器..."
docker exec ros_caric_container_1 bash -c "pkill -f ros1_converter.py" 2>/dev/null

echo "PPCom桥接系统已停止!"