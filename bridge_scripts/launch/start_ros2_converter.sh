#!/bin/bash
# 文件位置: /root/bridge_scripts/launch/start_ros2_system.sh
# 在ros_caric_drone容器中运行

echo "启动ROS2 PPCom系统..."

# 设置ROS2环境
export ROS_DOMAIN_ID=1
source /opt/ros/humble/setup.bash

# 检查脚本目录
if [ ! -d "/root/bridge_scripts/ros2_side" ]; then
    echo "✗ ROS2脚本目录不存在: /root/bridge_scripts/ros2_side"
    exit 1
fi

cd /root/bridge_scripts

# 等待桥接的JSON话题
echo "等待桥接的PPCom话题..."
timeout=60
count=0
while [ $count -lt $timeout ]; do
    if ros2 topic list | grep -q ppcom_topology_json; then
        echo "✓ 发现桥接的PPCom话题"
        break
    fi
    
    count=$((count + 1))
    sleep 1
    
    if [ $((count % 10)) -eq 0 ]; then
        echo "等待桥接话题... ($count/$timeout)"
    fi
done

if [ $count -eq $timeout ]; then
    echo "⚠ 未发现桥接话题，但仍然启动系统"
    echo "请确保:"
    echo "1. ROS1转换器已启动"
    echo "2. ROS1-ROS2桥接已启动"
    echo "3. 域桥接已启动"
fi

# 启动ROS2转换器
echo "启动ROS2转换器..."
python3 ros2_side/ros2_converter.py &
ROS2_CONVERTER_PID=$!

sleep 2

# 启动PPCom路由器
echo "启动PPCom路由器..."
python3 ros2_side/ppcom_router.py &
PPCOM_ROUTER_PID=$!

sleep 3

# 启动PPCom客户端
echo "启动PPCom客户端..."
python3 ros2_side/ppcom_client.py raffles &
python3 ros2_side/ppcom_client.py jurong &
python3 ros2_side/ppcom_client.py sentosa &
python3 ros2_side/ppcom_client.py changi &
python3 ros2_side/ppcom_client.py nanyang &

echo "ROS2 PPCom系统启动完成!"
echo "进程ID: 转换器=$ROS2_CONVERTER_PID, 路由器=$PPCOM_ROUTER_PID"

# 等待中断信号
wait