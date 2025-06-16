#!/bin/bash
# 文件位置: /root/bridge_scripts/launch/start_ros1_converter.sh
# 在ros_caric_container_1容器中运行

echo "启动ROS1 PPCom转换器..."

# 进入ROS1工作空间
cd /root/ws_caric

# 设置完整的ROS环境
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# 验证环境
echo "验证ROS环境..."
if ! python3 -c "from rotors_comm.msg import PPComTopology; print('✓ ROS环境正确')" 2>/dev/null; then
    echo "✗ ROS环境设置失败"
    echo "请确保:"
    echo "1. 工作空间已编译: catkin build"
    echo "2. 环境已设置: source devel/setup.bash"
    exit 1
fi

# 检查脚本文件
if [ ! -f "scripts/ppcom_bridge/ros1_converter.py" ]; then
    echo "✗ 转换器脚本不存在: scripts/ppcom_bridge/ros1_converter.py"
    exit 1
fi

# 等待PPCom话题（如果CARIC仿真正在运行）
echo "等待PPCom话题..."
timeout=30
count=0
while [ $count -lt $timeout ]; do
    if rostopic list | grep -q ppcom; then
        echo "✓ 发现PPCom话题"
        break
    fi
    
    count=$((count + 1))
    sleep 1
    
    if [ $((count % 5)) -eq 0 ]; then
        echo "等待PPCom话题... ($count/$timeout)"
    fi
done

if [ $count -eq $timeout ]; then
    echo "⚠ 未发现PPCom话题，但仍然启动转换器"
    echo "请确保CARIC仿真已启动: roslaunch caric_baseline run.launch scenario:=mbs"
fi

# 启动转换器
echo "启动ROS1 PPCom转换器..."
python3 scripts/ppcom_bridge/ros1_converter.py