#!/bin/bash
# 文件位置: /root/bridge_scripts/launch/start_system.sh
# 在主机上运行，协调所有容器

echo "启动完整PPCom桥接系统..."

# 检查容器状态
if ! docker ps | grep -q ros_caric_container_1; then
    echo "✗ CARIC容器未运行，请先执行: docker compose up"
    exit 1
fi

echo "=== 启动ROS1转换器 ==="
docker exec -d ros_caric_container_1 bash -c "
    chmod +x /root/bridge_scripts/launch/start_ros1_converter.sh
    /root/bridge_scripts/launch/start_ros1_converter.sh
"

sleep 3

echo "=== 启动ROS1-ROS2桥接 ==="
docker exec -d ros_caric_bridge bash -c "
    source /opt/ros/humble/setup.bash
    rosparam load bridge.param
    ros2 run ros1_bridge parameter_bridge
" 2>/dev/null

sleep 3

echo "=== 启动域桥接 ==="
docker exec -d ros_caric_bridge bash -c "
    source /opt/ros/humble/setup.bash
    ros2 run domain_bridge domain_bridge domain_bridge.yaml
" 2>/dev/null

sleep 3

echo "=== 启动ROS2系统 ==="
docker exec -d ros_caric_drone bash -c "
    chmod +x /root/bridge_scripts/launch/start_ros2_system.sh
    /root/bridge_scripts/launch/start_ros2_system.sh
"

sleep 5

echo "=== 系统状态检查 ==="
echo "检查ROS1话题:"
docker exec ros_caric_container_1 bash -c "source /root/ws_caric/devel/setup.bash && rostopic list | grep ppcom"

echo "检查ROS2桥接话题:"
docker exec ros_caric_bridge bash -c "source /opt/ros/humble/setup.bash && ros2 topic list | grep ppcom"

echo "检查ROS2 Domain 1话题:"
docker exec ros_caric_drone bash -c "export ROS_DOMAIN_ID=1 && ros2 topic list | grep -E '(ppcom|ping_message)'"

echo "PPCom桥接系统启动完成!"
echo ""
echo "监控命令:"
echo "  # 监控raffles消息"
echo "  docker exec -it ros_caric_drone bash -c 'export ROS_DOMAIN_ID=1 && ros2 topic echo /ping_message/raffles'"
echo ""
echo "  # 查看所有PPCom话题"
echo "  docker exec -it ros_caric_drone bash -c 'export ROS_DOMAIN_ID=1 && ros2 topic list | grep ping_message'"