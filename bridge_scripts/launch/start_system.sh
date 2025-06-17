#!/bin/bash
# File location: /root/bridge_scripts/launch/start_system.sh
# Running on the host machine, coordinating all containers

echo "Starting the full PPCom bridge system..."

# Check container status
if ! docker ps | grep -q ros_caric_container_1; then
    echo "✗ CARIC container is not running, please run: docker compose up"
    exit 1
fi

echo "=== Starting ROS1 converter ==="
docker exec -d ros_caric_container_1 bash -c "
    chmod +x /root/bridge_scripts/launch/start_ros1_converter.sh
    /root/bridge_scripts/launch/start_ros1_converter.sh
"

sleep 3

echo "=== Starting ROS1-ROS2 bridge ==="
docker exec -d ros_caric_bridge bash -c "
    source /opt/ros/humble/setup.bash
    rosparam load bridge.param
    ros2 run ros1_bridge parameter_bridge
" 2>/dev/null

sleep 3

echo "=== Starting domain bridge ==="
docker exec -d ros_caric_bridge bash -c "
    source /opt/ros/humble/setup.bash
    ros2 run domain_bridge domain_bridge domain_bridge.yaml
" 2>/dev/null

sleep 3

echo "=== Starting ROS2 system ==="
docker exec -d ros_caric_drone bash -c "
    chmod +x /root/bridge_scripts/launch/start_ros2_system.sh
    /root/bridge_scripts/launch/start_ros2_system.sh
"

sleep 5

echo "=== System status check ==="
echo "Checking ROS1 topics:"
docker exec ros_caric_container_1 bash -c "source /root/ws_caric/devel/setup.bash && rostopic list | grep ppcom"

echo "Checking ROS2 bridge topics:"
docker exec ros_caric_bridge bash -c "source /opt/ros/humble/setup.bash && ros2 topic list | grep ppcom"

echo "Checking ROS2 Domain 1 topics:"
docker exec ros_caric_drone bash -c "export ROS_DOMAIN_ID=1 && ros2 topic list | grep -E '(ppcom|ping_message)'"

echo "PPCom bridge system startup completed!"
echo ""
echo "Monitoring commands:"
echo "  # Monitoring raffles messages"
echo "  docker exec -it ros_caric_drone bash -c 'export ROS_DOMAIN_ID=1 && ros2 topic echo /ping_message/raffles'"
echo ""
echo "  # View all PPCom topics"
echo "  docker exec -it ros_caric_drone bash -c 'export ROS_DOMAIN_ID=1 && ros2 topic list | grep ping_message'"