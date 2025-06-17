#!/bin/bash
# File location: /root/bridge_scripts/launch/stop_system.sh

echo "Stopping PPCom bridge system..."

echo "Stopping ROS2 clients..."
docker exec ros_caric_drone bash -c "pkill -f ppcom_client.py" 2>/dev/null

echo "Stopping ROS2 router..."
docker exec ros_caric_drone bash -c "pkill -f ppcom_router.py" 2>/dev/null

echo "Stopping ROS2 converter..."
docker exec ros_caric_drone bash -c "pkill -f ros2_converter.py" 2>/dev/null

echo "Stopping domain bridge..."
docker exec ros_caric_bridge bash -c "pkill -f domain_bridge" 2>/dev/null

echo "Stopping ROS1-ROS2 bridge..."
docker exec ros_caric_bridge bash -c "pkill -f parameter_bridge" 2>/dev/null

echo "Stopping ROS1 converter..."
docker exec ros_caric_container_1 bash -c "pkill -f ros1_converter.py" 2>/dev/null

echo "PPCom bridge system has been stopped!"