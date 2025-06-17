#!/bin/bash
# File location: /root/bridge_scripts/launch/start_ros2_system.sh
# Running in ros_caric_drone container

echo "Starting ROS2 PPCom system..."

# Set up ROS2 environment
export ROS_DOMAIN_ID=1
source /opt/ros/humble/setup.bash

# Check script directory
if [ ! -d "/root/bridge_scripts/ros2_side" ]; then
    echo "✗ ROS2 script directory does not exist: /root/bridge_scripts/ros2_side"
    exit 1
fi

cd /root/bridge_scripts

# Waiting for bridged JSON topic
echo "Waiting for the bridged PPCom topic..."
timeout=60
count=0
while [ $count -lt $timeout ]; do
    if ros2 topic list | grep -q ppcom_topology_json; then
        echo "✓ Bridged PPCom topic detected"
        break
    fi
    
    count=$((count + 1))
    sleep 1
    
    if [ $((count % 10)) -eq 0 ]; then
        echo "Waiting for bridged topic... ($count/$timeout)"
    fi
done

if [ $count -eq $timeout ]; then
    echo "⚠ Bridged topic not detected, but still starting system"
    echo "Please ensure:"
    echo "1. ROS1 converter is started"
    echo "2. ROS1-ROS2 bridge is started"
    echo "3. Domain bridge is started"
fi

# Start ROS2 converter
echo "Starting ROS2 converter..."
python3 ros2_side/ros2_converter.py &
ROS2_CONVERTER_PID=$!

sleep 2

# Start PPCom router
echo "Starting PPCom router..."
python3 ros2_side/ppcom_router.py &
PPCOM_ROUTER_PID=$!

sleep 3

# Start PPCom clients
echo "Starting PPCom clients..."
python3 ros2_side/ppcom_client.py raffles &
python3 ros2_side/ppcom_client.py jurong &
python3 ros2_side/ppcom_client.py sentosa &
python3 ros2_side/ppcom_client.py changi &
python3 ros2_side/ppcom_client.py nanyang &

echo "ROS2 PPCom system startup completed!"
echo "Process IDs: Converter=$ROS2_CONVERTER_PID, Router=$PPCOM_ROUTER_PID"

# Wait for interrupt signal
wait