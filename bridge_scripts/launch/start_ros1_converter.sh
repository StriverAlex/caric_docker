#!/bin/bash
# File location: /root/bridge_scripts/launch/start_ros1_converter.sh
# Running in ros_caric_container_1 container

echo "Starting ROS1 PPCom converter..."

# Enter ROS1 workspace
cd /root/ws_caric

# Set up complete ROS environment
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# Verify environment
echo "Verifying ROS environment..."
if ! python3 -c "from rotors_comm.msg import PPComTopology; print('✓ ROS environment correct')" 2>/dev/null; then
    echo "✗ ROS environment setup failed"
    echo "Please ensure:"
    echo "1. Workspace is compiled: catkin build"
    echo "2. Environment is set: source devel/setup.bash"
    exit 1
fi

# Check script file
if [ ! -f "scripts/ppcom_bridge/ros1_converter.py" ]; then
    echo "✗ Converter script does not exist: scripts/ppcom_bridge/ros1_converter.py"
    exit 1
fi

# Wait for PPCom topic (if CARIC simulation is running)
echo "Waiting for PPCom topic..."
timeout=30
count=0
while [ $count -lt $timeout ]; do
    if rostopic list | grep -q ppcom; then
        echo "✓ PPCom topic detected"
        break
    fi
    
    count=$((count + 1))
    sleep 1
    
    if [ $((count % 5)) -eq 0 ]; then
        echo "Waiting for PPCom topic... ($count/$timeout)"
    fi
done

if [ $count -eq $timeout ]; then
    echo "⚠ PPCom topic not detected, but still starting converter"
    echo "Please ensure CARIC simulation is started: roslaunch caric_baseline run.launch scenario:=mbs"
fi

# Start converter
echo "Starting ROS1 PPCom converter..."
python3 scripts/ppcom_bridge/ros1_converter.py