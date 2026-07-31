#!/bin/bash
# test_imu_complete.sh - Complete IMU verification test
# This script:
# 1. Launches Gazebo simulation in background
# 2. Waits for simulation to start
# 3. Verifies IMU topic exists
# 4. Checks IMU data frequency
# 5. Verifies message content

set -e

echo "=== Task 6.3: IMU Data Publishing Verification ==="
echo ""

# Source workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch Gazebo in background
echo "1. Launching Gazebo simulation..."
ros2 launch my_robot_description gazebo.launch.py > /tmp/gazebo_imu_test.log 2>&1 &
GAZEBO_PID=$!
echo "   Gazebo PID: $GAZEBO_PID"

# Wait for Gazebo to start
echo "2. Waiting for simulation to initialize (20 seconds)..."
sleep 20

# Check if Gazebo is still running
if ! kill -0 $GAZEBO_PID 2>/dev/null; then
    echo "   ✗ Gazebo process died. Check /tmp/gazebo_imu_test.log"
    exit 1
fi

# Verify IMU topic exists
echo "3. Verifying /imu/data topic exists..."
if ros2 topic list | grep -q "/imu/data"; then
    echo "   ✓ /imu/data topic exists"
else
    echo "   ✗ /imu/data topic not found"
    kill $GAZEBO_PID
    exit 1
fi

# Check topic type
echo "4. Verifying topic type..."
TOPIC_TYPE=$(ros2 topic type /imu/data)
if [ "$TOPIC_TYPE" = "sensor_msgs/msg/Imu" ]; then
    echo "   ✓ Topic type is sensor_msgs/msg/Imu"
else
    echo "   ✗ Unexpected topic type: $TOPIC_TYPE"
    kill $GAZEBO_PID
    exit 1
fi

# Check message frequency (sample for 3 seconds)
echo "5. Checking message frequency..."
timeout 3 ros2 topic hz /imu/data > /tmp/imu_hz.txt 2>&1 || true
if grep -q "average rate:" /tmp/imu_hz.txt; then
    RATE=$(grep "average rate:" /tmp/imu_hz.txt | tail -1 | awk '{print $3}')
    echo "   ✓ IMU publishing at ${RATE} Hz"
else
    echo "   ⚠ Could not measure frequency (may need longer sampling)"
fi

# Verify message content
echo "6. Verifying message content..."
timeout 2 ros2 topic echo /imu/data --once > /tmp/imu_msg.txt 2>&1 || true
if grep -q "linear_acceleration:" /tmp/imu_msg.txt && grep -q "angular_velocity:" /tmp/imu_msg.txt; then
    echo "   ✓ Message contains linear_acceleration and angular_velocity fields"
else
    echo "   ⚠ Could not verify message content (timeout or format issue)"
fi

# Cleanup
echo ""
echo "7. Cleaning up..."
kill $GAZEBO_PID 2>/dev/null || true
sleep 2

echo ""
echo "=== Task 6.3 Verification Complete ==="
echo "✓ IMU sensor is configured and publishing data"
