#!/bin/bash
#
# Gazebo Cleanup Script
# Kills all Gazebo and spawn_entity processes to ensure clean simulation state
#
# Usage: ./cleanup_gazebo.sh
#        Or call before launch: ./cleanup_gazebo.sh && ros2 launch ...
#

set -e



# Kill gzserver processes
if pgrep -f "gzserver" > /dev/null 2>&1; then
    echo "[1/4] Killing gzserver processes..."
    pkill -9 -f "gzserver"
else
    echo "[1/4] No gzserver processes found ✓"
fi

# Kill gzclient processes
if pgrep -f "gzclient" > /dev/null 2>&1; then
    echo "[2/4] Killing gzclient processes..."
    pkill -9 -f "gzclient"
else
    echo "[2/4] No gzclient processes found ✓"
fi

# Kill spawn_entity processes
if pgrep -f "spawn_entity" > /dev/null 2>&1; then
    echo "[3/4] Killing spawn_entity processes..."
    pkill -9 -f "spawn_entity"
else
    echo "[3/4] No spawn_entity processes found ✓"
fi

# Kill other ROS 2 processes (rsp, move_group, spawners)
echo "[*] Killing remaining ROS 2 simulation processes..."
pkill -9 -f "robot_state_publisher" || true
pkill -9 -f "move_group" || true
pkill -9 -f "spawner" || true
pkill -9 -f "pick_place" || true
pkill -9 -f "simple_pick_place" || true

# Wait for processes to fully terminate
sleep 2

# Verify all cleaned up
echo "[4/4] Verifying cleanup..."
if pgrep -f "gzserver|gzclient|spawn_entity" > /dev/null 2>&1; then
    echo "    ⚠️  WARNING: Some Gazebo processes still running:"
    pgrep -f "gzserver|gzclient|spawn_entity" || true
    echo "    Try running: pkill -9 -f 'gzserver|gzclient|spawn_entity'"
else
    echo "    ✓ All Gazebo processes cleaned up successfully"
fi


