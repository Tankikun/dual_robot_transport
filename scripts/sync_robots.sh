#!/bin/bash
# USAGE: ./scripts/sync_robots.sh
# Description: Pushes local code changes to both robots instantly using rsync.

# Define Robot IPs (Make sure these are Static!)
ROBOT_A="192.168.1.101"
ROBOT_B="192.168.1.102"
USER="ubuntu"
REMOTE_PATH="~/turtlebot3_ws/src/dual_robot_transport"

echo "=========================================="
echo "  SYNCING CODE TO FLEET"
echo "=========================================="

# 1. Sync to Robot A
echo "--> Sending to Robot A ($ROBOT_A)..."
# Exclude git files and pycache to save bandwidth
rsync -avz --exclude '.git' --exclude '__pycache__' --exclude 'build' --exclude 'install' \
    ../dual_robot_transport/ \
    $USER@$ROBOT_A:$REMOTE_PATH

# 2. Sync to Robot B
echo "--> Sending to Robot B ($ROBOT_B)..."
rsync -avz --exclude '.git' --exclude '__pycache__' \
    ../dual_robot_transport/ \
    $USER@$ROBOT_B:$REMOTE_PATH

echo "=========================================="
echo "  SYNC COMPLETE"
echo "=========================================="
