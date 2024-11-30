#!/bin/bash

# Prva mapa - catkin_ws
echo "Switching to catkin_ws directory..."
cd /home/rokpi/Documents/ros2/catkin_ws || { echo "Failed to access catkin_ws directory"; exit 1; }

echo "Switching to branch main..."
git checkout main || exit 1

echo "Pulling changes for catkin_ws (branch main)..."
git pull origin main || echo "Pull failed for catkin_ws (branch main)."

# Druga mapa - map_ws
echo "Switching to map_ws directory..."
cd /home/rokpi/Documents/ros2/map_ws || { echo "Failed to access map_ws directory"; exit 1; }

echo "Switching to branch main..."
git checkout main || exit 1

echo "Pulling changes for map_ws (branch main)..."
git pull origin main || echo "Pull failed for map_ws (branch main)."

echo "Done!"

