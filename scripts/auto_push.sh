#!/bin/bash

# Prva mapa - catkin_ws
echo "Switching to catkin_ws directory..."
cd /home/rokpi/Documents/ros2/catkin_ws || { echo "Failed to access catkin_ws directory"; exit 1; }

echo "Switching to branch main..."
git checkout main || exit 1

echo "Adding changes for catkin_ws..."
git add .

echo "Committing changes for catkin_ws..."
git commit -m "Auto-update for catkin_ws" || echo "No changes to commit for catkin_ws."

echo "Pushing changes for catkin_ws to branch main..."
git push origin main || echo "Push failed for catkin_ws (branch main)."

# Druga mapa - map_ws
echo "Switching to map_ws directory..."
cd /home/rokpi/Documents/ros2/map_ws || { echo "Failed to access map_ws directory"; exit 1; }

echo "Switching to branch main..."
git checkout main || exit 1

echo "Adding changes for map_ws..."
git add .

echo "Committing changes for map_ws..."
git commit -m "Auto-update for map_ws" || echo "No changes to commit for map_ws."

echo "Pushing changes for map_ws to branch main..."
git push origin main || echo "Push failed for map_ws (branch main)."

echo "Done!"

