#!/bin/bash

# FrostE Map Saving Script
# Saves the current RTAB-Map database and generates a 2D occupancy grid map
# Usage: ./save_froste_map.sh [map_name]

# Default map name with timestamp
if [ -z "$1" ]; then
    MAP_NAME="froste_map_$(date +%Y%m%d_%H%M%S)"
else
    MAP_NAME="$1"
fi

MAPS_DIR="$HOME/ugv_ws/maps"
MAP_PATH="$MAPS_DIR/$MAP_NAME"

echo "========================================="
echo " FrostE Map Saving Script"
echo "========================================="
echo "Map name: $MAP_NAME"
echo "Map directory: $MAP_PATH"
echo ""

# Create maps directory if it doesn't exist
mkdir -p "$MAPS_DIR"

# Save the 2D occupancy grid map using nav2_map_server
echo "[1/2] Saving 2D occupancy grid map..."
ros2 run nav2_map_server map_saver_cli -f "$MAP_PATH" --ros-args -p save_map_timeout:=10000.0

if [ $? -eq 0 ]; then
    echo "✓ 2D map saved to: $MAP_PATH.pgm and $MAP_PATH.yaml"
else
    echo "✗ Failed to save 2D map"
    exit 1
fi

# Copy the RTAB-Map database
echo "[2/2] Copying RTAB-Map database..."
RTABMAP_DB="$HOME/.ros/rtabmap.db"
if [ -f "$RTABMAP_DB" ]; then
    cp "$RTABMAP_DB" "$MAP_PATH.db"
    echo "✓ RTAB-Map database saved to: $MAP_PATH.db"
else
    echo "✗ RTAB-Map database not found at: $RTABMAP_DB"
    echo "  (This is normal if you just started mapping)"
fi

echo ""
echo "========================================="
echo " Map Saved Successfully!"
echo "========================================="
echo "Files created:"
echo "  - $MAP_PATH.yaml (map metadata)"
echo "  - $MAP_PATH.pgm (2D occupancy grid image)"
if [ -f "$MAP_PATH.db" ]; then
    echo "  - $MAP_PATH.db (RTAB-Map database)"
fi
echo ""
echo "To use this map for navigation:"
echo "  ros2 launch ugv_nav froste_nav.launch.py map:=$MAP_PATH.yaml"
echo ""
