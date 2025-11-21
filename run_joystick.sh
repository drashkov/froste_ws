#!/bin/bash
# Usage: ./run_joystick.sh [invert]
# Pass 'invert' as argument to invert controls (for motor controller bug)
INVERT=${1:-false}
if [ "$INVERT" = "invert" ]; then
    INVERT="true"
fi
ros2 launch ugv_tools teleop_twist_joy.launch.py invert_controls:=$INVERT
