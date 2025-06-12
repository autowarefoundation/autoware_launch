#!/usr/bin/env bash

CAMERA_TOPIC_NAME="${1:-/sensing/camera/image}"

SCRIPT_DIR=$(
    cd "$(dirname "$0")" || exit
    pwd
)

# get autoware_launch directory
AUTOWARE_LAUNCH_DIR="$SCRIPT_DIR"/../../

# define rviz files
AUTOWARE_RVIZ="$AUTOWARE_LAUNCH_DIR"/rviz/autoware.rviz
PLANNING_BEV_RVIZ="$AUTOWARE_LAUNCH_DIR"/rviz/planning_bev.rviz
PLANNING_TPV_RVIZ="$AUTOWARE_LAUNCH_DIR"/rviz/planning_tpv.rviz

# planning_bev.rviz
## 1. replace 'Displays:' block
"$SCRIPT_DIR"/replace_displays_block.sh "$AUTOWARE_RVIZ" "$PLANNING_BEV_RVIZ"
## 2. enable Debug/Planning
sed -i 'N;/      Enabled: false\n      Name: Debug/ {s/false/true/g};P;D' "$PLANNING_BEV_RVIZ"
sed -i 'N;/          Enabled: false\n          Name: Planning/ {s/false/true/g};P;D' "$PLANNING_BEV_RVIZ"
## 3. add camera image
CAMERA_IMAGE_BLOCK="    - Class: rviz_default_plugins/Image
      Enabled: true
      Max Value: 1
      Median window: 5
      Min Value: 0
      Name: Image
      Normalize Range: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: ${CAMERA_TOPIC_NAME}
      Value: true"
CAMERA_IMAGE_BLOCK_ESCAPED=$(printf '%s\n' "$CAMERA_IMAGE_BLOCK" | sed ':a;N;$!ba;s/\n/\\n/g')
sed -i "/^  Enabled: true$/{N;/^  Enabled: true\n  Global Options:\$/i\\$CAMERA_IMAGE_BLOCK_ESCAPED
}" "$PLANNING_BEV_RVIZ"
## 4. enable error_diag_graph visualization
sed -i 'N;/        - Class: autoware_string_stamped_rviz_plugin\/StringStampedOverlayDisplay\n          Enabled: false/ {s/false/true/g};P;D' "$PLANNING_BEV_RVIZ"

# planning_tpv.rviz
## 1. replace 'Displays:' block
"$SCRIPT_DIR"/replace_displays_block.sh "$AUTOWARE_RVIZ" "$PLANNING_TPV_RVIZ"
## 2. enable Debug/Planning
sed -i 'N;/      Enabled: false\n      Name: Debug/ {s/false/true/g};P;D' "$PLANNING_TPV_RVIZ"
sed -i 'N;/          Enabled: false\n          Name: Planning/ {s/false/true/g};P;D' "$PLANNING_TPV_RVIZ"
## 3. add camera image
sed -i "/^  Enabled: true$/{N;/^  Enabled: true\n  Global Options:\$/i\\$CAMERA_IMAGE_BLOCK_ESCAPED
}" "$PLANNING_TPV_RVIZ"
## 4. enable error_diag_graph visualization
sed -i 'N;/        - Class: autoware_string_stamped_rviz_plugin\/StringStampedOverlayDisplay\n          Enabled: false/ {s/false/true/g};P;D' "$PLANNING_TPV_RVIZ"
