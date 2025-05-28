#!/bin/bash

CAMERA_TOPIC_NAME="${1:-/sensing/camera/image}"

CURRENT_DIR=$(
    cd "$(dirname "$0")" || exit
    pwd
)

# planning_bev.rviz
## 1. replace 'Displays:' block
"$CURRENT_DIR"/replace_displays_block.sh "$CURRENT_DIR"/../autoware.rviz "$CURRENT_DIR"/../planning_bev.rviz
## 2. enable Debug/Planning
sed -i 'N;/      Enabled: false\n      Name: Debug/ {s/false/true/g};P;D' "$CURRENT_DIR"/../planning_bev.rviz
sed -i 'N;/          Enabled: false\n          Name: Planning/ {s/false/true/g};P;D' "$CURRENT_DIR"/../planning_bev.rviz
## 3. add camera image
camera_image_block="        - Class: rviz_default_plugins/Image
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
camera_image_block_escaped=$(printf '%s\n' "$camera_image_block" | sed ':a;N;$!ba;s/\n/\\n/g')
sed -i "/        - Class: autoware_string_stamped_rviz_plugin\/StringStampedOverlayDisplay/i\\${camera_image_block_escaped}" "$CURRENT_DIR"/../planning_bev.rviz

# planning_tpv.rviz
## 1. replace 'Displays:' block
"$CURRENT_DIR"/replace_displays_block.sh "$CURRENT_DIR"/../autoware.rviz "$CURRENT_DIR"/../planning_tpv.rviz
## 2. enable Debug/Planning
sed -i 'N;/      Enabled: false\n      Name: Debug/ {s/false/true/g};P;D' "$CURRENT_DIR"/../planning_tpv.rviz
sed -i 'N;/          Enabled: false\n          Name: Planning/ {s/false/true/g};P;D' "$CURRENT_DIR"/../planning_tpv.rviz
## 3. add camera image
camera_image_block="        - Class: rviz_default_plugins/Image
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
camera_image_block_escaped=$(printf '%s\n' "$camera_image_block" | sed ':a;N;$!ba;s/\n/\\n/g')
sed -i "/        - Class: autoware_string_stamped_rviz_plugin\/StringStampedOverlayDisplay/i\\${camera_image_block_escaped}" "$CURRENT_DIR"/../planning_tpv.rviz
