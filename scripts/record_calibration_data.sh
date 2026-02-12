#!/bin/bash

# Nova TF Calibration - PCD Recording Wrapper Script
# Convenient wrapper for recording calibration data from multiple LiDARs

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

CONFIG_FILE="${REPO_DIR}/config/pcd_recording_config.yaml"

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "=========================================================================="
echo "Nova TF Calibration - Multi-LiDAR PCD Recording"
echo "=========================================================================="
echo ""

# Check if config exists
if [ ! -f "$CONFIG_FILE" ]; then
    echo -e "${RED}❌ Config file not found: $CONFIG_FILE${NC}"
    echo ""
    echo "Please create the configuration file first!"
    echo "Example:"
    echo "  cp config/pcd_recording_config.yaml.example config/pcd_recording_config.yaml"
    echo "  nano config/pcd_recording_config.yaml"
    exit 1
fi

echo -e "${GREEN}✅ Found config: $CONFIG_FILE${NC}"
echo ""

# Display configuration
echo "Configuration:"
echo "----------------------------------------"
python3 -c "
import yaml
with open('$CONFIG_FILE', 'r') as f:
    config = yaml.safe_load(f)
print(f\"  Output directory: {config['recording']['output_directory']}\")
print(f\"  Frames per LiDAR: {config['recording']['frames_per_lidar']}\")
print(f\"  LiDARs:\")
for lidar in config['recording']['lidars']:
    if lidar.get('enabled', True):
        print(f\"    - {lidar['name']}: {lidar['topic']}\")
"
echo "----------------------------------------"
echo ""

# Important reminders
echo -e "${YELLOW}⚠️  IMPORTANT REMINDERS:${NC}"
echo "  1. Position robot in a FEATURE-RICH environment"
echo "  2. Keep robot STATIONARY during recording"
echo "  3. Ensure ALL LiDARs are publishing data"
echo "  4. Check that topics match your robot configuration"
echo ""

# Check if ROS topics are available
echo "Checking ROS topics..."
source /opt/ros/humble/setup.bash

# Extract topics from config
TOPICS=$(python3 -c "
import yaml
with open('$CONFIG_FILE', 'r') as f:
    config = yaml.safe_load(f)
for lidar in config['recording']['lidars']:
    if lidar.get('enabled', True):
        print(lidar['topic'])
")

ALL_AVAILABLE=true
for topic in $TOPICS; do
    if ros2 topic list | grep -q "^${topic}$"; then
        echo -e "  ${GREEN}✅${NC} $topic"
    else
        echo -e "  ${RED}❌${NC} $topic (NOT FOUND)"
        ALL_AVAILABLE=false
    fi
done

if [ "$ALL_AVAILABLE" = false ]; then
    echo ""
    echo -e "${RED}❌ Some topics are not available!${NC}"
    echo "Please start your robot's LiDAR drivers first."
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo ""
echo "=========================================================================="
echo "Press ENTER to start recording, or Ctrl+C to cancel..."
echo "=========================================================================="
read

# Run the recording script
python3 "${SCRIPT_DIR}/record_pcds_multi.py" --config "$CONFIG_FILE"

# Check results
OUTPUT_DIR=$(python3 -c "
import yaml, os
with open('$CONFIG_FILE', 'r') as f:
    config = yaml.safe_load(f)
print(os.path.expanduser(config['recording']['output_directory']))
")

echo ""
echo "=========================================================================="
echo "Recording Complete!"
echo "=========================================================================="
echo ""
echo "Recorded files:"
ls -lh "$OUTPUT_DIR"/*.pcd 2>/dev/null || echo "No PCD files found"
echo ""
echo "Next step: Run calibration with ./scripts/run_calibration.sh"
echo ""