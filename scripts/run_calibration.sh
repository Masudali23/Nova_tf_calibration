#!/bin/bash

# Nova TF Calibration - Calibration Runner Script
# Runs Multi_LiCa calibration with configured parameters

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

CONFIG_FILE="${REPO_DIR}/config/multi_lica_config.yaml"

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo "=========================================================================="
echo "Nova TF Calibration - Multi-LiDAR Calibration"
echo "=========================================================================="
echo ""

# Check if config exists
if [ ! -f "$CONFIG_FILE" ]; then
    echo -e "${RED}❌ Config file not found: $CONFIG_FILE${NC}"
    echo ""
    echo "Please create the configuration file first!"
    exit 1
fi

echo -e "${GREEN}✅ Found config: $CONFIG_FILE${NC}"
echo ""

# Check if ROS workspace is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}⚠️  ROS not sourced, attempting to source...${NC}"
    source /opt/ros/humble/setup.bash
fi

# Find and source workspace
if [ -d "$HOME/drobot_lidar/install" ]; then
    WORKSPACE="$HOME/drobot_lidar"
elif [ -d "$(pwd)/../install" ]; then
    WORKSPACE="$(cd "$(pwd)/.." && pwd)"
else
    echo -e "${RED}❌ Could not find ROS workspace${NC}"
    echo "Please source your workspace manually:"
    echo "  source /path/to/workspace/install/setup.bash"
    exit 1
fi

echo -e "${GREEN}✅ Sourcing workspace: $WORKSPACE${NC}"
source "$WORKSPACE/install/setup.bash"
echo ""

# Display configuration
echo "Configuration Summary:"
echo "----------------------------------------"
python3 << EOF
import yaml
with open('$CONFIG_FILE', 'r') as f:
    config = yaml.safe_load(f)

params = config['/**']['ros__parameters']
print(f"  PCD Directory: {params['pcd_directory']}")
print(f"  Output Directory: {params['output_dir']}")
print(f"  Frames per LiDAR: {params['frame_count']}")
print(f"  Target Frame: {params['target_frame_id']}")
print(f"  Visualization: {params['visualize']}")
print(f"  Fitness Threshold: {params['fitness_score_threshold']}")
EOF
echo "----------------------------------------"
echo ""

# Create output directory
OUTPUT_DIR=$(python3 -c "
import yaml, os
with open('$CONFIG_FILE', 'r') as f:
    config = yaml.safe_load(f)
print(os.path.expanduser(config['/**']['ros__parameters']['output_dir']))
")

mkdir -p "$OUTPUT_DIR"
echo -e "${GREEN}✅ Output directory ready: $OUTPUT_DIR${NC}"
echo ""

# Check if PCD files exist
PCD_DIR=$(python3 -c "
import yaml, os
with open('$CONFIG_FILE', 'r') as f:
    config = yaml.safe_load(f)
print(os.path.expanduser(config['/**']['ros__parameters']['pcd_directory']))
")

if [ ! -d "$PCD_DIR" ]; then
    echo -e "${RED}❌ PCD directory not found: $PCD_DIR${NC}"
    echo "Please record calibration data first!"
    exit 1
fi

PCD_COUNT=$(ls "$PCD_DIR"/*.pcd 2>/dev/null | wc -l)
if [ "$PCD_COUNT" -eq 0 ]; then
    echo -e "${RED}❌ No PCD files found in: $PCD_DIR${NC}"
    echo "Please record calibration data first!"
    exit 1
fi

echo -e "${GREEN}✅ Found $PCD_COUNT PCD files${NC}"
echo ""

# Check TEASER++ installation
if ! python3 -c "import teaserpp_python" 2>/dev/null; then
    echo -e "${RED}❌ TEASER++ Python bindings not found${NC}"
    echo "Please run: ./scripts/install_dependencies.sh"
    exit 1
fi

echo -e "${GREEN}✅ TEASER++ available${NC}"
echo ""

# Run calibration
echo "=========================================================================="
echo "Starting Calibration..."
echo "=========================================================================="
echo ""
echo -e "${BLUE}This may take 1-5 minutes depending on your hardware${NC}"
echo ""

ros2 launch multi_lidar_calibrator calibration.launch.py \
    parameter_file:="$CONFIG_FILE"

EXITCODE=$?

echo ""
echo "=========================================================================="

if [ $EXITCODE -eq 0 ]; then
    echo -e "${GREEN}✅ CALIBRATION COMPLETE!${NC}"
    echo "=========================================================================="
    echo ""
    echo "Results saved to: $OUTPUT_DIR"
    echo ""
    echo "Output files:"
    ls -lh "$OUTPUT_DIR"
    echo ""
    echo "View results:"
    echo "  cat $OUTPUT_DIR/results.txt"
    echo ""
    echo "Visualize:"
    echo "  python3 scripts/visualize_results.py"
    echo ""
else
    echo -e "${RED}❌ CALIBRATION FAILED${NC}"
    echo "=========================================================================="
    echo ""
    echo "Check the error messages above."
    echo "For troubleshooting, see: docs/troubleshooting.md"
    echo ""
    exit 1
fi