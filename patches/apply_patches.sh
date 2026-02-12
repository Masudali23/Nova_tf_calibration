#!/bin/bash

# Nova TF Calibration - Patch Application Script
# Applies necessary fixes to Multi_LiCa for our use case

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_status() {
    echo -e "${GREEN}[✓]${NC} $1"
}

print_error() {
    echo -e "${RED}[✗]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[!]${NC} $1"
}

echo "========================================================================"
echo "Applying Multi_LiCa Patches"
echo "========================================================================"
echo ""

# Find Multi_LiCa directory
MULTI_LICA_DIR=""

if [ -d "$REPO_DIR/../Multi_LiCa" ]; then
    MULTI_LICA_DIR="$REPO_DIR/../Multi_LiCa"
elif [ -d "$HOME/drobot_lidar/src/Multi_LiCa" ]; then
    MULTI_LICA_DIR="$HOME/drobot_lidar/src/Multi_LiCa"
else
    print_error "Could not find Multi_LiCa directory"
    echo "Please ensure Multi_LiCa is cloned in your workspace"
    exit 1
fi

print_status "Found Multi_LiCa at: $MULTI_LICA_DIR"

# Backup original file
CALIBRATOR_FILE="$MULTI_LICA_DIR/multi_lidar_calibrator/multi_lidar_calibrator.py"

if [ ! -f "$CALIBRATOR_FILE" ]; then
    print_error "Could not find multi_lidar_calibrator.py"
    exit 1
fi

# Create backup if not exists
if [ ! -f "${CALIBRATOR_FILE}.original" ]; then
    cp "$CALIBRATOR_FILE" "${CALIBRATOR_FILE}.original"
    print_status "Created backup: ${CALIBRATOR_FILE}.original"
else
    print_warning "Backup already exists, skipping"
fi

# Apply patch
PATCH_FILE="$SCRIPT_DIR/multi_lidar_calibrator_fix.patch"

if [ ! -f "$PATCH_FILE" ]; then
    print_error "Patch file not found: $PATCH_FILE"
    exit 1
fi

echo ""
echo "Applying patch..."

cd "$MULTI_LICA_DIR"

if patch -p1 --dry-run < "$PATCH_FILE" > /dev/null 2>&1; then
    patch -p1 < "$PATCH_FILE"
    print_status "Patch applied successfully"
else
    print_warning "Patch may already be applied or file was modified"
    echo ""
    echo "If calibration fails, try restoring original:"
    echo "  cp ${CALIBRATOR_FILE}.original ${CALIBRATOR_FILE}"
    echo "  Then re-run this script"
fi

echo ""
echo "========================================================================"
echo "✅ Patching complete!"
echo "========================================================================"
echo ""
echo "Changes applied:"
echo "  1. Fixed LiDAR name extraction (front_000.pcd → front)"
echo "  2. Fixed absolute path handling for pcd_directory and output_dir"
echo "  3. Added debug logging"
echo ""