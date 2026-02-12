#!/bin/bash

# Nova TF Calibration - Dependency Installation Script
# This script installs all required dependencies for multi-LiDAR calibration

set -e  # Exit on error

echo "========================================================================"
echo "Nova TF Calibration - Dependency Installation"
echo "========================================================================"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[✓]${NC} $1"
}

print_error() {
    echo -e "${RED}[✗]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[!]${NC} $1"
}

# Check if running on Ubuntu
if [ ! -f /etc/lsb-release ]; then
    print_error "This script is designed for Ubuntu. Exiting."
    exit 1
fi

# Check Ubuntu version
source /etc/lsb-release
if [[ "$DISTRIB_RELEASE" != "22.04" ]] && [[ "$DISTRIB_RELEASE" != "20.04" ]]; then
    print_warning "This script is tested on Ubuntu 20.04 and 22.04. You have $DISTRIB_RELEASE"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# ========================================================================
# STEP 1: Update System
# ========================================================================
echo ""
echo "========================================================================="
echo "STEP 1: Updating System Packages"
echo "========================================================================="

sudo apt update
print_status "System package list updated"

# ========================================================================
# STEP 2: Install Build Tools
# ========================================================================
echo ""
echo "========================================================================="
echo "STEP 2: Installing Build Tools"
echo "========================================================================="

sudo apt install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    python3-pip \
    python3-dev

print_status "Build tools installed"

# ========================================================================
# STEP 3: Install System Libraries
# ========================================================================
echo ""
echo "========================================================================="
echo "STEP 3: Installing System Libraries"
echo "========================================================================="

sudo apt install -y \
    libeigen3-dev \
    libboost-all-dev \
    libgtest-dev \
    libgflags-dev \
    libgoogle-glog-dev

print_status "System libraries installed"

# ========================================================================
# STEP 4: Install ROS 2 Dependencies
# ========================================================================
echo ""
echo "========================================================================="
echo "STEP 4: Installing ROS 2 Dependencies"
echo "========================================================================="

# Check if ROS 2 is installed
if [ ! -d "/opt/ros/humble" ]; then
    print_error "ROS 2 Humble not found. Please install ROS 2 Humble first."
    echo "Visit: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

source /opt/ros/humble/setup.bash
print_status "ROS 2 Humble detected"

sudo apt install -y \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-tf2-ros \
    ros-humble-pcl-ros \
    python3-colcon-common-extensions

print_status "ROS 2 dependencies installed"

# ========================================================================
# STEP 5: Install Python Packages
# ========================================================================
echo ""
echo "========================================================================="
echo "STEP 5: Installing Python Packages"
echo "========================================================================="

pip3 install --user \
    numpy \
    scipy \
    open3d \
    pyyaml \
    matplotlib

# Install ros2-numpy
pip3 install --user ros2-numpy

print_status "Python packages installed"

# ========================================================================
# STEP 6: Setup Workspace
# ========================================================================
echo ""
echo "========================================================================="
echo "STEP 6: Setting Up Workspace"
echo "========================================================================="

# Determine workspace directory
if [ -z "$ROS_WORKSPACE" ]; then
    ROS_WORKSPACE="$HOME/drobot_lidar"
    print_warning "ROS_WORKSPACE not set, using default: $ROS_WORKSPACE"
fi

mkdir -p "$ROS_WORKSPACE/src"
cd "$ROS_WORKSPACE/src"

print_status "Workspace created at $ROS_WORKSPACE"

# ========================================================================
# STEP 7: Install TEASER++
# ========================================================================
echo ""
echo "========================================================================="
echo "STEP 7: Installing TEASER++"
echo "========================================================================="

if [ -d "TEASER-plusplus" ]; then
    print_warning "TEASER-plusplus directory exists, skipping clone"
else
    git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
    print_status "TEASER++ cloned"
fi

cd TEASER-plusplus

# Apply vector fix if needed
if grep -q "^vector<" teaser/src/graph.cc 2>/dev/null; then
    print_warning "Applying std::vector fix to graph.cc"
    sed -i 's/\bvector</std::vector</g' teaser/src/graph.cc
    sed -i 's/std::std::vector/std::vector/g' teaser/src/graph.cc
    
    sed -i 's/\bvector</std::vector</g' teaser/include/teaser/graph.h
    sed -i 's/std::std::vector/std::vector/g' teaser/include/teaser/graph.h
    
    print_status "Vector fix applied"
fi

# Build TEASER++
mkdir -p build
cd build
cmake .. -DBUILD_PYTHON_BINDINGS=ON -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install

print_status "TEASER++ compiled and installed"

# Install Python bindings
cd python
pip3 install . --user

# Verify installation
if python3 -c "import teaserpp_python" 2>/dev/null; then
    print_status "TEASER++ Python bindings installed successfully"
else
    print_error "TEASER++ Python bindings installation failed"
    exit 1
fi

# ========================================================================
# STEP 8: Install Multi_LiCa
# ========================================================================
echo ""
echo "========================================================================="
echo "STEP 8: Installing Multi_LiCa"
echo "========================================================================="

cd "$ROS_WORKSPACE/src"

if [ -d "Multi_LiCa" ]; then
    print_warning "Multi_LiCa directory exists, skipping clone"
else
    git clone https://github.com/XbotGroup/Multi_LiCa.git
    print_status "Multi_LiCa cloned"
fi

# Apply patches if Nova_tf_calibration exists
if [ -d "$ROS_WORKSPACE/src/Nova_tf_calibration/patches" ]; then
    print_status "Applying Multi_LiCa patches..."
    cd "$ROS_WORKSPACE/src/Nova_tf_calibration"
    ./patches/apply_patches.sh
else
    print_warning "Nova_tf_calibration patches not found, skipping"
fi

# ========================================================================
# STEP 9: Build ROS 2 Workspace
# ========================================================================
echo ""
echo "========================================================================="
echo "STEP 9: Building ROS 2 Workspace"
echo "========================================================================="

cd "$ROS_WORKSPACE"
source /opt/ros/humble/setup.bash

colcon build --packages-select multi_lidar_calibrator --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -eq 0 ]; then
    print_status "Workspace built successfully"
else
    print_error "Workspace build failed"
    exit 1
fi

# ========================================================================
# STEP 10: Setup Environment
# ========================================================================
echo ""
echo "========================================================================="
echo "STEP 10: Setting Up Environment"
echo "========================================================================="

# Add to bashrc if not already there
if ! grep -q "source $ROS_WORKSPACE/install/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# Nova TF Calibration workspace" >> ~/.bashrc
    echo "source $ROS_WORKSPACE/install/setup.bash" >> ~/.bashrc
    print_status "Added workspace to ~/.bashrc"
else
    print_status "Workspace already in ~/.bashrc"
fi

# ========================================================================
# COMPLETION
# ========================================================================
echo ""
echo "========================================================================"
echo "✅ INSTALLATION COMPLETE!"
echo "========================================================================"
echo ""
echo "Next steps:"
echo "  1. Source your workspace: source $ROS_WORKSPACE/install/setup.bash"
echo "  2. Configure LiDARs: edit config/pcd_recording_config.yaml"
echo "  3. Record calibration data: ./scripts/record_calibration_data.sh"
echo "  4. Run calibration: ./scripts/run_calibration.sh"
echo ""
echo "For more information, see README.md"
echo ""