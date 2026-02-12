# Detailed Installation Guide

This document provides step-by-step installation instructions for the Nova TF Calibration system.

## Table of Contents
- [System Requirements](#system-requirements)
- [Manual Installation](#manual-installation)
- [Automated Installation](#automated-installation)
- [Verification](#verification)
- [Troubleshooting Installation](#troubleshooting-installation)
- [Platform-Specific Notes](#platform-specific-notes)
- [Next Steps](#next-steps)

---

## System Requirements

### Minimum Requirements
- **OS**: Ubuntu 22.04 or 20.04 LTS
- **RAM**: 8GB (16GB recommended)
- **Storage**: 10GB free space
- **CPU**: 4 cores (for reasonable compile times)

### Software Prerequisites
- **ROS 2 Humble** (or Foxy for Ubuntu 20.04)
- **Python 3.10+** (3.8+ for Ubuntu 20.04)
- **GCC/G++ 9+**
- **CMake 3.10+**

---

## Manual Installation

If the automated script fails or you prefer manual installation, follow these steps:

### Step 1: Install System Dependencies

```bash
# Update package list
sudo apt update

# Install build tools
sudo apt install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    python3-pip \
    python3-dev

# Install required libraries
sudo apt install -y \
    libeigen3-dev \
    libboost-all-dev \
    libgtest-dev \
    libgflags-dev \
    libgoogle-glog-dev
```

### Step 2: Install ROS 2 Dependencies

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Install ROS packages
sudo apt install -y \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-tf2-ros \
    ros-humble-pcl-ros \
    python3-colcon-common-extensions
```

### Step 3: Install Python Dependencies

```bash
# Upgrade pip
python3 -m pip install --upgrade pip

# Install required packages
pip3 install --user \
    numpy \
    scipy \
    open3d \
    pyyaml \
    matplotlib \
    ros2-numpy
```

### Step 4: Build TEASER++

```bash
# Create workspace
mkdir -p ~/drobot_lidar/src
cd ~/drobot_lidar/src

# Clone TEASER++
git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
cd TEASER-plusplus

# Apply vector fix
sed -i 's/\bvector</std::vector</g' teaser/src/graph.cc
sed -i 's/std::std::vector/std::vector/g' teaser/src/graph.cc
sed -i 's/\bvector</std::vector</g' teaser/include/teaser/graph.h
sed -i 's/std::std::vector/std::vector/g' teaser/include/teaser/graph.h

# Build
mkdir -p build && cd build
cmake .. -DBUILD_PYTHON_BINDINGS=ON -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install

# Install Python bindings
cd python
pip3 install . --user

# Verify
python3 -c "import teaserpp_python; print('TEASER++ installed successfully')"
```

### Step 5: Install Multi_LiCa

```bash
# Clone Multi_LiCa
cd ~/drobot_lidar/src
git clone https://github.com/XbotGroup/Multi_LiCa.git

# Clone Nova_tf_calibration
git clone https://github.com/Masudali23/Nova_tf_calibration.git

# Apply patches
cd Nova_tf_calibration
./patches/apply_patches.sh
```

### Step 6: Build Workspace

```bash
cd ~/drobot_lidar
source /opt/ros/humble/setup.bash
colcon build --packages-select multi_lidar_calibrator --symlink-install
source install/setup.bash
```

### Step 7: Add to Shell Configuration

```bash
# Add to ~/.bashrc
echo "source ~/drobot_lidar/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Automated Installation

Use the provided installation script:

```bash
git clone https://github.com/Masudali23/Nova_tf_calibration.git
cd Nova_tf_calibration
./scripts/install_dependencies.sh
```

The script will:
- Check system requirements
- Install all dependencies
- Build TEASER++ with Python bindings
- Clone and patch Multi_LiCa
- Build ROS 2 workspace
- Configure environment

**Estimated time:** 10-20 minutes

---

## Verification

After installation, verify everything works:

### 1. Check TEASER++

```bash
python3 -c "import teaserpp_python; print('✅ TEASER++ OK')"
```

### 2. Check Open3D

```bash
python3 -c "import open3d; print('✅ Open3D OK')"
```

### 3. Check Multi_LiCa

```bash
source ~/drobot_lidar/install/setup.bash
ros2 pkg list | grep multi_lidar_calibrator
```

Should output: `multi_lidar_calibrator`

### 4. Check Python Dependencies

```bash
python3 -c "
import numpy
import scipy
import yaml
import ros2_numpy
print('✅ All Python dependencies OK')
"
```

---

## Troubleshooting Installation

### Issue: TEASER++ Compilation Fails

**Error:**
```
error: 'vector' does not name a type
```

**Solution:** Apply the vector fix before compiling:

```bash
cd ~/drobot_lidar/src/TEASER-plusplus
sed -i 's/\bvector</std::vector</g' teaser/src/graph.cc
sed -i 's/\bvector</std::vector</g' teaser/include/teaser/graph.h
# Then rebuild
```

### Issue: Python Bindings Not Found

**Error:**
```
ModuleNotFoundError: No module named 'teaserpp_python'
```

**Solution:**

```bash
cd ~/drobot_lidar/src/TEASER-plusplus/build/python
pip3 install . --user --force-reinstall
```

### Issue: Multi_LiCa Build Fails

**Error:**
```
Could not find a package configuration file provided by "teaser"
```

**Solution:** TEASER++ not installed system-wide:

```bash
cd ~/drobot_lidar/src/TEASER-plusplus/build
sudo make install
sudo ldconfig
```

### Issue: ROS 2 Not Found

**Error:**
```
Package 'ros-humble-sensor-msgs' not found
```

**Solution:** Install ROS 2 Humble first:

```bash
# Follow official guide
# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
```

### Issue: Eigen3 Not Found

**Error:**
```
Could not find Eigen3
```

**Solution:**

```bash
sudo apt install libeigen3-dev
# Create symlink if needed
sudo ln -s /usr/include/eigen3/Eigen /usr/local/include/Eigen
```

---

## Platform-Specific Notes

### Ubuntu 20.04
- Use ROS 2 Foxy instead of Humble
- Python 3.8 instead of 3.10
- Some package names may differ slightly

### ARM64 (Jetson, Raspberry Pi)
- Compilation will be slower (allow 30-60 minutes)
- Consider reducing parallel jobs: `make -j2` instead of `make -j$(nproc)`
- May need to increase swap space

### Docker
A Dockerfile is provided for containerized installation:

```bash
docker build -t nova-tf-calibration .
docker run -it --rm nova-tf-calibration
```

---

## Next Steps

After successful installation:

1. Configure your LiDARs: `config/pcd_recording_config.yaml`
2. Record calibration data: `./scripts/record_calibration_data.sh`
3. Run calibration: `./scripts/run_calibration.sh`

For usage instructions, see the main README.md.