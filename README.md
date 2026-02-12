# Nova TF Calibration

**Precise multi-LiDAR extrinsic calibration for robotic platforms using Multi_LiCa and TEASER++**

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://ubuntu.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Complete guide for calibrating the spatial relationship (transformation) between multiple LiDAR sensors on a robot, validated on Nova/CyberDog platforms.

![Calibration Result](docs/images/calibration_result.png)

---

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Quick Start](#quick-start)
- [Detailed Guide](#detailed-guide)
  - [1. Recording PCD Files](#1-recording-pcd-files)
  - [2. Installation & Dependencies](#2-installation--dependencies)
  - [3. Configuration](#3-configuration)
  - [4. Running Calibration](#4-running-calibration)
  - [5. Visualization](#5-visualization)
- [How It Works](#how-it-works)
- [Code Modifications](#code-modifications)
- [Troubleshooting](#troubleshooting)
- [Results](#results)
- [Contributing](#contributing)
- [Acknowledgments](#acknowledgments)

---

## Overview

This repository provides a **complete, tested workflow** for multi-LiDAR calibration using:
- **Multi_LiCa**: Multi-LiDAR calibration framework with GICP refinement
- **TEASER++**: Fast and certifiably-robust point cloud registration
- **Open3D**: Point cloud processing and visualization

The calibration process automatically computes the 6-DOF transformation (translation + rotation) between LiDAR sensors, enabling accurate sensor fusion for SLAM, navigation, and perception tasks.

### What's Included
✅ PCD recording scripts  
✅ Complete installation guide with all fixes  
✅ Ready-to-use configuration files  
✅ Visualization tools  
✅ Documented code patches  
✅ Troubleshooting guide  

---

## Features

- **Automatic calibration** from static point cloud captures
- **Robust registration** using TEASER++ (outlier-resistant)
- **High accuracy** with GICP (Generalized ICP) refinement
- **No checkerboard required** - works with natural scenes
- **ROS 2 integration** with TF publishers
- **Visualization tools** for quality assessment

---

## Prerequisites

### Hardware
- Robot with 2+ LiDAR sensors
- LiDARs must have **overlapping field of view**
- Stationary calibration setup recommended

### Software
- **Ubuntu 22.04** (20.04 also works)
- **ROS 2 Humble**
- **Python 3.10+**
- **CMake 3.10+**
- **GCC/G++ 9+**

---

## Quick Start

```bash
# 1. Clone repository
git clone https://github.com/Masudali23/Nova_tf_calibration.git
cd Nova_tf_calibration

# 2. Install dependencies
./scripts/install_dependencies.sh

# 3. Record calibration data
python3 scripts/record_pcds.py --topic /pointcloud_front --lidar_name front --outdir data/
python3 scripts/record_pcds.py --topic /pointcloud_back --lidar_name back --outdir data/

# 4. Run calibration
./scripts/run_calibration.sh

# 5. Visualize results
python3 scripts/visualize_results.py
```

---

## Detailed Guide

### 1. Recording PCD Files

Point cloud data must be collected from all LiDARs while the robot is **stationary** and observing a **feature-rich environment** (not a blank wall).

#### 1.1 Automatic Recording Script

```python
# scripts/record_pcds.py
#!/usr/bin/env python3
"""
Record PointCloud2 messages to PCD files for calibration.
Captures multiple frames per LiDAR for robust calibration.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import ros2_numpy as rnp
import os
import argparse

class PCDRecorder(Node):
    def __init__(self, topic, lidar_name, save_dir, frame_count=10):
        super().__init__('pcd_recorder')
        self.subscription = self.create_subscription(
            PointCloud2, topic, self.listener_callback, 10)
        self.save_dir = save_dir
        self.lidar_name = lidar_name
        self.count = 0
        self.max_count = frame_count
        
        os.makedirs(save_dir, exist_ok=True)
        self.get_logger().info(f'Recording {frame_count} frames from {topic}...')

    def listener_callback(self, msg):
        if self.count >= self.max_count:
            return
            
        try:
            # Convert ROS PointCloud2 to numpy array
            points_array = rnp.numpify(msg)
            
            # Create Open3D point cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points_array['xyz'])
            
            # Save to file
            filename = f"{self.lidar_name}_{self.count:03d}.pcd"
            filepath = os.path.join(self.save_dir, filename)
            o3d.io.write_point_cloud(filepath, pcd)
            
            self.get_logger().info(f'Saved: {filename} ({len(pcd.points)} points)')
            self.count += 1
            
            if self.count == self.max_count:
                self.get_logger().info('Recording complete!')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Error saving PCD: {e}')

def main():
    parser = argparse.ArgumentParser(description='Record PCD files from ROS topics')
    parser.add_argument('--topic', required=True, help='PointCloud2 topic name')
    parser.add_argument('--lidar_name', required=True, help='LiDAR identifier (e.g., front, back)')
    parser.add_argument('--outdir', required=True, help='Output directory for PCD files')
    parser.add_argument('--frames', type=int, default=10, help='Number of frames to record')
    args = parser.parse_args()
    
    rclpy.init()
    recorder = PCDRecorder(args.topic, args.lidar_name, args.outdir, args.frames)
    rclpy.spin(recorder)
    recorder.destroy_node()

if __name__ == '__main__':
    main()
```

#### 1.2 Usage

```bash
# Terminal 1: Start your robot and LiDAR drivers
ros2 launch your_robot_bringup bringup.launch.py

# Terminal 2: Record front LiDAR
python3 scripts/record_pcds.py \
  --topic /pointcloud_front \
  --lidar_name front \
  --outdir ~/calibration_data/ \
  --frames 10

# Terminal 3: Record back LiDAR
python3 scripts/record_pcds.py \
  --topic /pointcloud_back \
  --lidar_name back \
  --outdir ~/calibration_data/ \
  --frames 10
```

#### 1.3 Verify Data

```bash
ls -lh ~/calibration_data/
# Should show:
# front_000.pcd, front_001.pcd, ..., front_009.pcd
# back_000.pcd, back_001.pcd, ..., back_009.pcd
```

---

### 2. Installation & Dependencies

#### 2.1 System Dependencies

```bash
# Update package lists
sudo apt update

# Install build tools
sudo apt install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libboost-all-dev \
    python3-pip \
    python3-colcon-common-extensions

# Install ROS 2 dependencies
sudo apt install -y \
    ros-humble-sensor-msgs \
    ros-humble-tf2-ros \
    ros-humble-pcl-ros

# Install Python packages
pip3 install --user \
    open3d \
    numpy \
    scipy \
    ros2-numpy
```

#### 2.2 Install TEASER++

TEASER++ requires compilation from source with Python bindings.

```bash
cd ~/drobot_lidar/src
git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
cd TEASER-plusplus
mkdir build && cd build
cmake .. -DBUILD_PYTHON_BINDINGS=ON
make -j$(nproc)
sudo make install
```

**⚠️ Common Issue: Missing `std::vector`**

If you encounter errors like `'vector' does not name a type`, apply this fix:

```bash
# Fix graph.cc
cd ~/drobot_lidar/src/TEASER-plusplus/teaser/src
nano graph.cc
```

Add at the top (after existing includes):
```cpp
#include <vector>
```

Replace all `vector<` with `std::vector<`:
```bash
sed -i 's/\bvector</std::vector</g' graph.cc
sed -i 's/std::std::vector/std::vector/g' graph.cc
```

Do the same for `graph.h`:
```bash
cd ../include/teaser
sed -i 's/\bvector</std::vector</g' graph.h
sed -i 's/std::std::vector/std::vector/g' graph.h
```

Rebuild:
```bash
cd ~/drobot_lidar/src/TEASER-plusplus/build
make clean
make -j$(nproc)
sudo make install
```

**Install Python Bindings:**

```bash
cd ~/drobot_lidar/src/TEASER-plusplus/build/python
pip3 install . --user

# Verify installation
python3 -c "import teaserpp_python; print('✅ TEASER++ installed!')"
```

#### 2.3 Install Multi_LiCa

```bash
cd ~/drobot_lidar/src
git clone https://github.com/XbotGroup/Multi_LiCa.git
```

**Apply Code Fixes** (see [Code Modifications](#code-modifications) section)

#### 2.4 Build ROS 2 Workspace

```bash
cd ~/drobot_lidar
source /opt/ros/humble/setup.bash
colcon build --packages-select multi_lidar_calibrator --symlink-install
source install/setup.bash
```

---

### 3. Configuration

Create configuration file `config/multi_lica_config.yaml`:

```yaml
/**:
  ros__parameters:
    # Data collection
    frame_count: 10
    runs_count: 1
    read_pcds_from_file: true
    pcd_directory: "/home/YOUR_USER/calibration_data/"
    
    # LiDAR topics (only used if reading from live ROS)
    lidar_topics: [/pointcloud_front, /pointcloud_back]
    tf_topic: /tf_static
    
    # Initial transformation estimates [x, y, z, roll, pitch, yaw]
    # These are rough guesses - calibration will refine them
    read_tf_from_table: true
    table_degrees: false  # Use radians
    
    front: [0.31, 0.0, 0.22, 0.0, 0.0, 0.0]  # Front LiDAR pose
    back: [-0.26, 0.0, 0.22, 0.0, 0.0, 0.0]  # Back LiDAR pose
    
    # Calibration settings
    target_frame_id: front  # Reference frame
    base_frame_id: base_link
    calibrate_target: false
    calibrate_to_base: false
    
    # Output
    output_dir: "/home/YOUR_USER/calibration_output/"
    visualize: false  # Set true if you have display
    
    # Algorithm parameters
    use_fitness_based_calibration: true
    max_corresp_dist: 0.5
    rel_fitness: 0.0000001
    rel_rmse: 0.0000001
    max_iterations: 100
    epsilon: 0.005
    voxel_size: 0.05
    remove_ground_flag: false
    fitness_score_threshold: 0.15
    
    # RANSAC parameters
    distance_threshold: 0.1
    ransac_n: 10
    num_iterations: 2000
    r_voxel_size: 0.1
    r_runs: 10
    base_to_ground_z: 0.22
```

---

### 4. Running Calibration

#### 4.1 Using Launch File

```bash
source /opt/ros/humble/setup.bash
source ~/drobot_lidar/install/setup.bash

ros2 launch multi_lidar_calibrator calibration.launch.py \
  parameter_file:=/path/to/multi_lica_config.yaml
```

#### 4.2 Using Convenience Script

```bash
#!/bin/bash
# scripts/run_calibration.sh

source /opt/ros/humble/setup.bash
source ~/drobot_lidar/install/setup.bash

# Create output directory
mkdir -p ~/calibration_output

# Run calibration
ros2 launch multi_lidar_calibrator calibration.launch.py \
  parameter_file:=$(pwd)/config/multi_lica_config.yaml

echo "✅ Calibration complete!"
echo "Results: ~/calibration_output/"
```

#### 4.3 Expected Output

```
[INFO] PCD directory: /home/user/calibration_data/
[INFO] Detected lidars: ['back', 'front']
[INFO] Found 10 files for back
[INFO] Found 10 files for front
[INFO] Starting the calibration...

Starting scale solver...
Scale estimation complete.
Starting rotation solver...
Rotation estimation complete.
Starting translation solver...
Translation estimation complete.

[INFO] calibration info:
back to front calibration
calibrated xyz = 0.568 0.077 0.009
calibrated rpy = 0.008 -0.023 0.007
fitness: 0.397, inlier_rmse: 0.142

[INFO] Saved fused point cloud: /home/user/calibration_output/
[INFO] Calibrations results are stored in: /home/user/calibration_output/
```

---

### 5. Visualization

#### 5.1 View Calibrated Point Cloud

```python
# scripts/visualize_results.py
import open3d as o3d

# Load calibrated result
pcd = o3d.io.read_point_cloud("~/calibration_output/stitched_transformed.pcd")
print(f"Loaded {len(pcd.points)} points")

# Visualize
o3d.visualization.draw_geometries(
    [pcd],
    window_name="Calibrated LiDAR Point Cloud",
    width=1280,
    height=720
)
```

#### 5.2 Compare Before/After

```python
import open3d as o3d

# Load both
before = o3d.io.read_point_cloud("~/calibration_output/stitched_initial.pcd")
after = o3d.io.read_point_cloud("~/calibration_output/stitched_transformed.pcd")

# Color differently
before.paint_uniform_color([1, 0, 0])  # Red
after.paint_uniform_color([0, 1, 0])   # Green

# Show side-by-side
before.translate([2, 0, 0])
o3d.visualization.draw_geometries([before, after])
```

---

## How It Works

### Calibration Pipeline

```
┌─────────────────┐
│  Record PCDs    │  10 frames per LiDAR
│  from ROS       │  (stationary, feature-rich scene)
└────────┬────────┘
         │
         v
┌─────────────────┐
│  Load & Filter  │  Voxel downsampling
│  Point Clouds   │  Optional ground removal
└────────┬────────┘
         │
         v
┌─────────────────┐
│  TEASER++       │  Global registration
│  Registration   │  Outlier-robust
└────────┬────────┘
         │
         v
┌─────────────────┐
│  GICP           │  Fine refinement
│  Refinement     │  Point-to-plane ICP
└────────┬────────┘
         │
         v
┌─────────────────┐
│  Output TF      │  6-DOF transformation
│  & Stitched PCD │  Quality metrics
└─────────────────┘
```

### Algorithm Details

1. **Feature Extraction**: FPFH (Fast Point Feature Histograms) computed for each point cloud
2. **Correspondence Estimation**: Feature matching with RANSAC-based filtering
3. **TEASER++ Optimization**:
   - Scale estimation (disabled for LiDAR-to-LiDAR)
   - Rotation estimation using GNC-TLS (Graduated Non-Convexity)
   - Translation estimation
4. **GICP Refinement**: Point-to-plane ICP for sub-centimeter accuracy
5. **Quality Assessment**: Fitness score and RMSE metrics

---

## Code Modifications

### Required Changes to Multi_LiCa

#### 1. Fix LiDAR Name Extraction

**File:** `multi_lidar_calibrator/multi_lidar_calibrator.py`  
**Location:** Lines 100-108

**Original:**
```python
lidar_list = [
    os.path.splitext(os.path.basename(path))[0]
    for path in glob.glob(self.pcd_in_dir + "*")
]
```

**Fixed:**
```python
# Extract unique lidar prefixes from PCD filenames
all_pcd_files = glob.glob(self.pcd_in_dir + "*.pcd")
lidar_list = list(set([
    os.path.basename(f).split('_')[0]
    for f in all_pcd_files
    if f.endswith('.pcd')
]))

# Debug logging
self.get_logger().info(f"PCD directory: {self.pcd_in_dir}")
self.get_logger().info(f"Detected lidars: {lidar_list}")

# Ensure files are sorted
for lidar in lidar_list:
    self.declare_parameter(lidar, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    pcd_paths[lidar] = sorted(glob.glob(self.pcd_in_dir + lidar + "_*.pcd"))
    self.get_logger().info(f"Found {len(pcd_paths[lidar])} files for {lidar}")
```

#### 2. Fix Absolute Path Handling

**File:** `multi_lidar_calibrator/multi_lidar_calibrator.py`  
**Location:** Lines 66-75

**Original:**
```python
self.output_dir = (
    os.path.dirname(os.path.realpath(__file__))
    + self.declare_parameter("output_dir", "/../output/").value
)
self.pcd_in_dir = (
    os.path.dirname(os.path.realpath(__file__))
    + self.declare_parameter("pcd_directory", "/../data/demo/").value
)
```

**Fixed:**
```python
# Handle output directory (absolute or relative)
output_dir_param = self.declare_parameter("output_dir", "/../output/").value
if os.path.isabs(output_dir_param):
    self.output_dir = output_dir_param
else:
    self.output_dir = os.path.dirname(os.path.realpath(__file__)) + output_dir_param

if not os.path.exists(self.output_dir):
    os.makedirs(self.output_dir)

# Handle PCD directory (absolute or relative)
pcd_dir_param = self.declare_parameter("pcd_directory", "/../data/demo/").value
if os.path.isabs(pcd_dir_param):
    self.pcd_in_dir = pcd_dir_param
else:
    self.pcd_in_dir = os.path.dirname(os.path.realpath(__file__)) + pcd_dir_param
```

### Apply Patches

```bash
# Apply all patches at once
cd ~/drobot_lidar/src/Multi_LiCa
patch -p1 < ~/Nova_tf_calibration/patches/multi_lidar_calibrator.patch
```

---

## Troubleshooting

### Common Issues

#### 1. **No lidars detected**
```
[INFO] Detected lidars: []
KeyError: 'front'
```

**Solution:** 
- Verify PCD files exist: `ls ~/calibration_data/*.pcd`
- Ensure config uses absolute path: `pcd_directory: "/full/path/to/calibration_data/"`
- Check filename format: `front_000.pcd`, `back_000.pcd`, etc.

#### 2. **TEASER++ import error**
```
ModuleNotFoundError: No module named 'teaserpp_python'
```

**Solution:**
```bash
cd ~/drobot_lidar/src/TEASER-plusplus/build/python
pip3 install . --user
python3 -c "import teaserpp_python"  # Verify
```

#### 3. **Permission denied on output directory**
```
PermissionError: [Errno 13] Permission denied: '/../output/'
```

**Solution:**
- Use absolute path in config: `output_dir: "/home/user/calibration_output/"`
- Create directory: `mkdir -p ~/calibration_output`

#### 4. **Low fitness score**
```
fitness: 0.05, inlier_rmse: 0.8
```

**Solution:**
- Ensure LiDARs have overlapping views
- Collect data in feature-rich environment (not blank walls)
- Adjust `voxel_size` (try 0.03 for denser clouds)
- Increase `max_corresp_dist` (try 1.0)

#### 5. **Open3D visualization fails**
```
[Open3D WARNING] Failed to initialize GLEW
```

**Solution:**
- Set `visualize: false` in config
- Use headless mode and visualize output PCDs separately
- Export display: `export DISPLAY=:0`

---

## Results

### Example Output

**Transformation Matrix (back → front):**
```
[[ 0.99999991 -0.00012675 -0.00040607  0.56843389]
 [ 0.00012669  0.99999998 -0.00014519  0.07693556]
 [ 0.00040609  0.00014514  0.99999991  0.00872941]
 [ 0.00000000  0.00000000  0.00000000  1.00000000]]
```

**Translation:** (0.568m, 0.077m, 0.009m)  
**Rotation:** (0.48°, -1.34°, 0.42°)  
**Fitness:** 0.397  
**RMSE:** 0.142m  

### Using Results in ROS 2

#### Static TF Publisher

```bash
ros2 run tf2_ros static_transform_publisher \
  0.568 0.077 0.009 \
  0.007 -0.023 0.008 \
  front_lidar back_lidar
```

#### URDF Integration

```xml
<joint name="back_lidar_joint" type="fixed">
  <parent link="front_lidar_link"/>
  <child link="back_lidar_link"/>
  <origin xyz="0.568 0.077 0.009" rpy="0.008 -0.023 0.007"/>
</joint>
```

---

## Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

---

## Acknowledgments

- [Multi_LiCa](https://github.com/XbotGroup/Multi_LiCa) by XbotGroup
- [TEASER++](https://github.com/MIT-SPARK/TEASER-plusplus) by MIT SPARK Lab
- [Open3D](http://www.open3d.org/)
- ROS 2 Community

---

## License

MIT License - see [LICENSE](LICENSE) file

---

## Citation

If you use this work in your research, please cite:

```bibtex
@misc{nova_tf_calibration,
  author = {Your Name},
  title = {Nova TF Calibration: Multi-LiDAR Extrinsic Calibration},
  year = {2026},
  publisher = {GitHub},
  url = {https://github.com/Masudali23/Nova_tf_calibration}
}
```

---

**Questions? Issues? Open a GitHub issue or discussion!** 🚀
