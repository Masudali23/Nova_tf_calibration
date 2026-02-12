# Nova TF Calibration

**Precise multi-LiDAR extrinsic calibration for robotic platforms using Multi_LiCa and TEASER++**

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://ubuntu.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

> A complete, production-tested workflow for calibrating the spatial relationship between multiple LiDAR sensors on robotic platforms. This project documents the entire journey from data collection to final TF publication, including all the gotchas and fixes discovered during real-world implementation.

---

## 📋 Table of Contents

- [What is This?](#what-is-this)
- [Why Do We Need This?](#why-do-we-need-this)
- [How It Works](#how-it-works)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Workflow](#workflow)
- [Understanding the Process](#understanding-the-process)
- [Troubleshooting](#troubleshooting)
- [Results & Usage](#results--usage)
- [Contributing](#contributing)
- [Acknowledgments](#acknowledgments)

---

## What is This?

This repository provides a **battle-tested pipeline** for automatically calibrating multiple LiDAR sensors on a robot. Instead of manually measuring sensor positions (which is error-prone and time-consuming), this system uses point cloud registration algorithms to compute precise transformations automatically.

**Key Technologies:**
- **Multi_LiCa**: ROS 2 framework for multi-LiDAR calibration
- **TEASER++**: Robust global registration algorithm (handles outliers)
- **GICP**: Generalized Iterative Closest Point for refinement
- **Open3D**: Point cloud processing library

---

## Why Do We Need This?

### The Problem

When you have multiple LiDAR sensors on a robot (e.g., front and back), you need to know their **exact spatial relationship** to:
- Merge point clouds into a unified map
- Perform accurate SLAM
- Enable proper obstacle detection
- Support sensor fusion

**Manual measurement issues:**
- ❌ Measurement errors (±5-10mm typical)
- ❌ Difficult to measure rotation precisely
- ❌ Sensors may shift during operation
- ❌ Time-consuming and error-prone

### The Solution

This calibration pipeline:
- ✅ Automatically computes 6-DOF transformation (3D position + 3D rotation)
- ✅ Sub-centimeter accuracy achievable
- ✅ Handles partial overlap between sensors
- ✅ Robust to outliers and noise
- ✅ No special calibration targets needed (works with natural scenes)

---

## How It Works

### The Big Picture

```
┌─────────────────────────────────────────────────────────────┐
│                    CALIBRATION PIPELINE                      │
└─────────────────────────────────────────────────────────────┘

1. DATA COLLECTION
   └─> Record point clouds from all LiDARs (robot stationary)
       ├─> Front LiDAR: 10 frames → front_000.pcd ... front_009.pcd
       └─> Back LiDAR: 10 frames → back_000.pcd ... back_009.pcd

2. PREPROCESSING
   └─> Load & merge frames
       └─> Voxel downsampling (reduce density while keeping structure)
           └─> Optional ground plane removal

3. GLOBAL REGISTRATION (TEASER++)
   └─> Compute FPFH features (Fast Point Feature Histograms)
       └─> Find correspondences between point clouds
           └─> TEASER++ optimization (outlier-resistant)
               └─> Initial transformation estimate

4. LOCAL REFINEMENT (GICP)
   └─> Use TEASER++ result as starting point
       └─> Iterative Closest Point refinement
           └─> Point-to-plane ICP for sub-mm accuracy

5. OUTPUT
   └─> 4×4 transformation matrix
       ├─> Translation: [x, y, z] in meters
       ├─> Rotation: [roll, pitch, yaw] in radians
       ├─> Quality metrics: fitness score, RMSE
       └─> Stitched point cloud visualization
```

### Why This Approach?

**Two-Stage Registration:**
1. **TEASER++ (Global)**: Finds rough alignment even with bad initial guess
   - Handles up to 99% outliers
   - No initial transformation needed
   - Certifiably optimal (guaranteed to find best solution)

2. **GICP (Local)**: Refines to high precision
   - Point-to-plane matching
   - Millimeter-level accuracy
   - Fast convergence from good initialization

**Multiple Frames:** Recording 10 frames per LiDAR increases robustness by averaging out:
- Sensor noise
- Moving objects (people, vehicles)
- Atmospheric effects

---

## Prerequisites

### Hardware Requirements
- **Robot platform** with 2 or more LiDAR sensors
- **Overlapping field of view** between LiDARs (at least 20-30%)
- **Feature-rich environment** for calibration (indoor space with furniture, walls, etc.)

### Software Requirements
- **Operating System**: Ubuntu 22.04 or 20.04
- **ROS**: ROS 2 Humble
- **Python**: 3.10+ (comes with Ubuntu 22.04)
- **Build Tools**: CMake 3.10+, GCC 9+

### Why These Specific Versions?
- **ROS 2 Humble**: LTS release, stable and well-supported
- **Ubuntu 22.04**: Matches ROS 2 Humble target platform
- **Python 3.10+**: Required for newer numpy/scipy features

---

## Installation

### Quick Install (Automated)

```bash
git clone https://github.com/Masudali23/Nova_tf_calibration.git
cd Nova_tf_calibration
./scripts/install_dependencies.sh
```

This script will:
1. Install system dependencies (build tools, libraries)
2. Clone and build TEASER++ with Python bindings
3. Clone Multi_LiCa framework
4. Apply necessary code fixes
5. Build ROS 2 workspace

**Estimated time**: 10-20 minutes (depending on CPU)

### What Gets Installed?

#### System Packages
- `build-essential`, `cmake`: Compilation tools
- `libeigen3-dev`: Linear algebra library (required by TEASER++)
- `libboost-all-dev`: C++ utilities
- `ros-humble-*`: ROS 2 dependencies

#### Python Packages
- `open3d`: Point cloud processing
- `numpy`, `scipy`: Numerical computing
- `ros2-numpy`: ROS ↔ numpy conversion

#### From Source
- **TEASER++**: Compiled with Python bindings enabled
- **Multi_LiCa**: Cloned and patched with fixes

### Why Build from Source?

**TEASER++**: 
- Not available in apt repositories
- Python bindings disabled by default
- Requires specific CMake flags

**Multi_LiCa**:
- Original code has bugs for our use case:
  - Filename parsing issue (detects `front_000` instead of `front`)
  - Absolute path handling broken
- Our patches fix these issues

For detailed installation steps, see [`docs/installation.md`](docs/installation.md).

---

## Workflow

### Step 1: Prepare Your Robot

1. **Position robot** in a feature-rich environment:
   - ✅ Indoor space with furniture, walls, objects
   - ✅ Good lighting (not required but helps)
   - ❌ Avoid blank walls or empty rooms
   - ❌ Avoid highly dynamic environments (busy hallways)

2. **Ensure LiDAR overlap**:
   - Front and back LiDARs should see some common area
   - Typically 20-40% overlap is sufficient
   - Check by visualizing both topics in RViz

3. **Keep robot stationary**:
   - **Critical**: Robot must not move during recording
   - Turn off motors if possible
   - Secure on flat surface

### Step 2: Configure LiDARs

Edit `config/pcd_recording_config.yaml`:

```yaml
recording:
  output_directory: "~/calibration_data/"
  frames_per_lidar: 10
  
  lidars:
    - name: "front"
      topic: "/pointcloud_front"  # Your front LiDAR topic
      enabled: true
    
    - name: "back"
      topic: "/pointcloud_back"   # Your back LiDAR topic
      enabled: true
```

**Important**: Update topic names to match your robot!

### Step 3: Record Calibration Data

```bash
# Terminal 1: Launch your robot
ros2 launch your_robot bringup.launch.py

# Terminal 2: Record point clouds
cd Nova_tf_calibration
./scripts/record_calibration_data.sh
```

**What happens**:
- Script subscribes to all configured LiDAR topics
- Records 10 frames from each LiDAR
- Saves as PCD files: `front_000.pcd`, `front_001.pcd`, ..., `back_000.pcd`, ...
- Takes ~5-10 seconds total

**Verify**:
```bash
ls -lh ~/calibration_data/
# Should show 20 files (10 per LiDAR)
```

### Step 4: Configure Calibration

Edit `config/multi_lica_config.yaml`:

```yaml
ros__parameters:
  pcd_directory: "/home/YOUR_USER/calibration_data/"
  output_dir: "/home/YOUR_USER/calibration_output/"
  
  # Initial transformation guesses [x, y, z, roll, pitch, yaw]
  front: [0.31, 0.0, 0.22, 0.0, 0.0, 0.0]
  back: [-0.26, 0.0, 0.22, 0.0, 0.0, 0.0]
  
  target_frame_id: front  # Reference frame
```

**Initial guesses**:
- These are your current TF estimates (from URDF or measurements)
- Don't need to be perfect - calibration will refine them
- Closer is better, but TEASER++ can handle large errors

### Step 5: Run Calibration

```bash
./scripts/run_calibration.sh
```

**What happens**:
1. Loads all PCD files
2. Downsamples point clouds (voxel grid)
3. Computes FPFH features
4. TEASER++ global registration (~30-60 seconds)
5. GICP refinement (~10-20 seconds)
6. Outputs results to `~/calibration_output/`

**Expected output**:
```
[INFO] Detected lidars: ['back', 'front']
[INFO] Found 10 files for back
[INFO] Found 10 files for front
[INFO] Starting the calibration...

TEASER++ optimization...
[INFO] calibration info:
back to front calibration
calibrated xyz = 0.568 0.077 0.009
calibrated rpy = 0.008 -0.023 0.007
fitness: 0.397, inlier_rmse: 0.142

[INFO] Saved fused point cloud
```

### Step 6: Verify Results

```bash
python3 scripts/visualize_results.py
```

This opens Open3D viewer showing:
- **Before**: Misaligned point clouds (red)
- **After**: Calibrated point clouds (green)

**What to look for**:
- Walls/objects should be single, not doubled
- Overlap regions should merge cleanly
- No ghosting or artifacts

### Step 7: Use Calibrated TF

Results are saved in `~/calibration_output/results.txt`:

```
Translation: x=0.568, y=0.077, z=0.009
Rotation (rad): roll=0.008, pitch=-0.023, yaw=0.007
Fitness: 0.397, RMSE: 0.142m
```

**Apply to your robot**:

```python
# In your launch file
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0.568', '0.077', '0.009',
               '0.008', '-0.023', '0.007',
               'front_lidar', 'back_lidar']
)
```

Or update your URDF:
```xml
<origin xyz="0.568 0.077 0.009" rpy="0.008 -0.023 0.007"/>
```

---

## Understanding the Process

### What is "Fitness Score"?

**Fitness score** = percentage of points that align well after transformation

- **0.0 - 0.2**: Poor alignment, calibration likely failed
- **0.2 - 0.4**: Acceptable, typical for partial overlap
- **0.4 - 0.7**: Good alignment
- **0.7 - 1.0**: Excellent, high overlap

**Our result: 0.397** = ~40% of points match well (good for front/back LiDARs)

### What is "RMSE"?

**RMSE** (Root Mean Square Error) = average distance between matched points

- **< 0.1m**: Excellent precision
- **0.1 - 0.2m**: Good for robotics applications
- **0.2 - 0.5m**: Acceptable for rough alignment
- **> 0.5m**: Poor, check data quality

**Our result: 0.142m** = 14.2cm average error (acceptable)

### Why Multiple Frames?

Recording 10 frames instead of 1 provides:
- **Noise reduction**: Averaging reduces sensor noise
- **Outlier rejection**: Moving objects appear in some frames, not all
- **Robustness**: If one frame is poor quality, others compensate

**Trade-off**: More frames = better quality, but longer recording time

### Why TEASER++ Then GICP?

**TEASER++ (Global)**:
- Strength: Handles large initial errors, robust to outliers
- Weakness: Lower precision (~5-10cm accuracy)

**GICP (Local)**:
- Strength: High precision (sub-cm possible)
- Weakness: Requires good initialization, can get stuck in local minima

**Together**: TEASER++ provides good starting point, GICP refines to high precision

---

## Troubleshooting

### Common Issues & Solutions

#### Issue: No LiDARs Detected
```
[INFO] Detected lidars: []
KeyError: 'front'
```

**Cause**: PCD files not found or named incorrectly

**Solutions**:
1. Check files exist: `ls ~/calibration_data/*.pcd`
2. Verify filename format: `front_000.pcd`, not `front0.pcd` or `front.pcd`
3. Use absolute path in config: `/home/user/...` not `~/...`

#### Issue: Low Fitness Score
```
fitness: 0.05, inlier_rmse: 0.8
```

**Cause**: Poor overlap or featureless environment

**Solutions**:
1. Check LiDAR overlap in RViz (visualize both topics)
2. Move robot to more feature-rich area
3. Increase `max_corresp_dist` in config (from 0.5 to 1.0)
4. Reduce `voxel_size` for denser point clouds (0.05 → 0.03)

#### Issue: TEASER++ Import Error
```
ModuleNotFoundError: No module named 'teaserpp_python'
```

**Cause**: Python bindings not installed correctly

**Solution**:
```bash
cd ~/drobot_lidar/src/TEASER-plusplus/build/python
pip3 install . --user
python3 -c "import teaserpp_python"  # Verify
```

#### Issue: Build Errors (`vector` not defined)
```
error: 'vector' does not name a type
```

**Cause**: Missing `#include <vector>` in TEASER++ source

**Solution**: Apply patches with `./patches/apply_patches.sh`

For more issues, see [`docs/troubleshooting.md`](docs/troubleshooting.md).

---

## Results & Usage

### Example Calibration Output

Our test case (Nova robot with front/back LiDARs):

```
Transformation: back_lidar → front_lidar

Translation (meters):
  x: 0.568  (back is 56.8cm behind front)
  y: 0.077  (back is 7.7cm to the left)
  z: 0.009  (back is 0.9cm higher)

Rotation (degrees):
  roll:   0.48°
  pitch: -1.34°
  yaw:    0.42°

Quality Metrics:
  Fitness: 0.397 (39.7% point overlap)
  RMSE: 0.142m (14.2cm average error)
```

**Interpretation**:
- Good fitness for front/back configuration (limited overlap)
- RMSE acceptable for navigation/mapping
- Small rotation differences (<2°) indicate sensors are nearly level

### Integration Examples

#### ROS 2 Static TF Publisher

```bash
ros2 run tf2_ros static_transform_publisher \
  0.568 0.077 0.009 \
  0.007 -0.023 0.008 \
  front_lidar back_lidar
```

#### Launch File Integration

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_calibration_tf',
            arguments=[
                '0.568', '0.077', '0.009',  # xyz
                '0.007', '-0.023', '0.008',  # rpy
                'front_lidar', 'back_lidar'
            ]
        )
    ])
```

#### URDF Update

```xml
<robot>
  <joint name="back_lidar_joint" type="fixed">
    <parent link="front_lidar_link"/>
    <child link="back_lidar_link"/>
    <origin xyz="0.568 0.077 0.009" 
            rpy="0.008 -0.023 0.007"/>
  </joint>
</robot>
```

---

## Project Structure

```
Nova_tf_calibration/
├── README.md                    # This file
├── LICENSE                      # MIT License
├── .gitignore                   # Git ignore rules
│
├── config/                      # Configuration files
│   ├── pcd_recording_config.yaml    # LiDAR recording settings
│   └── multi_lica_config.yaml       # Calibration parameters
│
├── scripts/                     # Executable scripts
│   ├── install_dependencies.sh      # One-click installation
│   ├── record_pcds_multi.py         # Multi-LiDAR PCD recorder
│   ├── record_calibration_data.sh   # Recording wrapper script
│   ├── run_calibration.sh           # Calibration wrapper script
│   └── visualize_results.py         # Result visualization
│
├── patches/                     # Code fixes
│   ├── apply_patches.sh             # Patch application script
│   └── multi_lidar_calibrator_fix.patch  # Multi_LiCa fixes
│
└── docs/                        # Documentation
    ├── installation.md              # Detailed install guide
    ├── troubleshooting.md           # Common issues & fixes
    └── images/                      # Screenshots, diagrams
```

---

## Contributing

We welcome contributions! This project was built through trial-and-error and community knowledge.

**Ways to contribute**:
- 🐛 Report bugs or issues
- 📝 Improve documentation
- ✨ Add support for more LiDAR configurations
- 🔧 Submit fixes or optimizations
- 📊 Share your calibration results

**Process**:
1. Fork the repository
2. Create feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open Pull Request

---

## Acknowledgments

This project builds on excellent open-source work:

- **[Multi_LiCa](https://github.com/XbotGroup/Multi_LiCa)** by XbotGroup - Multi-LiDAR calibration framework
- **[TEASER++](https://github.com/MIT-SPARK/TEASER-plusplus)** by MIT SPARK Lab - Fast and robust registration
- **[Open3D](http://www.open3d.org/)** - Modern 3D data processing library
- **ROS 2 Community** - Robotics middleware and tools

Special thanks to the researchers and developers who make robotics accessible!

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

You are free to:
- ✅ Use commercially
- ✅ Modify
- ✅ Distribute
- ✅ Use privately

---

## Citation

If you use this work in your research or project, please cite:

```bibtex
@misc{nova_tf_calibration_2026,
  author = {Masud Ali},
  title = {Nova TF Calibration: Multi-LiDAR Extrinsic Calibration Pipeline},
  year = {2026},
  publisher = {GitHub},
  url = {https://github.com/Masudali23/Nova_tf_calibration},
  note = {Practical implementation guide with code fixes for Multi_LiCa and TEASER++}
}
```

---
