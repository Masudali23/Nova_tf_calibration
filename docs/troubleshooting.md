# Troubleshooting Guide

Common issues and solutions for Nova TF Calibration.

## Table of Contents

- [Installation Issues](#installation-issues)
- [Recording Issues](#recording-issues)
- [Calibration Issues](#calibration-issues)
- [Visualization Issues](#visualization-issues)
- [General Issues](#general-issues)
- [Getting Help](#getting-help)
- [Quick Checklist](#quick-checklist)

---

## Installation Issues

### TEASER++ Compilation Fails

**Symptom:**
```
error: 'vector' does not name a type
```

**Cause:** Missing `#include <vector>` in TEASER++ source files

**Solution:**
```bash
cd ~/drobot_lidar/src/TEASER-plusplus
sed -i 's/\bvector</std::vector</g' teaser/src/graph.cc
sed -i 's/\bvector</std::vector</g' teaser/include/teaser/graph.h
cd build
make clean
make -j$(nproc)
sudo make install
```

### Python Bindings Not Found

**Symptom:**
```
ModuleNotFoundError: No module named 'teaserpp_python'
```

**Cause:** Python bindings not installed or wrong Python version

**Solution:**
```bash
cd ~/drobot_lidar/src/TEASER-plusplus/build/python
pip3 install . --user --force-reinstall

# Verify
python3 -c "import teaserpp_python"
```

---

## Recording Issues

### No LiDAR Topics Found

**Symptom:**
```
❌ /pointcloud_front (NOT FOUND)
```

**Cause:** LiDAR drivers not running or topic names incorrect

**Solution:**

1. Check if LiDAR drivers are running:
```bash
ros2 topic list
```

2. Find correct topic names:
```bash
ros2 topic list | grep -i point
```

3. Update `config/pcd_recording_config.yaml` with correct topics

### Empty or Small Point Clouds

**Symptom:**
```
⚠️ Only 50 points, skipping (min: 100)
```

**Cause:** LiDAR obstructed, wrong filtering parameters, or sensor issue

**Solution:**

1. Check LiDAR has clear view (not pointing at ground/sky)

2. Verify sensor is working: 
```bash
ros2 topic echo /your_lidar_topic
```

3. Adjust filtering in config:
```yaml
filtering:
  min_range: 0.2  # Reduce if indoor
  max_range: 30.0  # Increase if needed
  min_points: 50   # Lower threshold
```

### Recording Timeout

**Symptom:**
```
⏱️ Timeout reached!
⚠️ Some LiDARs did not complete
```

**Cause:** Topics not publishing fast enough or incorrect topic names

**Solution:**

1. Increase timeout in config:
```yaml
timeout_seconds: 60  # Increase from 30
```

2. Check topic publishing rate:
```bash
ros2 topic hz /pointcloud_front
```

3. Reduce frames_per_lidar if needed:
```yaml
frames_per_lidar: 5  # Reduce from 10
```

---

## Calibration Issues

### No LiDARs Detected

**Symptom:**
```
[INFO] Detected lidars: []
KeyError: 'front'
```

**Cause:** PCD files not found or incorrect naming

**Solution:**

1. Verify files exist:
```bash
ls ~/calibration_data/*.pcd
```

2. Check filename format (should be `front_000.pcd`, `back_000.pcd`, etc.)

3. Use absolute path in config:
```yaml
pcd_directory: "/home/username/calibration_data/"  # Not ~/
```

4. Verify permissions:
```bash
chmod +r ~/calibration_data/*.pcd
```

### Low Fitness Score

**Symptom:**
```
fitness: 0.05, inlier_rmse: 0.8
Calibration NOT SUCCESSFUL!
```

**Cause:** Poor overlap between LiDARs or featureless environment

**Solution:**

1. **Increase overlap:** Position robot so LiDARs see more common area

2. **Improve environment:** Move to area with:
   - ✅ Furniture, walls, corners
   - ✅ Varied geometry
   - ❌ Avoid blank walls, empty rooms

3. Adjust parameters in `config/multi_lica_config.yaml`:
```yaml
max_corresp_dist: 1.0  # Increase from 0.5
voxel_size: 0.03       # Decrease from 0.05 (denser)
fitness_score_threshold: 0.1  # Lower threshold
```

4. Try fitness-based calibration:
```yaml
use_fitness_based_calibration: true
```

### Permission Denied on Output

**Symptom:**
```
PermissionError: [Errno 13] Permission denied: '/../output/'
```

**Cause:** Invalid output directory or permission issue

**Solution:**

1. Use absolute path:
```yaml
output_dir: "/home/username/calibration_output/"
```

2. Create directory manually:
```bash
mkdir -p ~/calibration_output
chmod 755 ~/calibration_output
```

### Calibration Takes Too Long

**Symptom:** Calibration runs for >10 minutes

**Cause:** Too many points or suboptimal parameters

**Solution:**

1. Reduce point cloud density:
```yaml
voxel_size: 0.1  # Increase from 0.05
```

2. Reduce frames:
```yaml
frame_count: 5  # Reduce from 10
```

3. Reduce iterations:
```yaml
max_iterations: 50  # Reduce from 100
num_iterations: 1000  # Reduce from 2000
```

---

## Visualization Issues

### Open3D Window Black/Empty

**Symptom:** Visualization window opens but shows nothing

**Cause:** Display configuration issue or empty point cloud

**Solution:**

1. Check file size:
```bash
ls -lh ~/calibration_output/stitched_transformed.pcd
```
   Should be >1MB

2. Try headless mode:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
python3 scripts/visualize_results.py
```

3. Disable visualization in config:
```yaml
visualize: false
```

### GLEW Initialization Failed

**Symptom:**
```
[Open3D WARNING] Failed to initialize GLEW
```

**Cause:** Graphics driver issue or running over SSH without X11 forwarding

**Solution:**

1. **If local:** Update graphics drivers

2. **If remote (SSH):**
```bash
export DISPLAY=:0
# Or use software rendering
export LIBGL_ALWAYS_SOFTWARE=1
```

3. **Alternative:** Disable visualization and view files separately:
```bash
# Copy to local machine and view with CloudCompare or MeshLab
scp user@robot:~/calibration_output/*.pcd .
```

---

## General Issues

### Import Error: ros2_numpy

**Symptom:**
```
ModuleNotFoundError: No module named 'ros2_numpy'
```

**Solution:**
```bash
pip3 install --user ros2-numpy
```

### Workspace Not Found

**Symptom:**
```
source: ~/drobot_lidar/install/setup.bash: No such file or directory
```

**Solution:**

1. Build workspace first:
```bash
cd ~/drobot_lidar
colcon build --packages-select multi_lidar_calibrator
```

2. Or adjust path if workspace is elsewhere:
```bash
export ROS_WORKSPACE=/path/to/your/workspace
source $ROS_WORKSPACE/install/setup.bash
```

### Calibration Results Look Wrong

**Symptom:** Large unexpected translation/rotation

**Possible Causes:**
- Wrong initial guesses - Update config with better estimates
- LiDARs moved during recording - Repeat data collection
- Poor environment - Record in more feature-rich area
- Wrong target frame - Check `target_frame_id` in config

**Verification Steps:**

1. Visualize results:
```bash
python3 scripts/visualize_results.py
```

2. Check fitness score (should be >0.2 for partial overlap)

3. Verify point clouds align in visualization

4. Compare before/after transformations in `results.txt`

---

## Getting Help

If your issue isn't listed here:

### Check Logs
```bash
tail -f ~/.ros/log/latest/multi_lidar_calibrator-1.log
```

### Enable Debug Mode
Add to launch file:
```python
arguments=['--ros-args', '--log-level', 'DEBUG']
```

### Open GitHub Issue
Include:
- Error messages
- Share config files
- Describe your setup (robot model, LiDAR types)

### Community Support
- GitHub Discussions
- ROS Answers (tag: multi-lidar-calibration)

---

## Quick Checklist

Before asking for help, verify:

- ☑️ ROS 2 Humble installed and sourced
- ☑️ TEASER++ Python bindings installed (`python3 -c "import teaserpp_python"`)
- ☑️ PCD files exist and are >100KB each
- ☑️ Config file uses absolute paths
- ☑️ LiDARs have overlapping field of view
- ☑️ Recorded data in feature-rich environment
- ☑️ Robot was stationary during recording
- ☑️ All patches applied to Multi_LiCa