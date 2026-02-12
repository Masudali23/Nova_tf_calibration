#!/usr/bin/env python3
"""
Multi-LiDAR PCD Recording Script
Records point clouds from multiple LiDAR topics simultaneously.
Configuration-driven: just add LiDARs to YAML config file.

NO Open3D required - uses only numpy and ROS2.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import struct
import yaml
import os
import sys
from datetime import datetime
from pathlib import Path


class PCDWriter:
    """Simple PCD file writer (ASCII format)."""
    
    @staticmethod
    def write_pcd(filename, points):
        """
        Write points to PCD file in ASCII format.
        
        Args:
            filename: Output file path
            points: Nx3 numpy array of XYZ points
        """
        if len(points) == 0:
            print(f"   ⚠️  Warning: Empty point cloud, skipping {filename}")
            return False
            
        with open(filename, 'w') as f:
            # PCD header
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")
            f.write(f"WIDTH {len(points)}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(points)}\n")
            f.write("DATA ascii\n")
            
            # Write points
            for p in points:
                f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
        
        return True


class PointCloudExtractor:
    """Extract XYZ points from PointCloud2 messages."""
    
    @staticmethod
    def extract_xyz(msg, min_range=0.2, max_range=25.0):
        """
        Extract XYZ points from PointCloud2 message.
        
        Args:
            msg: sensor_msgs/PointCloud2 message
            min_range: Minimum distance filter (meters)
            max_range: Maximum distance filter (meters)
            
        Returns:
            Nx3 numpy array of XYZ points
        """
        points = []
        
        # Find field offsets
        x_off = y_off = z_off = None
        for field in msg.fields:
            if field.name == 'x':
                x_off = field.offset
            elif field.name == 'y':
                y_off = field.offset
            elif field.name == 'z':
                z_off = field.offset
        
        if x_off is None:
            return np.array([])
        
        # Parse binary data
        data = bytes(msg.data)
        step = msg.point_step
        
        for i in range(0, len(data), step):
            try:
                x = struct.unpack('f', data[i+x_off:i+x_off+4])[0]
                y = struct.unpack('f', data[i+y_off:i+y_off+4])[0]
                z = struct.unpack('f', data[i+z_off:i+z_off+4])[0]
                
                # Filter invalid and out-of-range points
                if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                    dist = np.sqrt(x*x + y*y + z*z)
                    if min_range < dist < max_range:
                        points.append([x, y, z])
            except:
                continue
        
        return np.array(points, dtype=np.float32)


class LidarRecorder:
    """Recorder for a single LiDAR."""
    
    def __init__(self, name, topic, output_dir, max_frames, min_points, voxel_size):
        self.name = name
        self.topic = topic
        self.output_dir = output_dir
        self.max_frames = max_frames
        self.min_points = min_points
        self.voxel_size = voxel_size
        self.count = 0
        self.subscription = None
        self.node = None
        
    def is_complete(self):
        """Check if recording is complete."""
        return self.count >= self.max_frames
    
    def callback(self, msg):
        """Handle incoming PointCloud2 message."""
        if self.is_complete():
            return
        
        # Extract points
        points = PointCloudExtractor.extract_xyz(msg)
        
        if len(points) < self.min_points:
            self.node.get_logger().warn(
                f"{self.name}: Only {len(points)} points, skipping (min: {self.min_points})"
            )
            return
        
        # Optional voxel downsampling
        if self.voxel_size > 0:
            points = self.voxel_downsample(points, self.voxel_size)
        
        # Save to PCD file
        filename = f"{self.name}_{self.count:03d}.pcd"
        filepath = os.path.join(self.output_dir, filename)
        
        if PCDWriter.write_pcd(filepath, points):
            self.node.get_logger().info(
                f"✅ {self.name}: Saved {filename} ({len(points)} points) [{self.count+1}/{self.max_frames}]"
            )
            self.count += 1
        
        # Check if all frames collected
        if self.is_complete():
            self.node.get_logger().info(f"🎉 {self.name}: Recording complete!")
    
    def voxel_downsample(self, points, voxel_size):
        """Simple voxel grid downsampling."""
        if len(points) == 0:
            return points
        
        voxel_indices = np.floor(points / voxel_size).astype(np.int32)
        
        # Use dictionary to group points by voxel
        voxel_dict = {}
        for i, voxel in enumerate(voxel_indices):
            key = tuple(voxel)
            if key not in voxel_dict:
                voxel_dict[key] = [points[i]]
            else:
                voxel_dict[key].append(points[i])
        
        # Return centroids
        result = np.array([np.mean(pts, axis=0) for pts in voxel_dict.values()])
        return result


class MultiLidarPCDRecorder(Node):
    """
    Multi-LiDAR PCD recording node.
    Reads configuration and records from multiple LiDARs simultaneously.
    """
    
    def __init__(self, config_path):
        super().__init__('multi_lidar_pcd_recorder')
        
        # Load configuration
        self.config = self.load_config(config_path)
        
        # Create output directory
        output_dir = os.path.expanduser(self.config['recording']['output_directory'])
        os.makedirs(output_dir, exist_ok=True)
        self.get_logger().info(f"📁 Output directory: {output_dir}")
        
        # Initialize recorders for each LiDAR
        self.recorders = []
        frames_per_lidar = self.config['recording']['frames_per_lidar']
        min_points = self.config['recording']['filtering']['min_points']
        voxel_size = self.config['recording']['voxel_size']
        
        for lidar_config in self.config['recording']['lidars']:
            if not lidar_config.get('enabled', True):
                self.get_logger().info(f"⏭️  Skipping disabled LiDAR: {lidar_config['name']}")
                continue
            
            recorder = LidarRecorder(
                name=lidar_config['name'],
                topic=lidar_config['topic'],
                output_dir=output_dir,
                max_frames=frames_per_lidar,
                min_points=min_points,
                voxel_size=voxel_size
            )
            recorder.node = self  # Give recorder access to node logger
            
            # Create subscription
            recorder.subscription = self.create_subscription(
                PointCloud2,
                lidar_config['topic'],
                recorder.callback,
                10
            )
            
            self.recorders.append(recorder)
            self.get_logger().info(
                f"🎤 Recording {lidar_config['name']}: {lidar_config['topic']} "
                f"({frames_per_lidar} frames)"
            )
        
        if not self.recorders:
            self.get_logger().error("❌ No LiDARs enabled in config!")
            sys.exit(1)
        
        # Create timer to check completion
        self.timer = self.create_timer(1.0, self.check_completion)
        
        # Timeout timer
        timeout = self.config['recording']['timeout_seconds']
        self.timeout_timer = self.create_timer(timeout, self.timeout_callback)
        
        self.get_logger().info(f"⏱️  Timeout: {timeout} seconds")
        self.get_logger().info("="*70)
        self.get_logger().info("🚀 Recording started! Keep robot stationary...")
        self.get_logger().info("="*70)
    
    def load_config(self, config_path):
        """Load YAML configuration file."""
        config_path = os.path.expanduser(config_path)
        
        if not os.path.exists(config_path):
            self.get_logger().error(f"❌ Config file not found: {config_path}")
            sys.exit(1)
        
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        self.get_logger().info(f"✅ Loaded config: {config_path}")
        return config
    
    def check_completion(self):
        """Check if all recorders are complete."""
        all_complete = all(recorder.is_complete() for recorder in self.recorders)
        
        if all_complete:
            self.get_logger().info("="*70)
            self.get_logger().info("🎉 ALL LIDARS RECORDING COMPLETE!")
            self.get_logger().info("="*70)
            
            # Print summary
            for recorder in self.recorders:
                self.get_logger().info(
                    f"   {recorder.name}: {recorder.count}/{recorder.max_frames} frames"
                )
            
            self.get_logger().info("="*70)
            rclpy.shutdown()
    
    def timeout_callback(self):
        """Handle timeout."""
        self.get_logger().warn("⏱️  Timeout reached!")
        
        # Check which recorders didn't complete
        incomplete = [r for r in self.recorders if not r.is_complete()]
        
        if incomplete:
            self.get_logger().warn("⚠️  Some LiDARs did not complete:")
            for recorder in incomplete:
                self.get_logger().warn(
                    f"   {recorder.name}: {recorder.count}/{recorder.max_frames} frames"
                )
        
        rclpy.shutdown()


def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Record PCD files from multiple LiDARs using config file'
    )
    parser.add_argument(
        '--config',
        type=str,
        default='config/pcd_recording_config.yaml',
        help='Path to YAML configuration file'
    )
    args = parser.parse_args()
    
    print("="*70)
    print("Multi-LiDAR PCD Recorder")
    print("="*70)
    
    rclpy.init()
    
    try:
        recorder = MultiLidarPCDRecorder(args.config)
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()