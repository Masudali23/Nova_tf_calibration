#!/usr/bin/env python3
"""
Nova TF Calibration - Results Visualization Script
Visualizes calibration results using Open3D
"""

import open3d as o3d
import numpy as np
import os
import sys
from pathlib import Path


def find_calibration_output():
    """Find calibration output directory."""
    # Try common locations
    possible_dirs = [
        os.path.expanduser("~/calibration_output"),
        os.path.expanduser("~/drobot_lidar/calibration_output"),
        "./calibration_output",
        "../calibration_output",
    ]
    
    for dir_path in possible_dirs:
        if os.path.exists(dir_path):
            return dir_path
    
    return None


def load_point_cloud(filepath):
    """Load point cloud from PCD file."""
    if not os.path.exists(filepath):
        print(f"❌ File not found: {filepath}")
        return None
    
    try:
        pcd = o3d.io.read_point_cloud(filepath)
        print(f"✅ Loaded {len(pcd.points)} points from {os.path.basename(filepath)}")
        return pcd
    except Exception as e:
        print(f"❌ Error loading {filepath}: {e}")
        return None


def visualize_comparison(before_pcd, after_pcd):
    """Visualize before and after calibration side by side."""
    if before_pcd is None or after_pcd is None:
        print("❌ Cannot visualize: missing point clouds")
        return
    
    # Color them differently
    before_pcd.paint_uniform_color([1, 0, 0])  # Red = before
    after_pcd.paint_uniform_color([0, 1, 0])   # Green = after
    
    # Offset before cloud to the side
    before_pcd.translate([3, 0, 0])
    
    print("\n" + "="*70)
    print("Visualization Controls:")
    print("="*70)
    print("  Mouse Left + Drag:   Rotate view")
    print("  Mouse Right + Drag:  Translate view")
    print("  Mouse Wheel:         Zoom")
    print("  H:                   Show help")
    print("  Q / ESC:             Close window")
    print("="*70)
    print("\nShowing comparison:")
    print("  RED (left)   = Before calibration")
    print("  GREEN (right) = After calibration")
    print("\nPress Q to close...")
    
    o3d.visualization.draw_geometries(
        [before_pcd, after_pcd],
        window_name="Calibration Comparison: Before (Red) vs After (Green)",
        width=1280,
        height=720
    )


def visualize_single(pcd, title="Point Cloud"):
    """Visualize a single point cloud."""
    if pcd is None:
        print("❌ Cannot visualize: missing point cloud")
        return
    
    # Downsample for better performance
    if len(pcd.points) > 50000:
        print(f"Downsampling from {len(pcd.points)} points...")
        pcd = pcd.voxel_down_sample(voxel_size=0.05)
        print(f"Downsampled to {len(pcd.points)} points")
    
    # Estimate normals for better visualization
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
    )
    
    # Create coordinate frame
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    
    print(f"\nShowing: {title}")
    print("Press Q to close...")
    
    o3d.visualization.draw_geometries(
        [pcd, coord_frame],
        window_name=title,
        width=1280,
        height=720
    )


def print_results(output_dir):
    """Print calibration results from results.txt."""
    results_file = os.path.join(output_dir, "results.txt")
    
    if not os.path.exists(results_file):
        print("⚠️  Results file not found")
        return
    
    print("\n" + "="*70)
    print("CALIBRATION RESULTS")
    print("="*70)
    
    with open(results_file, 'r') as f:
        print(f.read())
    
    print("="*70)


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Visualize calibration results')
    parser.add_argument(
        '--output-dir',
        type=str,
        help='Path to calibration output directory'
    )
    parser.add_argument(
        '--mode',
        type=str,
        choices=['comparison', 'before', 'after'],
        default='comparison',
        help='Visualization mode'
    )
    args = parser.parse_args()
    
    print("="*70)
    print("Nova TF Calibration - Results Visualization")
    print("="*70)
    
    # Find output directory
    if args.output_dir:
        output_dir = os.path.expanduser(args.output_dir)
    else:
        output_dir = find_calibration_output()
    
    if output_dir is None or not os.path.exists(output_dir):
        print("❌ Could not find calibration output directory")
        print("\nPlease specify with --output-dir, or run calibration first")
        sys.exit(1)
    
    print(f"\n📁 Output directory: {output_dir}")
    
    # Print results
    print_results(output_dir)
    
    # Load point clouds
    before_path = os.path.join(output_dir, "stitched_initial.pcd")
    after_path = os.path.join(output_dir, "stitched_transformed.pcd")
    
    if args.mode == 'comparison':
        before = load_point_cloud(before_path)
        after = load_point_cloud(after_path)
        visualize_comparison(before, after)
    
    elif args.mode == 'before':
        before = load_point_cloud(before_path)
        visualize_single(before, "Before Calibration")
    
    elif args.mode == 'after':
        after = load_point_cloud(after_path)
        visualize_single(after, "After Calibration")
    
    print("\n✅ Visualization complete")


if __name__ == '__main__':
    main()