#!/usr/bin/env python3
"""
Script to visualize 3D SLAM results from CSV files.
Generates 3D plots comparing unoptimized vs optimized trajectories.
"""

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import os

def plot_3d_trajectory(initial_csv, optimized_csv, output_file=None, title="3D SLAM Trajectory"):
    """Plot 3D trajectory comparison."""
    # Read CSV files
    initial = pd.read_csv(initial_csv)
    optimized = pd.read_csv(optimized_csv)
    
    # Create figure with 3D axis
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot unoptimized trajectory
    ax.plot(initial['x'], initial['y'], initial['z'], 
            'r-', alpha=0.6, linewidth=1.5, label='Unoptimized Trajectory')
    ax.scatter(initial['x'], initial['y'], initial['z'], 
               c='red', s=10, alpha=0.3)
    
    # Plot optimized trajectory
    ax.plot(optimized['x'], optimized['y'], optimized['z'], 
            'b-', linewidth=2, label='Optimized Trajectory')
    ax.scatter(optimized['x'], optimized['y'], optimized['z'], 
               c='blue', s=10, alpha=0.5)
    
    # Mark start and end
    ax.scatter(initial['x'].iloc[0], initial['y'].iloc[0], initial['z'].iloc[0],
              c='green', s=200, marker='o', edgecolors='black', linewidth=2,
              label='Start', zorder=10)
    ax.scatter(optimized['x'].iloc[-1], optimized['y'].iloc[-1], optimized['z'].iloc[-1],
              c='purple', s=200, marker='s', edgecolors='black', linewidth=2,
              label='End', zorder=10)
    
    # Configure plot
    ax.set_xlabel('X Position (m)', fontsize=11)
    ax.set_ylabel('Y Position (m)', fontsize=11)
    ax.set_zlabel('Z Position (m)', fontsize=11)
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=9)
    ax.grid(True, alpha=0.3)
    
    # Set equal aspect ratio (approximately)
    max_range = max(
        initial['x'].max() - initial['x'].min(),
        initial['y'].max() - initial['y'].min(),
        initial['z'].max() - initial['z'].min()
    ) / 2.0
    
    mid_x = (initial['x'].max() + initial['x'].min()) / 2.0
    mid_y = (initial['y'].max() + initial['y'].min()) / 2.0
    mid_z = (initial['z'].max() + initial['z'].min()) / 2.0
    
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    plt.tight_layout()
    
    # Save or show
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Saved plot to {output_file}")
    else:
        plt.show()
    
    plt.close()

def plot_3d_trajectory_2d_projections(initial_csv, optimized_csv, output_file=None):
    """Plot 3D trajectory as 2D projections (XY, XZ, YZ)."""
    # Read CSV files
    initial = pd.read_csv(initial_csv)
    optimized = pd.read_csv(optimized_csv)
    
    # Create figure with subplots
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    
    # XY projection
    axes[0].plot(initial['x'], initial['y'], 'r-', alpha=0.6, linewidth=1.5, label='Unoptimized')
    axes[0].plot(optimized['x'], optimized['y'], 'b-', linewidth=2, label='Optimized')
    axes[0].set_xlabel('X Position (m)')
    axes[0].set_ylabel('Y Position (m)')
    axes[0].set_title('XY Projection (Top View)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[0].axis('equal')
    
    # XZ projection
    axes[1].plot(initial['x'], initial['z'], 'r-', alpha=0.6, linewidth=1.5, label='Unoptimized')
    axes[1].plot(optimized['x'], optimized['z'], 'b-', linewidth=2, label='Optimized')
    axes[1].set_xlabel('X Position (m)')
    axes[1].set_ylabel('Z Position (m)')
    axes[1].set_title('XZ Projection (Side View)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    axes[1].axis('equal')
    
    # YZ projection
    axes[2].plot(initial['y'], initial['z'], 'r-', alpha=0.6, linewidth=1.5, label='Unoptimized')
    axes[2].plot(optimized['y'], optimized['z'], 'b-', linewidth=2, label='Optimized')
    axes[2].set_xlabel('Y Position (m)')
    axes[2].set_ylabel('Z Position (m)')
    axes[2].set_title('YZ Projection (Front View)')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    axes[2].axis('equal')
    
    plt.tight_layout()
    
    # Save or show
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Saved 2D projections to {output_file}")
    else:
        plt.show()
    
    plt.close()

def main():
    if len(sys.argv) < 3:
        print("Usage: python plot_3d_results.py <initial_csv> <optimized_csv> [output_image] [title]")
        print("\nExample:")
        print("  python plot_3d_results.py results/3d_batch_initial.csv results/3d_batch_optimized.csv results/3d_batch.png")
        sys.exit(1)
    
    initial_csv = sys.argv[1]
    optimized_csv = sys.argv[2]
    output_file = sys.argv[3] if len(sys.argv) > 3 else None
    title = sys.argv[4] if len(sys.argv) > 4 else "3D SLAM Trajectory"
    
    # Check if files exist
    if not os.path.exists(initial_csv):
        print(f"Error: File not found: {initial_csv}")
        sys.exit(1)
    if not os.path.exists(optimized_csv):
        print(f"Error: File not found: {optimized_csv}")
        sys.exit(1)
    
    # Create output directory if needed
    if output_file:
        output_dir = os.path.dirname(output_file)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)
    
    # Generate 3D plot
    plot_3d_trajectory(initial_csv, optimized_csv, output_file, title)
    
    # Also generate 2D projections
    if output_file:
        base_name = output_file.rsplit('.', 1)[0]
        projection_file = f"{base_name}_projections.png"
        plot_3d_trajectory_2d_projections(initial_csv, optimized_csv, projection_file)

if __name__ == "__main__":
    main()

