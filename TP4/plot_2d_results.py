#!/usr/bin/env python3
"""
Script to visualize 2D SLAM results from CSV files.
Generates plots comparing unoptimized vs optimized trajectories.
"""

import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

def plot_2d_trajectory(initial_csv, optimized_csv, output_file=None, title="2D SLAM Trajectory"):
    """Plot 2D trajectory comparison."""
    # Read CSV files
    initial = pd.read_csv(initial_csv)
    optimized = pd.read_csv(optimized_csv)
    
    # Create figure
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Plot unoptimized trajectory
    ax.plot(initial['x'], initial['y'], 'r-', alpha=0.6, linewidth=1.5, 
            label='Unoptimized Trajectory')
    ax.scatter(initial['x'], initial['y'], c='red', s=10, alpha=0.3)
    
    # Plot optimized trajectory
    ax.plot(optimized['x'], optimized['y'], 'b-', linewidth=2, 
            label='Optimized Trajectory')
    ax.scatter(optimized['x'], optimized['y'], c='blue', s=10, alpha=0.5)
    
    # Mark start and end
    ax.scatter(initial['x'].iloc[0], initial['y'].iloc[0], 
              c='green', s=200, marker='o', edgecolors='black', linewidth=2,
              label='Start', zorder=10)
    ax.scatter(optimized['x'].iloc[-1], optimized['y'].iloc[-1], 
              c='purple', s=200, marker='s', edgecolors='black', linewidth=2,
              label='End', zorder=10)
    
    # Configure plot
    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    plt.tight_layout()
    
    # Save or show
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Saved plot to {output_file}")
    else:
        plt.show()
    
    plt.close()

def main():
    if len(sys.argv) < 3:
        print("Usage: python plot_2d_results.py <initial_csv> <optimized_csv> [output_image] [title]")
        print("\nExample:")
        print("  python plot_2d_results.py results/2d_batch_initial.csv results/2d_batch_optimized.csv results/2d_batch.png")
        sys.exit(1)
    
    initial_csv = sys.argv[1]
    optimized_csv = sys.argv[2]
    output_file = sys.argv[3] if len(sys.argv) > 3 else None
    title = sys.argv[4] if len(sys.argv) > 4 else "2D SLAM Trajectory"
    
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
    
    # Generate plot
    plot_2d_trajectory(initial_csv, optimized_csv, output_file, title)

if __name__ == "__main__":
    main()

