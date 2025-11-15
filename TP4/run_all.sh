#!/bin/bash
# Complete workflow to run all SLAM tasks and generate visualizations

set -e  # Exit on error

echo "=========================================="
echo "GTSAM Graph-SLAM - Complete Workflow"
echo "=========================================="
echo ""

# Make sure we're in TP4 directory
cd "$(dirname "$0")"

# Build the project
echo "Step 1: Building the project..."
mkdir -p build
cd build
cmake .. || { echo "CMake failed!"; exit 1; }
make || { echo "Build failed!"; exit 1; }
cd ..
echo "✓ Build successful!"
echo ""

# Create results directory
mkdir -p results
echo "✓ Results directory created"
echo ""

# Task 2.2.B - 2D Batch
echo "=========================================="
echo "Task 2.2.B: 2D Batch Optimization"
echo "=========================================="
./build/graph_slam 2d-batch input_INTEL_g2o.g2o results/2d_batch
python3 plot_2d_results.py results/2d_batch_initial.csv results/2d_batch_optimized.csv results/2d_batch.png "2D Batch Optimization (Gauss-Newton)"
echo "✓ Task 2.2.B complete!"
echo ""

# Task 2.3.C - 2D Incremental
echo "=========================================="
echo "Task 2.3.C: 2D Incremental Optimization"
echo "=========================================="
./build/graph_slam 2d-incremental input_INTEL_g2o.g2o results/2d_incremental
python3 plot_2d_results.py results/2d_incremental_initial.csv results/2d_incremental_optimized.csv results/2d_incremental.png "2D Incremental Optimization (ISAM2)"
echo "✓ Task 2.3.C complete!"
echo ""

# Task 3.2.B - 3D Batch
if [ -f "data/parking-garage.g2o" ] && [ -s "data/parking-garage.g2o" ]; then
    echo "=========================================="
    echo "Task 3.2.B: 3D Batch Optimization"
    echo "=========================================="
    ./build/graph_slam 3d-batch data/parking-garage.g2o results/3d_batch
    python3 plot_3d_results.py results/3d_batch_initial.csv results/3d_batch_optimized.csv results/3d_batch.png "3D Batch Optimization"
    echo "✓ Task 3.2.B complete!"
    echo ""
else
    echo "⚠ Skipping 3D tasks: data/parking-garage.g2o not found or empty"
    echo "  Download from: https://www.dropbox.com/s/mzj3pdb27z2j6s/parking-garage.g2o"
    echo ""
fi

# Task 3.3.C - 3D Incremental
if [ -f "data/parking-garage.g2o" ] && [ -s "data/parking-garage.g2o" ]; then
    echo "=========================================="
    echo "Task 3.3.C: 3D Incremental Optimization"
    echo "=========================================="
    ./build/graph_slam 3d-incremental data/parking-garage.g2o results/3d_incremental
    python3 plot_3d_results.py results/3d_incremental_initial.csv results/3d_incremental_optimized.csv results/3d_incremental.png "3D Incremental Optimization (ISAM2)"
    echo "✓ Task 3.3.C complete!"
    echo ""
fi

echo "=========================================="
echo "All tasks completed successfully!"
echo "=========================================="
echo ""
echo "Generated files in results/:"
ls -lh results/
echo ""
echo "View the PNG images to see the optimized trajectories."

