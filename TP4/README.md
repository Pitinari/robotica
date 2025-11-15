# GTSAM Graph-SLAM Project

Implementation of 2D and 3D Pose Graph SLAM using the GTSAM (Georgia Tech Smoothing and Mapping) library. This project implements both **batch optimization** (Gauss-Newton) and **incremental optimization** (ISAM2) methods.

## Project Structure

```
TP4/
├── src/
│   ├── main.cpp                  # Main entry point
│   ├── g2o_reader.cpp            # G2O file parser
│   ├── graph_slam_2d.cpp         # 2D SLAM implementation
│   ├── graph_slam_3d.cpp         # 3D SLAM implementation
│   └── utils/
│       ├── visualization.cpp     # Visualization utilities
│       └── file_utils.cpp        # File I/O utilities
├── include/
│   ├── g2o_reader.h
│   ├── graph_slam_2d.h
│   ├── graph_slam_3d.h
│   └── utils/
│       ├── visualization.h
│       └── file_utils.h
├── data/
│   ├── input_INTEL_g2o.g2o      # 2D dataset (Intel Research Lab)
│   └── parking-garage.g2o        # 3D dataset (Parking Garage)
├── results/                      # Output directory for results
├── plot_2d_results.py           # Python script for 2D visualization
├── plot_3d_results.py           # Python script for 3D visualization
└── CMakeLists.txt               # CMake build configuration
```

## Prerequisites

### Required Software

- **CMake** (>= 3.10)
- **C++ Compiler** with C++14 support (GCC >= 4.7.3)
- **GTSAM** library (already built in `../gtsam`)
- **Python 3** with packages:
  - `matplotlib`
  - `pandas`
  - `numpy`

### Install Python Dependencies

```bash
pip install matplotlib pandas numpy
```

## Building the Project

The GTSAM library is already built in `/home/maxi/lcc/robotica/gtsam/build`. To build this project:

```bash
cd /home/maxi/lcc/robotica/TP4

# Create build directory
mkdir -p build
cd build

# Configure with CMake
cmake ..

# Build
make

# The executable will be: build/graph_slam
```

## Running the Algorithms

The program supports four modes of operation:

### Usage

```bash
./build/graph_slam <mode> <g2o_file> <output_prefix>
```

**Modes:**

- `2d-batch` - 2D batch optimization using Gauss-Newton
- `2d-incremental` - 2D incremental optimization using ISAM2
- `3d-batch` - 3D batch optimization using Gauss-Newton
- `3d-incremental` - 3D incremental optimization using ISAM2

### Task 2.2.B - 2D Batch Optimization

Optimize the Intel dataset using batch Gauss-Newton optimization:

```bash
./build/graph_slam 2d-batch input_INTEL_g2o.g2o results/2d_batch
```

**Output:**

- `results/2d_batch_initial.csv` - Unoptimized trajectory
- `results/2d_batch_optimized.csv` - Optimized trajectory

**Visualize:**

```bash
python3 plot_2d_results.py results/2d_batch_initial.csv results/2d_batch_optimized.csv results/2d_batch.png "2D Batch Optimization"
```

### Task 2.3.C - 2D Incremental Optimization

Optimize the Intel dataset using incremental ISAM2:

```bash
./build/graph_slam 2d-incremental input_INTEL_g2o.g2o results/2d_incremental
```

**Output:**

- `results/2d_incremental_initial.csv` - Unoptimized trajectory
- `results/2d_incremental_optimized.csv` - Optimized trajectory

**Visualize:**

```bash
python3 plot_2d_results.py results/2d_incremental_initial.csv results/2d_incremental_optimized.csv results/2d_incremental.png "2D Incremental Optimization (ISAM2)"
```

### Task 3.2.B - 3D Batch Optimization

Optimize the Parking Garage dataset using batch Gauss-Newton:

```bash
./build/graph_slam 3d-batch data/parking-garage.g2o results/3d_batch
```

**Output:**

- `results/3d_batch_initial.csv` - Unoptimized trajectory
- `results/3d_batch_optimized.csv` - Optimized trajectory

**Visualize:**

```bash
python3 plot_3d_results.py results/3d_batch_initial.csv results/3d_batch_optimized.csv results/3d_batch.png "3D Batch Optimization"
```

### Task 3.3.C - 3D Incremental Optimization

Optimize the Parking Garage dataset using incremental ISAM2:

```bash
./build/graph_slam 3d-incremental data/parking-garage.g2o results/3d_incremental
```

**Output:**

- `results/3d_incremental_initial.csv` - Unoptimized trajectory
- `results/3d_incremental_optimized.csv` - Optimized trajectory

**Visualize:**

```bash
python3 plot_3d_results.py results/3d_incremental_initial.csv results/3d_incremental_optimized.csv results/3d_incremental.png "3D Incremental Optimization (ISAM2)"
```

## Complete Workflow Example

Run all tasks and generate all visualizations:

```bash
#!/bin/bash
# Make sure we're in TP4 directory
cd /home/maxi/lcc/robotica/TP4

# Build the project
mkdir -p build
cd build
cmake ..
make
cd ..

# Create results directory
mkdir -p results

# Task 2.2.B - 2D Batch
echo "Running 2D Batch Optimization..."
./build/graph_slam 2d-batch input_INTEL_g2o.g2o results/2d_batch
python3 plot_2d_results.py results/2d_batch_initial.csv results/2d_batch_optimized.csv results/2d_batch.png "2D Batch Optimization (Gauss-Newton)"

# Task 2.3.C - 2D Incremental
echo "Running 2D Incremental Optimization..."
./build/graph_slam 2d-incremental input_INTEL_g2o.g2o results/2d_incremental
python3 plot_2d_results.py results/2d_incremental_initial.csv results/2d_incremental_optimized.csv results/2d_incremental.png "2D Incremental Optimization (ISAM2)"

# Task 3.2.B - 3D Batch (if parking-garage.g2o exists)
if [ -f "data/parking-garage.g2o" ]; then
    echo "Running 3D Batch Optimization..."
    ./build/graph_slam 3d-batch data/parking-garage.g2o results/3d_batch
    python3 plot_3d_results.py results/3d_batch_initial.csv results/3d_batch_optimized.csv results/3d_batch.png "3D Batch Optimization"
fi

# Task 3.3.C - 3D Incremental (if parking-garage.g2o exists)
if [ -f "data/parking-garage.g2o" ]; then
    echo "Running 3D Incremental Optimization..."
    ./build/graph_slam 3d-incremental data/parking-garage.g2o results/3d_incremental
    python3 plot_3d_results.py results/3d_incremental_initial.csv results/3d_incremental_optimized.csv results/3d_incremental.png "3D Incremental Optimization (ISAM2)"
fi

echo "All tasks completed! Check the results/ directory for outputs."
```

Save this as `run_all.sh` and execute:

```bash
chmod +x run_all.sh
./run_all.sh
```

## Output Files

### CSV Format

**2D Trajectory CSV:**

```csv
id,x,y,theta
0,0.0,0.0,0.0
1,0.5,0.1,0.05
...
```

**3D Trajectory CSV:**

```csv
id,x,y,z,qx,qy,qz,qw
0,0.0,0.0,0.0,0.0,0.0,0.0,1.0
1,1.0,0.1,0.0,0.0,0.0,0.05,0.998
...
```

### Visualization

The Python scripts generate publication-quality plots showing:

- **Red line**: Unoptimized trajectory (from G2O file initial estimates)
- **Blue line**: Optimized trajectory (after SLAM optimization)
- **Green circle**: Start position
- **Purple square**: End position

## Implementation Details

### G2O File Format

**2D Format (SE2):**

- `VERTEX_SE2 id x y theta` - Pose vertices
- `EDGE_SE2 id1 id2 dx dy dtheta info(0) info(1) info(2) info(3) info(4) info(5)` - Edges

**3D Format (SE3:QUAT):**

- `VERTEX_SE3:QUAT id x y z qx qy qz qw` - Pose vertices
- `EDGE_SE3:QUAT id1 id2 dx dy dz qx qy qz qw info(0) ... info(20)` - Edges

### Information Matrix

The information matrix in G2O format is stored as the upper triangular part:

- **2D**: 6 values representing 3×3 symmetric matrix
- **3D**: 21 values representing 6×6 symmetric matrix

The code converts this to a full symmetric matrix for GTSAM's noise model.

### Optimization Methods

**Batch Optimization (Gauss-Newton):**

- Builds complete factor graph with all poses and constraints
- Optimizes all variables simultaneously
- Fast for small to medium problems
- No incremental updates

**Incremental Optimization (ISAM2):**

- Processes poses one at a time (Algorithm 1 from assignment)
- Updates only affected parts of the factor graph
- Uses Bayes tree for efficient inference
- Better for large-scale or online SLAM

## Troubleshooting

### GTSAM Not Found

If CMake cannot find GTSAM, verify the path:

```bash
ls ../gtsam/build/GTSAMConfig.cmake
```

If needed, set the path explicitly:

```bash
cmake -DGTSAM_DIR=/home/maxi/lcc/robotica/gtsam/build ..
```

### Python Import Errors

Install missing packages:

```bash
pip install --user matplotlib pandas numpy
```

### Build Errors

Clean and rebuild:

```bash
cd build
rm -rf *
cmake ..
make
```

## References

- GTSAM Documentation: https://gtsam.org/
- G2O File Format: https://github.com/RainerKuemmerle/g2o/wiki/File-Format-SLAM-2D
- Assignment PDF: `tp_graph_slam.pdf`

## Authors

- Implementation by: [Your Name]
- Course: Robótica Móvil - R-521
- Institution: Universidad Nacional de Córdoba
