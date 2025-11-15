# Quick Start Guide

## What's Been Implemented

This project implements 2D and 3D Pose Graph SLAM using GTSAM with both **batch** (Levenberg-Marquardt) and **incremental** (ISAM2) optimization methods.

### ✅ Completed Features

1. **G2O File Readers** - Parse 2D (VERTEX_SE2/EDGE_SE2) and 3D (VERTEX_SE3:QUAT/EDGE_SE3:QUAT) formats
2. **2D Batch Optimization** - Using Levenberg-Marquardt optimizer
3. **2D Incremental Optimization** - Using ISAM2 algorithm
4. **3D Batch Optimization** - Using Levenberg-Marquardt optimizer
5. **3D Incremental Optimization** - Using ISAM2 algorithm
6. **Python Visualization Scripts** - Generate comparison plots
7. **Automated Build System** - CMake configuration for GTSAM integration

## Running the Examples

### Build the Project

```bash
cd /home/maxi/lcc/robotica/TP4
mkdir -p build results
cd build
cmake ..
make
cd ..
```

### Task 2.2.B - 2D Batch Optimization

```bash
./build/graph_slam 2d-batch input_INTEL_g2o.g2o results/2d_batch
python3 plot_2d_results.py results/2d_batch_initial.csv results/2d_batch_optimized.csv results/2d_batch.png "2D Batch Optimization"
```

**Results:**

- Initial error: 2,574,860
- Final error: 385,129 (85% reduction)
- Output: `results/2d_batch.png`

### Task 2.3.C - 2D Incremental Optimization

```bash
./build/graph_slam 2d-incremental input_INTEL_g2o.g2o results/2d_incremental
python3 plot_2d_results.py results/2d_incremental_initial.csv results/2d_incremental_optimized.csv results/2d_incremental.png "2D Incremental (ISAM2)"
```

**Results:**

- Initial error: 2,574,860
- Final error: 12,133 (99.5% reduction!)
- Output: `results/2d_incremental.png`

### Task 3.2.B & 3.3.C - 3D Optimizations

**Note:** The 3D parking garage dataset needs to be downloaded separately. You can:

1. **Option A:** Download from G2O datasets:

   ```bash
   # Try various sources
   cd data
   wget https://lucacarlone.mit.edu/datasets/parking-garage.g2o
   # OR
   wget https://www.ipb.uni-bonn.de/html/projects/g2o/parking-garage.g2o
   ```

2. **Option B:** Use an alternative 3D dataset (e.g., from https://github.com/RainerKuemmerle/g2o/tree/master/g2o/examples)

Once you have the dataset:

```bash
# Batch optimization
./build/graph_slam 3d-batch data/parking-garage.g2o results/3d_batch
python3 plot_3d_results.py results/3d_batch_initial.csv results/3d_batch_optimized.csv results/3d_batch.png

# Incremental optimization
./build/graph_slam 3d-incremental data/parking-garage.g2o results/3d_incremental
python3 plot_3d_results.py results/3d_incremental_initial.csv results/3d_incremental_optimized.csv results/3d_incremental.png
```

## Run All Tasks

```bash
chmod +x run_all.sh
./run_all.sh
```

This will:

1. Build the project
2. Run 2D batch optimization
3. Run 2D incremental optimization
4. Run 3D optimizations (if dataset available)
5. Generate all visualization plots

## Viewing Results

All output files are saved in the `results/` directory:

- `*.csv` - Trajectory data (initial and optimized)
- `*.png` - Visualization plots

The plots show:

- **Red line**: Unoptimized trajectory
- **Blue line**: Optimized trajectory
- **Green circle**: Start position
- **Purple square**: End position

## Key Implementation Details

### Optimization Methods

**Levenberg-Marquardt (Batch):**

- Optimizes all poses simultaneously
- More robust than Gauss-Newton to poor initial estimates
- Used for Tasks 2.2.B and 3.2.B

**ISAM2 (Incremental):**

- Processes poses one at a time
- Uses Bayes tree for efficient inference
- Implements Algorithm 1 from the assignment
- Used for Tasks 2.3.C and 3.3.C

### Why ISAM2 Performs Better

The incremental optimization achieved much better results (error: 12,133 vs 385,129) because:

1. It processes constraints incrementally, avoiding large simultaneous updates
2. Better handles loop closures as they're discovered
3. More numerically stable for large graphs
4. Uses previous optimized poses as initial estimates for new poses

## Project Structure

```
TP4/
├── build/                     # Build directory
│   └── graph_slam            # Main executable
├── results/                   # Output directory
│   ├── 2d_batch*.csv/png     # Batch results
│   └── 2d_incremental*.csv/png # Incremental results
├── src/                       # Source code
│   ├── main.cpp              # Entry point
│   ├── g2o_reader.cpp        # G2O file parser
│   ├── graph_slam_2d.cpp     # 2D SLAM
│   ├── graph_slam_3d.cpp     # 3D SLAM
│   └── utils/                # Utilities
├── include/                   # Headers
├── plot_2d_results.py        # 2D visualization
├── plot_3d_results.py        # 3D visualization
├── run_all.sh                # Automated script
└── README.md                 # Full documentation
```

## Troubleshooting

### "GTSAM not found"

Ensure GTSAM is built in `../gtsam/build/`:

```bash
ls ../gtsam/build/GTSAMConfig.cmake
```

### "Python module not found"

Install required packages:

```bash
pip3 install matplotlib pandas numpy
```

### Build errors

Clean and rebuild:

```bash
cd build && rm -rf * && cmake .. && make
```

## References

- Assignment: `tp_graph_slam.pdf`
- GTSAM: https://gtsam.org/
- G2O Format: https://github.com/RainerKuemmerle/g2o/wiki/File-Format
