#ifndef GRAPH_SLAM_3D_H
#define GRAPH_SLAM_3D_H

#include <string>
#include <gtsam/nonlinear/Values.h>

// Batch optimization using Gauss-Newton
gtsam::Values optimize3DBatch(const std::string &g2oFile, const std::string &outputPrefix);

// Incremental optimization using ISAM2
gtsam::Values optimize3DIncremental(const std::string &g2oFile, const std::string &outputPrefix);

#endif // GRAPH_SLAM_3D_H