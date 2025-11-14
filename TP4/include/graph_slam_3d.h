#ifndef GRAPH_SLAM_3D_H
#define GRAPH_SLAM_3D_H

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <vector>

// Function to read 3D G2O data and extract poses and edges
void readG2O3D(const std::string& filename, std::vector<gtsam::Pose3>& poses, std::vector<gtsam::BetweenFactor<gtsam::Pose3>>& edges);

// Function to perform batch optimization using GTSAM
gtsam::Values optimizeBatch(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initialEstimate);

// Function to perform incremental optimization using ISAM2
gtsam::Values optimizeIncremental(const std::vector<gtsam::Pose3>& poses, const std::vector<gtsam::BetweenFactor<gtsam::Pose3>>& edges);

#endif // GRAPH_SLAM_3D_H