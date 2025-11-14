#ifndef GRAPH_SLAM_2D_H
#define GRAPH_SLAM_2D_H

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <vector>

// Function to read G2O data and generate poses and edges
void readG2OData(const std::string& filename, std::vector<gtsam::Pose2>& poses, std::vector<gtsam::BetweenFactor<gtsam::Pose2>>& edges);

// Function to construct the factor graph from poses and edges
gtsam::NonlinearFactorGraph constructFactorGraph(const std::vector<gtsam::Pose2>& poses, const std::vector<gtsam::BetweenFactor<gtsam::Pose2>>& edges);

// Function to optimize the poses using GTSAM
gtsam::Values optimizePoses(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initialEstimate);

#endif // GRAPH_SLAM_2D_H