#include <iostream>
#include <fstream>
#include <vector>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include "g2o_reader.h"
#include "graph_slam_2d.h"

using namespace std;
using namespace gtsam;

void GraphSlam2D::run(const string& g2oFile) {
    // Read poses and edges from G2O file
    vector<Pose2> poses;
    vector<Edge> edges;
    readG2O(g2oFile, poses, edges);

    // Construct the factor graph
    NonlinearFactorGraph graph;
    Values initialEstimate;

    // Add prior for the first pose
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));
    graph.add(PriorFactor<Pose2>(0, poses[0], priorNoise));
    initialEstimate.insert(0, poses[0]);

    // Add edges to the graph
    for (const auto& edge : edges) {
        graph.add(BetweenFactor<Pose2>(edge.id1, edge.id2, edge.pose, edge.model));
    }

    // Optimize the graph using Gauss-Newton
    GaussNewtonOptimizer optimizer(graph, initialEstimate);
    Values result = optimizer.optimize();

    // Output the optimized poses
    for (const auto& id : result.keys()) {
        Pose2 pose = result.at<Pose2>(id);
        cout << "Pose " << id << ": " << pose.x() << ", " << pose.y() << ", " << pose.theta() << endl;
    }
}

int main(int argc, char** argv) {
    if (argc != 2) {
        cerr << "Usage: " << argv[0] << " <g2o_file>" << endl;
        return 1;
    }

    string g2oFile = argv[1];
    GraphSlam2D graphSlam;
    graphSlam.run(g2oFile);

    return 0;
}