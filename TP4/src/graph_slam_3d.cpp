#include <iostream>
#include <fstream>
#include <vector>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include "g2o_reader.h"
#include "graph_slam_3d.h"

using namespace std;
using namespace gtsam;

void runGraphSlam3D(const string& g2oFile) {
    // Read poses and edges from the G2O file
    vector<Pose3> poses;
    vector<Edge> edges;
    if (!readG2O(g2oFile, poses, edges)) {
        cerr << "Failed to read G2O file: " << g2oFile << endl;
        return;
    }

    // Initialize the factor graph and initial estimate
    NonlinearFactorGraph graph;
    Values initialEstimate;

    // Build the graph from poses and edges
    for (const auto& pose : poses) {
        // Add prior for the first pose
        if (pose.id() == 0) {
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector6::Constant(1e-3));
            graph.add(PriorFactor<Pose3>(Symbol('x', pose.id()), pose, priorNoise));
        }
        initialEstimate.insert(Symbol('x', pose.id()), pose);
    }

    for (const auto& edge : edges) {
        noiseModel::Gaussian::shared_ptr model = noiseModel::Gaussian::Covariance(edge.info);
        graph.add(BetweenFactor<Pose3>(Symbol('x', edge.id1), Symbol('x', edge.id2), edge.pose, model));
    }

    // Optimize the graph using ISAM2
    ISAM2 isam;
    isam.update(graph, initialEstimate);
    Values result = isam.calculateEstimate();

    // Output the optimized poses
    for (const auto& id : poses) {
        Pose3 optimizedPose = result.at<Pose3>(Symbol('x', id.id()));
        cout << "Optimized Pose " << id.id() << ": " << optimizedPose << endl;
    }
}

int main(int argc, char** argv) {
    if (argc != 2) {
        cerr << "Usage: " << argv[0] << " <g2o_file>" << endl;
        return 1;
    }

    string g2oFile = argv[1];
    runGraphSlam3D(g2oFile);

    return 0;
}