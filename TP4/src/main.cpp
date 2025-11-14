#include <iostream>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include "g2o_reader.h"
#include "graph_slam_2d.h"
#include "graph_slam_3d.h"

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {
    if (argc < 3) {
        cerr << "Usage: " << argv[0] << " <dataset_type> <dataset_path>" << endl;
        cerr << "dataset_type: 2d or 3d" << endl;
        return 1;
    }

    string dataset_type = argv[1];
    string dataset_path = argv[2];

    if (dataset_type == "2d") {
        // Initialize 2D Graph-SLAM
        NonlinearFactorGraph graph;
        Values initial_estimates;

        // Read G2O data
        if (!readG2OData(dataset_path, graph, initial_estimates)) {
            cerr << "Failed to read G2O data from " << dataset_path << endl;
            return 1;
        }

        // Optimize the graph
        Values result = optimizeGraph(graph, initial_estimates);

        // Visualize the results
        visualize2D(result);
    } else if (dataset_type == "3d") {
        // Initialize 3D Graph-SLAM
        NonlinearFactorGraph graph;
        Values initial_estimates;

        // Read G2O data
        if (!readG2OData(dataset_path, graph, initial_estimates)) {
            cerr << "Failed to read G2O data from " << dataset_path << endl;
            return 1;
        }

        // Optimize the graph
        Values result = optimizeGraph3D(graph, initial_estimates);

        // Visualize the results
        visualize3D(result);
    } else {
        cerr << "Invalid dataset type. Use '2d' or '3d'." << endl;
        return 1;
    }

    return 0;
}