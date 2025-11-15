#include "graph_slam_2d.h"
#include "g2o_reader.h"
#include <iostream>
#include <fstream>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

using namespace std;
using namespace gtsam;
using symbol_shorthand::X;

// Helper function to save trajectory to CSV
void saveTrajectory2D(const string &filename, const Values &values)
{
    ofstream file(filename);
    if (!file.is_open())
    {
        cerr << "Error: Could not open file " << filename << endl;
        return;
    }

    file << "id,x,y,theta" << endl;
    for (const auto &key_value : values)
    {
        Key key = key_value.key;
        Pose2 pose = values.at<Pose2>(key);
        file << Symbol(key).index() << "," << pose.x() << "," << pose.y() << "," << pose.theta() << endl;
    }
    file.close();
    cout << "Saved trajectory to " << filename << endl;
}

// Batch optimization using Gauss-Newton
Values optimize2DBatch(const string &g2oFile, const string &outputPrefix)
{
    cout << "\n=== 2D Batch Optimization (Gauss-Newton) ===" << endl;

    // Read G2O file
    vector<Pose2Data> poses;
    vector<Edge2Data> edges;
    if (!readG2O2D(g2oFile, poses, edges))
    {
        cerr << "Failed to read G2O file" << endl;
        return Values();
    }

    // Build factor graph
    NonlinearFactorGraph graph;
    Values initialEstimate;

    // Add all poses to initial estimate
    for (const auto &pose : poses)
    {
        Pose2 p(pose.x, pose.y, pose.theta);
        initialEstimate.insert(X(pose.id), p);
    }

    // Add prior on first pose with strong constraint
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(1e-6, 1e-6, 1e-8));
    graph.addPrior(X(poses[0].id), Pose2(poses[0].x, poses[0].y, poses[0].theta), priorNoise);

    // Add between factors for all edges
    for (const auto &edge : edges)
    {
        Pose2 measurement(edge.dx, edge.dy, edge.dtheta);
        Eigen::Matrix3d infoMatrix = informationToMatrix2D(edge.information);
        noiseModel::Gaussian::shared_ptr noise = noiseModel::Gaussian::Information(infoMatrix);
        graph.add(BetweenFactor<Pose2>(X(edge.id1), X(edge.id2), measurement, noise));
    }

    cout << "Graph has " << graph.size() << " factors and " << initialEstimate.size() << " variables" << endl;

    // Save initial trajectory
    saveTrajectory2D(outputPrefix + "_initial.csv", initialEstimate);

    // Optimize using Gauss-Newton as specified in Task 2.2.B
    GaussNewtonParams params;
    params.setVerbosity("ERROR");
    params.setMaxIterations(100);
    GaussNewtonOptimizer optimizer(graph, initialEstimate, params);

    Values result = optimizer.optimize();

    cout << "Optimization complete!" << endl;
    cout << "Initial error: " << graph.error(initialEstimate) << endl;
    cout << "Final error: " << graph.error(result) << endl;

    // Save optimized trajectory
    saveTrajectory2D(outputPrefix + "_optimized.csv", result);

    return result;
}

// Incremental optimization using ISAM2
Values optimize2DIncremental(const string &g2oFile, const string &outputPrefix)
{
    cout << "\n=== 2D Incremental Optimization (ISAM2) ===" << endl;

    // Read G2O file
    vector<Pose2Data> poses;
    vector<Edge2Data> edges;
    if (!readG2O2D(g2oFile, poses, edges))
    {
        cerr << "Failed to read G2O file" << endl;
        return Values();
    }

    // Initialize ISAM2
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    ISAM2 isam(parameters);

    Values initialEstimate;

    // Add all poses to initial estimate first
    for (const auto &pose : poses)
    {
        Pose2 p(pose.x, pose.y, pose.theta);
        initialEstimate.insert(X(pose.id), p);
    }

    // Save initial trajectory
    saveTrajectory2D(outputPrefix + "_initial.csv", initialEstimate);

    // Build graph incrementally
    NonlinearFactorGraph graph;
    Values currentEstimate;

    // Process poses incrementally
    for (size_t i = 0; i < poses.size(); ++i)
    {
        NonlinearFactorGraph newFactors;
        Values newValues;

        if (i == 0)
        {
            // First pose: add prior
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(1e-6, 1e-6, 1e-8));
            newFactors.addPrior(X(poses[i].id), Pose2(poses[i].x, poses[i].y, poses[i].theta), priorNoise);
            newValues.insert(X(poses[i].id), Pose2(poses[i].x, poses[i].y, poses[i].theta));
        }
        else
        {
            // Use last optimized pose as initial estimate
            Values currentResult = isam.calculateEstimate();
            if (currentResult.exists(X(poses[i - 1].id)))
            {
                Pose2 prevPose = currentResult.at<Pose2>(X(poses[i - 1].id));
                newValues.insert(X(poses[i].id), prevPose);
            }
            else
            {
                newValues.insert(X(poses[i].id), Pose2(poses[i].x, poses[i].y, poses[i].theta));
            }
        }

        // Add edges connected to current pose
        for (const auto &edge : edges)
        {
            if (edge.id2 == poses[i].id && edge.id1 < poses[i].id)
            {
                Pose2 measurement(edge.dx, edge.dy, edge.dtheta);
                Eigen::Matrix3d infoMatrix = informationToMatrix2D(edge.information);
                noiseModel::Gaussian::shared_ptr noise = noiseModel::Gaussian::Information(infoMatrix);
                newFactors.add(BetweenFactor<Pose2>(X(edge.id1), X(edge.id2), measurement, noise));
            }
        }

        // Update ISAM2
        isam.update(newFactors, newValues);
    }

    // Get final result
    Values result = isam.calculateEstimate();

    cout << "Incremental optimization complete!" << endl;

    // Build full graph for error calculation
    NonlinearFactorGraph fullGraph;
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(1e-6, 1e-6, 1e-8));
    fullGraph.addPrior(X(poses[0].id), Pose2(poses[0].x, poses[0].y, poses[0].theta), priorNoise);
    for (const auto &edge : edges)
    {
        Pose2 measurement(edge.dx, edge.dy, edge.dtheta);
        Eigen::Matrix3d infoMatrix = informationToMatrix2D(edge.information);
        noiseModel::Gaussian::shared_ptr noise = noiseModel::Gaussian::Information(infoMatrix);
        fullGraph.add(BetweenFactor<Pose2>(X(edge.id1), X(edge.id2), measurement, noise));
    }

    cout << "Initial error: " << fullGraph.error(initialEstimate) << endl;
    cout << "Final error: " << fullGraph.error(result) << endl;

    // Save optimized trajectory
    saveTrajectory2D(outputPrefix + "_optimized.csv", result);

    return result;
}
