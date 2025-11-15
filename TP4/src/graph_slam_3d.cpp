#include "graph_slam_3d.h"
#include "g2o_reader.h"
#include <iostream>
#include <fstream>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
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

// Helper function to save 3D trajectory to CSV
void saveTrajectory3D(const string &filename, const Values &values)
{
    ofstream file(filename);
    if (!file.is_open())
    {
        cerr << "Error: Could not open file " << filename << endl;
        return;
    }

    file << "id,x,y,z,qx,qy,qz,qw" << endl;
    for (const auto &key_value : values)
    {
        Key key = key_value.key;
        Pose3 pose = values.at<Pose3>(key);
        Quaternion q = pose.rotation().toQuaternion();
        Point3 t = pose.translation();
        file << Symbol(key).index() << ","
             << t.x() << "," << t.y() << "," << t.z() << ","
             << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << endl;
    }
    file.close();
    cout << "Saved trajectory to " << filename << endl;
}

// Batch optimization using Gauss-Newton
Values optimize3DBatch(const string &g2oFile, const string &outputPrefix)
{
    cout << "\n=== 3D Batch Optimization (Gauss-Newton) ===" << endl;

    // Read G2O file
    vector<Pose3Data> poses;
    vector<Edge3Data> edges;
    if (!readG2O3D(g2oFile, poses, edges))
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
        Rot3 rotation = Rot3::Quaternion(pose.qw, pose.qx, pose.qy, pose.qz);
        Point3 translation(pose.x, pose.y, pose.z);
        Pose3 p(rotation, translation);
        initialEstimate.insert(X(pose.id), p);
    }

    // Add prior on first pose with strong constraint
    Vector6 priorSigmas;
    priorSigmas << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8; // (roll, pitch, yaw, x, y, z)
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(priorSigmas);

    Rot3 rotation0 = Rot3::Quaternion(poses[0].qw, poses[0].qx, poses[0].qy, poses[0].qz);
    Point3 translation0(poses[0].x, poses[0].y, poses[0].z);
    graph.addPrior(X(poses[0].id), Pose3(rotation0, translation0), priorNoise);

    // Add between factors for all edges
    for (const auto &edge : edges)
    {
        Rot3 rotMeas = Rot3::Quaternion(edge.qw, edge.qx, edge.qy, edge.qz);
        Point3 transMeas(edge.dx, edge.dy, edge.dz);
        Pose3 measurement(rotMeas, transMeas);

        Eigen::Matrix<double, 6, 6> infoMatrix = informationToMatrix3D(edge.information);
        noiseModel::Gaussian::shared_ptr noise = noiseModel::Gaussian::Information(infoMatrix);
        graph.add(BetweenFactor<Pose3>(X(edge.id1), X(edge.id2), measurement, noise));
    }

    cout << "Graph has " << graph.size() << " factors and " << initialEstimate.size() << " variables" << endl;

    // Save initial trajectory
    saveTrajectory3D(outputPrefix + "_initial.csv", initialEstimate);

    // Optimize using Gauss-Newton as specified in Task 3.2.B
    GaussNewtonParams params;
    params.setVerbosity("ERROR");
    params.setMaxIterations(100);
    GaussNewtonOptimizer optimizer(graph, initialEstimate, params);

    Values result = optimizer.optimize();

    cout << "Optimization complete!" << endl;
    cout << "Initial error: " << graph.error(initialEstimate) << endl;
    cout << "Final error: " << graph.error(result) << endl;

    // Save optimized trajectory
    saveTrajectory3D(outputPrefix + "_optimized.csv", result);

    return result;
}

// Incremental optimization using ISAM2
Values optimize3DIncremental(const string &g2oFile, const string &outputPrefix)
{
    cout << "\n=== 3D Incremental Optimization (ISAM2) ===" << endl;

    // Read G2O file
    vector<Pose3Data> poses;
    vector<Edge3Data> edges;
    if (!readG2O3D(g2oFile, poses, edges))
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
        Rot3 rotation = Rot3::Quaternion(pose.qw, pose.qx, pose.qy, pose.qz);
        Point3 translation(pose.x, pose.y, pose.z);
        Pose3 p(rotation, translation);
        initialEstimate.insert(X(pose.id), p);
    }

    // Save initial trajectory
    saveTrajectory3D(outputPrefix + "_initial.csv", initialEstimate);

    // Process poses incrementally
    for (size_t i = 0; i < poses.size(); ++i)
    {
        NonlinearFactorGraph newFactors;
        Values newValues;

        if (i == 0)
        {
            // First pose: add prior
            Vector6 priorSigmas;
            priorSigmas << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8;
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(priorSigmas);

            Rot3 rotation = Rot3::Quaternion(poses[i].qw, poses[i].qx, poses[i].qy, poses[i].qz);
            Point3 translation(poses[i].x, poses[i].y, poses[i].z);
            Pose3 pose(rotation, translation);

            newFactors.addPrior(X(poses[i].id), pose, priorNoise);
            newValues.insert(X(poses[i].id), pose);
        }
        else
        {
            // Use last optimized pose as initial estimate
            Values currentResult = isam.calculateEstimate();
            if (currentResult.exists(X(poses[i - 1].id)))
            {
                Pose3 prevPose = currentResult.at<Pose3>(X(poses[i - 1].id));
                newValues.insert(X(poses[i].id), prevPose);
            }
            else
            {
                Rot3 rotation = Rot3::Quaternion(poses[i].qw, poses[i].qx, poses[i].qy, poses[i].qz);
                Point3 translation(poses[i].x, poses[i].y, poses[i].z);
                newValues.insert(X(poses[i].id), Pose3(rotation, translation));
            }
        }

        // Add edges connected to current pose
        for (const auto &edge : edges)
        {
            if (edge.id2 == poses[i].id && edge.id1 < poses[i].id)
            {
                Rot3 rotMeas = Rot3::Quaternion(edge.qw, edge.qx, edge.qy, edge.qz);
                Point3 transMeas(edge.dx, edge.dy, edge.dz);
                Pose3 measurement(rotMeas, transMeas);

                Eigen::Matrix<double, 6, 6> infoMatrix = informationToMatrix3D(edge.information);
                noiseModel::Gaussian::shared_ptr noise = noiseModel::Gaussian::Information(infoMatrix);
                newFactors.add(BetweenFactor<Pose3>(X(edge.id1), X(edge.id2), measurement, noise));
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
    Vector6 priorSigmas;
    priorSigmas << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-8;
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(priorSigmas);

    Rot3 rotation0 = Rot3::Quaternion(poses[0].qw, poses[0].qx, poses[0].qy, poses[0].qz);
    Point3 translation0(poses[0].x, poses[0].y, poses[0].z);
    fullGraph.addPrior(X(poses[0].id), Pose3(rotation0, translation0), priorNoise);

    for (const auto &edge : edges)
    {
        Rot3 rotMeas = Rot3::Quaternion(edge.qw, edge.qx, edge.qy, edge.qz);
        Point3 transMeas(edge.dx, edge.dy, edge.dz);
        Pose3 measurement(rotMeas, transMeas);

        Eigen::Matrix<double, 6, 6> infoMatrix = informationToMatrix3D(edge.information);
        noiseModel::Gaussian::shared_ptr noise = noiseModel::Gaussian::Information(infoMatrix);
        fullGraph.add(BetweenFactor<Pose3>(X(edge.id1), X(edge.id2), measurement, noise));
    }

    cout << "Initial error: " << fullGraph.error(initialEstimate) << endl;
    cout << "Final error: " << fullGraph.error(result) << endl;

    // Save optimized trajectory
    saveTrajectory3D(outputPrefix + "_optimized.csv", result);

    return result;
}
