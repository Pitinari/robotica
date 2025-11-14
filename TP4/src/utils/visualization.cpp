#include "utils/visualization.h"
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/Pose2.h>
#include <gtsam/slam/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <matplotlibcpp.h>
#include <vector>

namespace plt = matplotlibcpp;

void plotTrajectory(const gtsam::Values& result, const std::string& title) {
    std::vector<double> x, y;

    // Extract poses from the result
    for (const auto& key_value : result) {
        auto pose = result.at<gtsam::Pose2>(key_value.key);
        x.push_back(pose.x());
        y.push_back(pose.y());
    }

    // Create the plot
    plt::figure();
    plt::plot(x, y, "b-");
    plt::title(title);
    plt::xlabel("X Position");
    plt::ylabel("Y Position");
    plt::grid(true);
    plt::axis("equal");
    plt::show();
}

void plot3DTrajectory(const gtsam::Values& result, const std::string& title) {
    std::vector<double> x, y, z;

    // Extract poses from the result
    for (const auto& key_value : result) {
        auto pose = result.at<gtsam::Pose3>(key_value.key);
        x.push_back(pose.x());
        y.push_back(pose.y());
        z.push_back(pose.z());
    }

    // Create the 3D plot
    plt::figure();
    plt::plot3(x, y, z, "b-");
    plt::title(title);
    plt::xlabel("X Position");
    plt::ylabel("Y Position");
    plt::zlabel("Z Position");
    plt::grid(true);
    plt::show();
}