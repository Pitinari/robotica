#include "utils/visualization.h"
#include <iostream>

// Visualization is now done via CSV output and Python scripts
// This file is kept for compatibility but doesn't use matplotlib-cpp

void visualizeTrajectory(const std::vector<Eigen::Vector2d> &trajectory)
{
    std::cout << "Trajectory visualization: Use Python plotting scripts on CSV output files" << std::endl;
}

void plotGraph(const std::vector<Eigen::Vector2d> &poses, const std::vector<std::pair<int, int>> &edges)
{
    std::cout << "Graph plotting: Use Python plotting scripts on CSV output files" << std::endl;
}
