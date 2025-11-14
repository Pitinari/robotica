#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <vector>
#include <Eigen/Dense>

// Function to visualize the optimized trajectory
void visualizeTrajectory(const std::vector<Eigen::Vector2d>& trajectory);

// Function to plot the graph of poses and edges
void plotGraph(const std::vector<Eigen::Vector2d>& poses, const std::vector<std::pair<int, int>>& edges);

#endif // VISUALIZATION_H