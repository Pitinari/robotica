#ifndef G2O_READER_H
#define G2O_READER_H

#include <vector>
#include <string>
#include <Eigen/Dense>

// Structs for 2D poses and edges
struct Pose2Data
{
    int id;
    double x;
    double y;
    double theta;
};

struct Edge2Data
{
    int id1;
    int id2;
    double dx;
    double dy;
    double dtheta;
    Eigen::Matrix<double, 6, 1> information; // Upper triangular: [q11 q12 q13 q22 q23 q33]
};

// Structs for 3D poses and edges
struct Pose3Data
{
    int id;
    double x, y, z;
    double qx, qy, qz, qw; // Quaternion
};

struct Edge3Data
{
    int id1;
    int id2;
    double dx, dy, dz;
    double qx, qy, qz, qw;                    // Quaternion
    Eigen::Matrix<double, 21, 1> information; // Upper triangular for 6x6 matrix
};

// Function to read 2D G2O files
bool readG2O2D(const std::string &filename,
               std::vector<Pose2Data> &poses,
               std::vector<Edge2Data> &edges);

// Function to read 3D G2O files
bool readG2O3D(const std::string &filename,
               std::vector<Pose3Data> &poses,
               std::vector<Edge3Data> &edges);

// Utility to convert upper triangular information to symmetric matrix (3x3 for 2D)
Eigen::Matrix3d informationToMatrix2D(const Eigen::Matrix<double, 6, 1> &info);

// Utility to convert upper triangular information to symmetric matrix (6x6 for 3D)
Eigen::Matrix<double, 6, 6> informationToMatrix3D(const Eigen::Matrix<double, 21, 1> &info);

#endif // G2O_READER_H