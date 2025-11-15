#include "g2o_reader.h"
#include <fstream>
#include <sstream>
#include <iostream>

using namespace std;

// Read 2D G2O file (VERTEX_SE2 and EDGE_SE2)
bool readG2O2D(const string &filename,
               vector<Pose2Data> &poses,
               vector<Edge2Data> &edges)
{
    ifstream file(filename);
    string line;

    if (!file.is_open())
    {
        cerr << "Error opening file: " << filename << endl;
        return false;
    }

    poses.clear();
    edges.clear();

    while (getline(file, line))
    {
        istringstream iss(line);
        string tag;
        iss >> tag;

        if (tag == "VERTEX_SE2")
        {
            Pose2Data pose;
            iss >> pose.id >> pose.x >> pose.y >> pose.theta;
            poses.push_back(pose);
        }
        else if (tag == "EDGE_SE2")
        {
            Edge2Data edge;
            iss >> edge.id1 >> edge.id2 >> edge.dx >> edge.dy >> edge.dtheta;
            // Read upper triangular information matrix: [q11 q12 q13 q22 q23 q33]
            for (int i = 0; i < 6; ++i)
            {
                iss >> edge.information(i);
            }
            edges.push_back(edge);
        }
    }

    file.close();
    cout << "Read " << poses.size() << " poses and " << edges.size() << " edges from " << filename << endl;
    return true;
}

// Read 3D G2O file (VERTEX_SE3:QUAT and EDGE_SE3:QUAT)
bool readG2O3D(const string &filename,
               vector<Pose3Data> &poses,
               vector<Edge3Data> &edges)
{
    ifstream file(filename);
    string line;

    if (!file.is_open())
    {
        cerr << "Error opening file: " << filename << endl;
        return false;
    }

    poses.clear();
    edges.clear();

    while (getline(file, line))
    {
        istringstream iss(line);
        string tag;
        iss >> tag;

        if (tag == "VERTEX_SE3:QUAT")
        {
            Pose3Data pose;
            iss >> pose.id >> pose.x >> pose.y >> pose.z >> pose.qx >> pose.qy >> pose.qz >> pose.qw;
            poses.push_back(pose);
        }
        else if (tag == "EDGE_SE3:QUAT")
        {
            Edge3Data edge;
            iss >> edge.id1 >> edge.id2 >> edge.dx >> edge.dy >> edge.dz >> edge.qx >> edge.qy >> edge.qz >> edge.qw;
            // Read upper triangular information matrix (21 elements for 6x6 symmetric)
            for (int i = 0; i < 21; ++i)
            {
                iss >> edge.information(i);
            }
            edges.push_back(edge);
        }
    }

    file.close();
    cout << "Read " << poses.size() << " poses and " << edges.size() << " edges from " << filename << endl;
    return true;
}

// Convert upper triangular information vector to 3x3 symmetric matrix for 2D
// info = [q11 q12 q13 q22 q23 q33]
Eigen::Matrix3d informationToMatrix2D(const Eigen::Matrix<double, 6, 1> &info)
{
    Eigen::Matrix3d mat;
    mat(0, 0) = info(0); // q11
    mat(0, 1) = info(1); // q12
    mat(0, 2) = info(2); // q13
    mat(1, 0) = info(1); // q12 (symmetric)
    mat(1, 1) = info(3); // q22
    mat(1, 2) = info(4); // q23
    mat(2, 0) = info(2); // q13 (symmetric)
    mat(2, 1) = info(4); // q23 (symmetric)
    mat(2, 2) = info(5); // q33
    return mat;
}

// Convert upper triangular information vector to 6x6 symmetric matrix for 3D
// info = [q11 q12 q13 q14 q15 q16 q22 q23 q24 q25 q26 q33 q34 q35 q36 q44 q45 q46 q55 q56 q66]
Eigen::Matrix<double, 6, 6> informationToMatrix3D(const Eigen::Matrix<double, 21, 1> &info)
{
    Eigen::Matrix<double, 6, 6> mat;
    int idx = 0;
    for (int i = 0; i < 6; ++i)
    {
        for (int j = i; j < 6; ++j)
        {
            mat(i, j) = info(idx);
            mat(j, i) = info(idx); // Make symmetric
            idx++;
        }
    }
    return mat;
}