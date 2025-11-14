#include "g2o_reader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

struct Pose2 {
    int id;
    double x;
    double y;
    double theta;
};

struct Edge2 {
    int id1;
    int id2;
    double dx;
    double dy;
    double dtheta;
    Matrix<double, 6, 1> information;
};

vector<Pose2> readG2O2D(const string& filename) {
    vector<Pose2> poses;
    ifstream file(filename);
    string line;

    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return poses;
    }

    while (getline(file, line)) {
        if (line.substr(0, 10) == "VERTEX_SE2") {
            istringstream iss(line);
            Pose2 pose;
            iss >> line >> pose.id >> pose.x >> pose.y >> pose.theta;
            poses.push_back(pose);
        }
    }

    file.close();
    return poses;
}

vector<Edge2> readG2OEdges2D(const string& filename) {
    vector<Edge2> edges;
    ifstream file(filename);
    string line;

    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return edges;
    }

    while (getline(file, line)) {
        if (line.substr(0, 10) == "EDGE_SE2") {
            istringstream iss(line);
            Edge2 edge;
            Matrix<double, 6, 1> info;
            iss >> line >> edge.id1 >> edge.id2 >> edge.dx >> edge.dy >> edge.dtheta;
            for (int i = 0; i < 6; ++i) {
                iss >> info(i);
            }
            edge.information = info;
            edges.push_back(edge);
        }
    }

    file.close();
    return edges;
}

vector<Pose3> readG2O3D(const string& filename) {
    vector<Pose3> poses;
    ifstream file(filename);
    string line;

    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return poses;
    }

    while (getline(file, line)) {
        if (line.substr(0, 15) == "VERTEX_SE3:QUAT") {
            istringstream iss(line);
            Pose3 pose;
            iss >> line >> pose.id >> pose.x >> pose.y >> pose.z >> pose.qx >> pose.qy >> pose.qz >> pose.qw;
            poses.push_back(pose);
        }
    }

    file.close();
    return poses;
}

vector<Edge3> readG2OEdges3D(const string& filename) {
    vector<Edge3> edges;
    ifstream file(filename);
    string line;

    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
        return edges;
    }

    while (getline(file, line)) {
        if (line.substr(0, 15) == "EDGE_SE3:QUAT") {
            istringstream iss(line);
            Edge3 edge;
            Matrix<double, 21, 1> info;
            iss >> line >> edge.id1 >> edge.id2 >> edge.dx >> edge.dy >> edge.dz >> edge.qx >> edge.qy >> edge.qz >> edge.qw;
            for (int i = 0; i < 21; ++i) {
                iss >> info(i);
            }
            edge.information = info;
            edges.push_back(edge);
        }
    }

    file.close();
    return edges;
}