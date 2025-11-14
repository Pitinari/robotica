#ifndef G2O_READER_H
#define G2O_READER_H

#include <vector>
#include <string>
#include <Eigen/Dense>

struct Pose2 {
    int id;
    double x;
    double y;
    double theta;

    Pose2(int id, double x, double y, double theta) : id(id), x(x), y(y), theta(theta) {}
};

struct Edge2 {
    int id1;
    int id2;
    double dx;
    double dy;
    double dtheta;
    Eigen::Matrix<double, 6, 1> information;

    Edge2(int id1, int id2, double dx, double dy, double dtheta, const Eigen::Matrix<double, 6, 1>& info)
        : id1(id1), id2(id2), dx(dx), dy(dy), dtheta(dtheta), information(info) {}
};

class G2OReader {
public:
    static bool readG2OFile(const std::string& filename, std::vector<Pose2>& poses, std::vector<Edge2>& edges);
    static Eigen::Matrix3d constructCovariance(const Eigen::Matrix<double, 6, 1>& info);
};

#endif // G2O_READER_H