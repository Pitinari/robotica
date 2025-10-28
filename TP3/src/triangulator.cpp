#include "stereo_vision/triangulator.hpp"
#include <iostream>

namespace stereo_vision
{

    Triangulator::Triangulator() : initialized_(false) {}

    void Triangulator::initialize(const cv::Mat &P1, const cv::Mat &P2)
    {
        P1_ = P1.clone();
        P2_ = P2.clone();
        initialized_ = true;
        std::cout << "Triangulator initialized" << std::endl;
    }

    void Triangulator::triangulate(const std::vector<cv::KeyPoint> &keypoints1,
                                   const std::vector<cv::KeyPoint> &keypoints2,
                                   const std::vector<cv::DMatch> &matches,
                                   std::vector<cv::Point3f> &points3d)
    {
        if (!initialized_)
        {
            std::cerr << "Triangulator not initialized!" << std::endl;
            return;
        }

        // Extract matched 2D points
        std::vector<cv::Point2f> points1, points2;
        for (const auto &match : matches)
        {
            points1.push_back(keypoints1[match.queryIdx].pt);
            points2.push_back(keypoints2[match.trainIdx].pt);
        }

        triangulate(points1, points2, points3d);
    }

    void Triangulator::triangulate(const std::vector<cv::Point2f> &points1,
                                   const std::vector<cv::Point2f> &points2,
                                   std::vector<cv::Point3f> &points3d)
    {
        if (!initialized_)
        {
            std::cerr << "Triangulator not initialized!" << std::endl;
            return;
        }

        if (points1.size() != points2.size() || points1.empty())
        {
            std::cerr << "Invalid point correspondences for triangulation" << std::endl;
            return;
        }

        // Triangulate points
        cv::Mat points4d;
        cv::triangulatePoints(P1_, P2_, points1, points2, points4d);

        // Convert from homogeneous to 3D coordinates
        points3d.clear();
        points3d.reserve(points4d.cols);

        for (int i = 0; i < points4d.cols; ++i)
        {
            cv::Mat col = points4d.col(i);
            float w = col.at<float>(3);

            if (std::abs(w) > 1e-6)
            {
                cv::Point3f pt(
                    col.at<float>(0) / w,
                    col.at<float>(1) / w,
                    col.at<float>(2) / w);

                // Additional validation: check for reasonable depth and position
                if (pt.z > 0.1 && pt.z < 100.0 &&
                    std::abs(pt.x) < 50.0 && std::abs(pt.y) < 50.0)
                {
                    points3d.push_back(pt);
                }
                else
                {
                    // Invalid point - push a point far away to be filtered later
                    points3d.push_back(cv::Point3f(0, 0, -1));
                }
            }
            else
            {
                // Invalid point - push a point far away to be filtered later
                points3d.push_back(cv::Point3f(0, 0, -1));
            }
        }

        std::cout << "Triangulated " << points3d.size() << " 3D points" << std::endl;
    }

    void Triangulator::filterByReprojectionError(const std::vector<cv::Point2f> &points1,
                                                 const std::vector<cv::Point2f> &points2,
                                                 std::vector<cv::Point3f> &points3d,
                                                 std::vector<int> &valid_indices,
                                                 float max_error)
    {
        valid_indices.clear();

        for (size_t i = 0; i < points3d.size(); ++i)
        {
            // Project 3D point back to both images
            cv::Mat point3d_h = (cv::Mat_<double>(4, 1) << points3d[i].x, points3d[i].y, points3d[i].z, 1.0);

            cv::Mat proj1 = P1_ * point3d_h;
            cv::Mat proj2 = P2_ * point3d_h;

            cv::Point2f reproj1(proj1.at<double>(0) / proj1.at<double>(2),
                                proj1.at<double>(1) / proj1.at<double>(2));
            cv::Point2f reproj2(proj2.at<double>(0) / proj2.at<double>(2),
                                proj2.at<double>(1) / proj2.at<double>(2));

            float error1 = cv::norm(reproj1 - points1[i]);
            float error2 = cv::norm(reproj2 - points2[i]);

            if (error1 < max_error && error2 < max_error)
            {
                valid_indices.push_back(i);
            }
        }

        std::cout << "Reprojection filtering: " << valid_indices.size()
                  << " valid points out of " << points3d.size() << std::endl;
    }

    void Triangulator::filterByDepth(std::vector<cv::Point3f> &points3d,
                                     std::vector<int> &valid_indices,
                                     float min_depth,
                                     float max_depth)
    {
        valid_indices.clear();

        for (size_t i = 0; i < points3d.size(); ++i)
        {
            float depth = points3d[i].z;
            // Filter out negative depths (invalid points) and points outside range
            if (depth > min_depth && depth < max_depth)
            {
                valid_indices.push_back(i);
            }
        }

        std::cout << "Depth filtering: " << valid_indices.size()
                  << " valid points out of " << points3d.size() << std::endl;
    }

    void Triangulator::transformToWorld(const std::vector<cv::Point3f> &points_camera,
                                        const Eigen::Matrix4d &camera_to_world,
                                        std::vector<cv::Point3f> &points_world)
    {
        points_world.clear();
        points_world.reserve(points_camera.size());

        for (const auto &pt : points_camera)
        {
            Eigen::Vector4d pt_cam(pt.x, pt.y, pt.z, 1.0);
            Eigen::Vector4d pt_world = camera_to_world * pt_cam;

            points_world.push_back(cv::Point3f(
                pt_world(0), pt_world(1), pt_world(2)));
        }
    }

} // namespace stereo_vision
