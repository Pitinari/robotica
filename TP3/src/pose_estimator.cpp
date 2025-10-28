#include "stereo_vision/pose_estimator.hpp"
#include <iostream>

namespace stereo_vision
{

    PoseEstimator::PoseEstimator() : initialized_(false) {}

    void PoseEstimator::initialize(const cv::Mat &camera_matrix)
    {
        camera_matrix_ = camera_matrix.clone();
        initialized_ = true;
        std::cout << "Pose estimator initialized with camera matrix:\n"
                  << camera_matrix_ << std::endl;
    }

    bool PoseEstimator::estimateRelativePose(const std::vector<cv::KeyPoint> &keypoints1,
                                             const std::vector<cv::KeyPoint> &keypoints2,
                                             const std::vector<cv::DMatch> &matches,
                                             cv::Mat &R, cv::Mat &t,
                                             std::vector<uchar> &inlier_mask)
    {
        if (!initialized_)
        {
            std::cerr << "Pose estimator not initialized!" << std::endl;
            return false;
        }

        // Extract matched points
        std::vector<cv::Point2f> points1, points2;
        for (const auto &match : matches)
        {
            points1.push_back(keypoints1[match.queryIdx].pt);
            points2.push_back(keypoints2[match.trainIdx].pt);
        }

        return estimateRelativePose(points1, points2, R, t, inlier_mask);
    }

    bool PoseEstimator::estimateRelativePose(const std::vector<cv::Point2f> &points1,
                                             const std::vector<cv::Point2f> &points2,
                                             cv::Mat &R, cv::Mat &t,
                                             std::vector<uchar> &inlier_mask)
    {
        if (!initialized_)
        {
            std::cerr << "Pose estimator not initialized!" << std::endl;
            return false;
        }

        if (points1.size() < 8)
        {
            std::cerr << "Not enough point correspondences (need at least 8)" << std::endl;
            return false;
        }

        // Find essential matrix using RANSAC
        essential_matrix_ = cv::findEssentialMat(
            points1, points2, camera_matrix_,
            cv::RANSAC, 0.999, 1.0, inlier_mask);

        if (essential_matrix_.empty())
        {
            std::cerr << "Failed to compute essential matrix" << std::endl;
            return false;
        }

        // Count inliers
        int inlier_count = 0;
        for (auto mask : inlier_mask)
        {
            if (mask)
                inlier_count++;
        }

        std::cout << "Essential matrix found with " << inlier_count
                  << " inliers out of " << points1.size() << " points" << std::endl;

        // Recover pose from essential matrix
        int num_inliers = cv::recoverPose(essential_matrix_, points1, points2,
                                          camera_matrix_, R, t, inlier_mask);

        std::cout << "Pose recovered with " << num_inliers << " inliers" << std::endl;
        std::cout << "Translation (unit vector): " << t.t() << std::endl;

        return true;
    }

    void PoseEstimator::scaleTranslation(cv::Mat &t, double scale_factor)
    {
        t *= scale_factor;
        std::cout << "Translation scaled by " << scale_factor
                  << ", new value: " << t.t() << std::endl;
    }

    Eigen::Matrix4d PoseEstimator::computeTransformation(const cv::Mat &R, const cv::Mat &t)
    {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                transform(i, j) = R.at<double>(i, j);
            }
            transform(i, 3) = t.at<double>(i);
        }

        return transform;
    }

    void PoseEstimator::updatePose(const cv::Mat &R_rel, const cv::Mat &t_rel,
                                   Eigen::Matrix4d &cumulative_pose)
    {
        Eigen::Matrix4d relative_transform = computeTransformation(R_rel, t_rel);
        cumulative_pose = cumulative_pose * relative_transform;
    }

    bool PoseEstimator::estimateStereoBaseline(const std::vector<cv::KeyPoint> &left_kp,
                                               const std::vector<cv::KeyPoint> &right_kp,
                                               const std::vector<cv::DMatch> &matches,
                                               cv::Mat &R, cv::Mat &t,
                                               double &baseline)
    {
        std::vector<uchar> inlier_mask;
        if (estimateRelativePose(left_kp, right_kp, matches, R, t, inlier_mask))
        {
            // The translation vector from recoverPose is unit length
            // So we need to scale it to get the actual baseline
            baseline = cv::norm(t);
            return true;
        }
        return false;
    }

    double PoseEstimator::computeReprojectionError(const std::vector<cv::Point2f> &points1,
                                                   const std::vector<cv::Point2f> &points2,
                                                   const cv::Mat &E,
                                                   const std::vector<uchar> &mask)
    {
        double total_error = 0.0;
        int count = 0;

        for (size_t i = 0; i < points1.size(); ++i)
        {
            if (!mask.empty() && !mask[i])
                continue;

            // Compute Sampson distance (first-order geometric error)
            cv::Mat pt1 = (cv::Mat_<double>(3, 1) << points1[i].x, points1[i].y, 1.0);
            cv::Mat pt2 = (cv::Mat_<double>(3, 1) << points2[i].x, points2[i].y, 1.0);

            cv::Mat Ex1 = E * pt1;
            cv::Mat Etx2 = E.t() * pt2;

            double num = pt2.dot(Ex1);
            double error = (num * num) /
                           (Ex1.at<double>(0) * Ex1.at<double>(0) +
                            Ex1.at<double>(1) * Ex1.at<double>(1) +
                            Etx2.at<double>(0) * Etx2.at<double>(0) +
                            Etx2.at<double>(1) * Etx2.at<double>(1));

            total_error += std::sqrt(error);
            count++;
        }

        double mean_error = count > 0 ? total_error / count : 0.0;
        std::cout << "Mean reprojection error: " << mean_error << " pixels" << std::endl;

        return mean_error;
    }

} // namespace stereo_vision
