#ifndef STEREO_VISION_TRIANGULATOR_HPP
#define STEREO_VISION_TRIANGULATOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Dense>

namespace stereo_vision
{

    /**
     * @brief Class for 3D point triangulation from stereo correspondences
     */
    class Triangulator
    {
    public:
        Triangulator();

        /**
         * @brief Initialize with projection matrices
         */
        void initialize(const cv::Mat &P1, const cv::Mat &P2);

        /**
         * @brief Triangulate points from matched keypoints
         */
        void triangulate(const std::vector<cv::KeyPoint> &keypoints1,
                         const std::vector<cv::KeyPoint> &keypoints2,
                         const std::vector<cv::DMatch> &matches,
                         std::vector<cv::Point3f> &points3d);

        /**
         * @brief Triangulate points from 2D point correspondences
         */
        void triangulate(const std::vector<cv::Point2f> &points1,
                         const std::vector<cv::Point2f> &points2,
                         std::vector<cv::Point3f> &points3d);

        /**
         * @brief Filter 3D points based on reprojection error
         */
        void filterByReprojectionError(const std::vector<cv::Point2f> &points1,
                                       const std::vector<cv::Point2f> &points2,
                                       std::vector<cv::Point3f> &points3d,
                                       std::vector<int> &valid_indices,
                                       float max_error = 2.0f);

        /**
         * @brief Filter 3D points based on depth (Z value)
         */
        void filterByDepth(std::vector<cv::Point3f> &points3d,
                           std::vector<int> &valid_indices,
                           float min_depth = 0.1f,
                           float max_depth = 100.0f);

        /**
         * @brief Transform 3D points to world frame
         */
        void transformToWorld(const std::vector<cv::Point3f> &points_camera,
                              const Eigen::Matrix4d &camera_to_world,
                              std::vector<cv::Point3f> &points_world);

        bool isInitialized() const { return initialized_; }

    private:
        bool initialized_;
        cv::Mat P1_; // Left camera projection matrix
        cv::Mat P2_; // Right camera projection matrix
    };

} // namespace stereo_vision

#endif // STEREO_VISION_TRIANGULATOR_HPP
