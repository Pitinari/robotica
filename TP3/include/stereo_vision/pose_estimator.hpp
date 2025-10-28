#ifndef STEREO_VISION_POSE_ESTIMATOR_HPP
#define STEREO_VISION_POSE_ESTIMATOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <Eigen/Dense>
#include <vector>

namespace stereo_vision
{

    /**
     * @brief Class for monocular visual odometry and pose estimation
     */
    class PoseEstimator
    {
    public:
        PoseEstimator();

        /**
         * @brief Initialize with camera intrinsics
         */
        void initialize(const cv::Mat &camera_matrix);

        /**
         * @brief Estimate relative pose between two frames using matched features
         */
        bool estimateRelativePose(const std::vector<cv::KeyPoint> &keypoints1,
                                  const std::vector<cv::KeyPoint> &keypoints2,
                                  const std::vector<cv::DMatch> &matches,
                                  cv::Mat &R, cv::Mat &t,
                                  std::vector<uchar> &inlier_mask);

        /**
         * @brief Estimate relative pose from point correspondences
         */
        bool estimateRelativePose(const std::vector<cv::Point2f> &points1,
                                  const std::vector<cv::Point2f> &points2,
                                  cv::Mat &R, cv::Mat &t,
                                  std::vector<uchar> &inlier_mask);

        /**
         * @brief Scale translation vector using ground truth or known scale
         */
        void scaleTranslation(cv::Mat &t, double scale_factor);

        /**
         * @brief Compute transformation matrix from R and t
         */
        Eigen::Matrix4d computeTransformation(const cv::Mat &R, const cv::Mat &t);

        /**
         * @brief Update cumulative pose with new relative pose
         */
        void updatePose(const cv::Mat &R_rel, const cv::Mat &t_rel,
                        Eigen::Matrix4d &cumulative_pose);

        /**
         * @brief Estimate stereo baseline from stereo pair
         */
        bool estimateStereoBaseline(const std::vector<cv::KeyPoint> &left_kp,
                                    const std::vector<cv::KeyPoint> &right_kp,
                                    const std::vector<cv::DMatch> &matches,
                                    cv::Mat &R, cv::Mat &t,
                                    double &baseline);

        /**
         * @brief Compute reprojection error
         */
        double computeReprojectionError(const std::vector<cv::Point2f> &points1,
                                        const std::vector<cv::Point2f> &points2,
                                        const cv::Mat &E,
                                        const std::vector<uchar> &mask);

        bool isInitialized() const { return initialized_; }

    private:
        bool initialized_;
        cv::Mat camera_matrix_;
        cv::Mat essential_matrix_;
    };

} // namespace stereo_vision

#endif // STEREO_VISION_POSE_ESTIMATOR_HPP
