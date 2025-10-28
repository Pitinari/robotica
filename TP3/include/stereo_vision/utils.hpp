#ifndef STEREO_VISION_UTILS_HPP
#define STEREO_VISION_UTILS_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <fstream>
#include <vector>
#include <Eigen/Dense>

namespace stereo_vision
{

    /**
     * @brief Camera calibration parameters
     */
    struct CameraCalibration
    {
        cv::Mat camera_matrix;
        cv::Mat distortion_coeffs;
        cv::Size image_size;
        std::string camera_model;
        cv::Mat projection_matrix; // Rectified projection matrix (3x4)
    };

    /**
     * @brief Stereo camera calibration parameters
     */
    struct StereoCalibration
    {
        CameraCalibration left;
        CameraCalibration right;
        cv::Mat R;       // Rotation matrix from left to right camera
        cv::Mat T;       // Translation vector from left to right camera
        double baseline; // Distance between cameras
    };

    /**
     * @brief Ground truth pose data
     */
    struct GroundTruthPose
    {
        double timestamp;
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        Eigen::Matrix4d transformation_matrix;
    };

    /**
     * @brief Load camera calibration from YAML file
     */
    bool loadCameraCalibration(const std::string &yaml_path, CameraCalibration &calib);

    /**
     * @brief Load stereo calibration from YAML files
     */
    bool loadStereoCalibration(const std::string &left_yaml,
                               const std::string &right_yaml,
                               const std::string &extrinsics_yaml,
                               StereoCalibration &stereo_calib);

    /**
     * @brief Read ground truth poses from CSV file
     */
    std::vector<GroundTruthPose> readGroundTruthPoses(const std::string &csv_path);

    /**
     * @brief Convert Eigen transformation to OpenCV rotation and translation
     */
    void eigenToCV(const Eigen::Matrix4d &transform, cv::Mat &R, cv::Mat &t);

    /**
     * @brief Convert OpenCV rotation and translation to Eigen transformation
     */
    Eigen::Matrix4d cvToEigen(const cv::Mat &R, const cv::Mat &t);

    /**
     * @brief Save image with timestamp to output directory
     */
    void saveImage(const cv::Mat &image, const std::string &prefix, const std::string &output_dir = "output");

    /**
     * @brief Create output directory if it doesn't exist
     */
    void ensureOutputDirectory(const std::string &path);

    /**
     * @brief Draw keypoints on image
     */
    cv::Mat drawKeypoints(const cv::Mat &image, const std::vector<cv::KeyPoint> &keypoints);

    /**
     * @brief Draw matches between two images
     */
    cv::Mat drawMatches(const cv::Mat &img1, const std::vector<cv::KeyPoint> &kp1,
                        const cv::Mat &img2, const std::vector<cv::KeyPoint> &kp2,
                        const std::vector<cv::DMatch> &matches);

} // namespace stereo_vision

#endif // STEREO_VISION_UTILS_HPP
