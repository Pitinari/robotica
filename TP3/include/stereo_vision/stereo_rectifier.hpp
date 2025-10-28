#ifndef STEREO_VISION_STEREO_RECTIFIER_HPP
#define STEREO_VISION_STEREO_RECTIFIER_HPP

#include <opencv2/opencv.hpp>
#include "utils.hpp"

namespace stereo_vision
{

    /**
     * @brief Class for stereo image rectification
     */
    class StereoRectifier
    {
    public:
        StereoRectifier();

        /**
         * @brief Initialize rectification with calibration parameters
         */
        bool initialize(const StereoCalibration &stereo_calib);

        /**
         * @brief Rectify stereo image pair
         */
        void rectify(const cv::Mat &left_raw, const cv::Mat &right_raw,
                     cv::Mat &left_rect, cv::Mat &right_rect);

        /**
         * @brief Get rectification maps
         */
        void getRectificationMaps(cv::Mat &left_map1, cv::Mat &left_map2,
                                  cv::Mat &right_map1, cv::Mat &right_map2) const;

        /**
         * @brief Get projection matrices for rectified images
         */
        void getProjectionMatrices(cv::Mat &P1, cv::Mat &P2) const;

        /**
         * @brief Get Q matrix for 3D reprojection
         */
        cv::Mat getQMatrix() const { return Q_.clone(); }

        /**
         * @brief Get new camera matrices after rectification
         */
        void getNewCameraMatrices(cv::Mat &K1_new, cv::Mat &K2_new) const;

        bool isInitialized() const { return initialized_; }

    private:
        bool initialized_;

        // Rectification maps
        cv::Mat map1_left_, map2_left_;
        cv::Mat map1_right_, map2_right_;

        // Rectification transforms
        cv::Mat R1_, R2_; // Rotation matrices
        cv::Mat P1_, P2_; // Projection matrices
        cv::Mat Q_;       // Disparity-to-depth mapping matrix

        // New camera matrices after rectification
        cv::Mat K1_new_, K2_new_;

        cv::Size image_size_;
    };

} // namespace stereo_vision

#endif // STEREO_VISION_STEREO_RECTIFIER_HPP
