#include "stereo_vision/stereo_rectifier.hpp"
#include <iostream>

namespace stereo_vision
{

    StereoRectifier::StereoRectifier() : initialized_(false) {}

    bool StereoRectifier::initialize(const StereoCalibration &stereo_calib)
    {
        try
        {
            image_size_ = stereo_calib.left.image_size;

            // Perform stereo rectification
            cv::stereoRectify(
                stereo_calib.left.camera_matrix,
                stereo_calib.left.distortion_coeffs,
                stereo_calib.right.camera_matrix,
                stereo_calib.right.distortion_coeffs,
                image_size_,
                stereo_calib.R,
                stereo_calib.T,
                R1_, R2_,
                P1_, P2_,
                Q_,
                cv::CALIB_ZERO_DISPARITY, // Makes images coplanar
                0,                        // Alpha parameter (0 = crop to valid pixels only)
                image_size_);

            // Compute rectification maps
            cv::initUndistortRectifyMap(
                stereo_calib.left.camera_matrix,
                stereo_calib.left.distortion_coeffs,
                R1_, P1_,
                image_size_,
                CV_32FC1,
                map1_left_, map2_left_);

            cv::initUndistortRectifyMap(
                stereo_calib.right.camera_matrix,
                stereo_calib.right.distortion_coeffs,
                R2_, P2_,
                image_size_,
                CV_32FC1,
                map1_right_, map2_right_);

            // Extract new camera matrices from projection matrices
            K1_new_ = P1_(cv::Rect(0, 0, 3, 3)).clone();
            K2_new_ = P2_(cv::Rect(0, 0, 3, 3)).clone();

            initialized_ = true;
            std::cout << "Stereo rectification initialized successfully" << std::endl;
            std::cout << "Image size: " << image_size_ << std::endl;
            std::cout << "Q matrix:\n"
                      << Q_ << std::endl;

            return true;
        }
        catch (const cv::Exception &e)
        {
            std::cerr << "Error initializing stereo rectification: " << e.what() << std::endl;
            return false;
        }
    }

    void StereoRectifier::rectify(const cv::Mat &left_raw, const cv::Mat &right_raw,
                                  cv::Mat &left_rect, cv::Mat &right_rect)
    {
        if (!initialized_)
        {
            std::cerr << "StereoRectifier not initialized!" << std::endl;
            return;
        }

        // Apply rectification maps
        cv::remap(left_raw, left_rect, map1_left_, map2_left_, cv::INTER_LINEAR);
        cv::remap(right_raw, right_rect, map1_right_, map2_right_, cv::INTER_LINEAR);
    }

    void StereoRectifier::getRectificationMaps(cv::Mat &left_map1, cv::Mat &left_map2,
                                               cv::Mat &right_map1, cv::Mat &right_map2) const
    {
        left_map1 = map1_left_.clone();
        left_map2 = map2_left_.clone();
        right_map1 = map1_right_.clone();
        right_map2 = map2_right_.clone();
    }

    void StereoRectifier::getProjectionMatrices(cv::Mat &P1, cv::Mat &P2) const
    {
        P1 = P1_.clone();
        P2 = P2_.clone();
    }

    void StereoRectifier::getNewCameraMatrices(cv::Mat &K1_new, cv::Mat &K2_new) const
    {
        K1_new = K1_new_.clone();
        K2_new = K2_new_.clone();
    }

} // namespace stereo_vision
