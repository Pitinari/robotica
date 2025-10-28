#include "stereo_vision/disparity_computer.hpp"
#include <iostream>

namespace stereo_vision
{

    DisparityComputer::DisparityComputer(StereoMatcherType type) : matcher_type_(type)
    {
        initializeMatcher();
    }

    void DisparityComputer::initializeMatcher()
    {
        switch (matcher_type_)
        {
        case StereoMatcherType::BLOCK_MATCHING:
            stereo_matcher_ = cv::StereoBM::create(128, 21);
            break;
        case StereoMatcherType::SGBM:
            stereo_matcher_ = cv::StereoSGBM::create(
                0,    // minDisparity
                128,  // numDisparities (must be divisible by 16)
                5,    // blockSize
                600,  // P1
                2400, // P2
                1,    // disp12MaxDiff
                63,   // preFilterCap
                10,   // uniquenessRatio
                100,  // speckleWindowSize
                32,   // speckleRange
                cv::StereoSGBM::MODE_SGBM);
            break;
        case StereoMatcherType::SGBM_3WAY:
            stereo_matcher_ = cv::StereoSGBM::create(
                0, 128, 5, 600, 2400, 1, 63, 10, 100, 32,
                cv::StereoSGBM::MODE_SGBM_3WAY);
            break;
        default:
            stereo_matcher_ = cv::StereoSGBM::create();
        }

        std::cout << "Disparity computer initialized with ";
        switch (matcher_type_)
        {
        case StereoMatcherType::BLOCK_MATCHING:
            std::cout << "Block Matching";
            break;
        case StereoMatcherType::SGBM:
            std::cout << "SGBM";
            break;
        case StereoMatcherType::SGBM_3WAY:
            std::cout << "SGBM 3-Way";
            break;
        }
        std::cout << std::endl;
    }

    void DisparityComputer::compute(const cv::Mat &left_rect, const cv::Mat &right_rect,
                                    cv::Mat &disparity)
    {
        if (!stereo_matcher_)
        {
            std::cerr << "Stereo matcher not initialized!" << std::endl;
            return;
        }

        if (left_rect.empty() || right_rect.empty())
        {
            std::cerr << "Empty input images!" << std::endl;
            return;
        }

        // Convert to grayscale if needed
        cv::Mat left_gray, right_gray;
        if (left_rect.channels() == 3)
        {
            cv::cvtColor(left_rect, left_gray, cv::COLOR_BGR2GRAY);
        }
        else
        {
            left_gray = left_rect;
        }

        if (right_rect.channels() == 3)
        {
            cv::cvtColor(right_rect, right_gray, cv::COLOR_BGR2GRAY);
        }
        else
        {
            right_gray = right_rect;
        }

        // Compute disparity
        stereo_matcher_->compute(left_gray, right_gray, disparity);

        // Convert to float and normalize (disparity is computed in 16-bit fixed point)
        disparity.convertTo(disparity, CV_32F, 1.0 / 16.0);

        std::cout << "Computed disparity map: " << disparity.size() << std::endl;
    }

    cv::Mat DisparityComputer::getVisualizedDisparity(const cv::Mat &disparity) const
    {
        cv::Mat vis;

        // Normalize disparity for visualization
        double min_val, max_val;
        cv::minMaxLoc(disparity, &min_val, &max_val);

        cv::Mat disparity_normalized;
        disparity.convertTo(disparity_normalized, CV_8U, 255.0 / (max_val - min_val),
                            -min_val * 255.0 / (max_val - min_val));

        // Apply color map
        cv::applyColorMap(disparity_normalized, vis, cv::COLORMAP_JET);

        return vis;
    }

    void DisparityComputer::setSGBMParams(int min_disparity,
                                          int num_disparities,
                                          int block_size,
                                          int P1, int P2,
                                          int disp_12_max_diff,
                                          int pre_filter_cap,
                                          int uniqueness_ratio,
                                          int speckle_window_size,
                                          int speckle_range,
                                          int mode)
    {
        // Validate parameters
        if (num_disparities % 16 != 0)
        {
            std::cerr << "Warning: num_disparities must be divisible by 16, rounding up" << std::endl;
            num_disparities = ((num_disparities + 15) / 16) * 16;
        }

        if (block_size % 2 == 0 || block_size < 1)
        {
            std::cerr << "Warning: block_size must be odd and >= 1, using 5" << std::endl;
            block_size = 5;
        }

        // Set P1 and P2 based on block size if not provided
        if (P1 == 0)
            P1 = 8 * block_size * block_size;
        if (P2 == 0)
            P2 = 32 * block_size * block_size;

        stereo_matcher_ = cv::StereoSGBM::create(
            min_disparity, num_disparities, block_size,
            P1, P2, disp_12_max_diff, pre_filter_cap,
            uniqueness_ratio, speckle_window_size, speckle_range, mode);

        matcher_type_ = (mode == cv::StereoSGBM::MODE_SGBM_3WAY) ? StereoMatcherType::SGBM_3WAY : StereoMatcherType::SGBM;

        std::cout << "SGBM parameters updated" << std::endl;
    }

    void DisparityComputer::setBMParams(int min_disparity,
                                        int num_disparities,
                                        int block_size,
                                        int pre_filter_size,
                                        int pre_filter_cap,
                                        int texture_threshold,
                                        int uniqueness_ratio,
                                        int speckle_window_size,
                                        int speckle_range)
    {
        auto bm = cv::StereoBM::create(num_disparities, block_size);
        bm->setMinDisparity(min_disparity);
        bm->setPreFilterSize(pre_filter_size);
        bm->setPreFilterCap(pre_filter_cap);
        bm->setTextureThreshold(texture_threshold);
        bm->setUniquenessRatio(uniqueness_ratio);
        bm->setSpeckleWindowSize(speckle_window_size);
        bm->setSpeckleRange(speckle_range);

        stereo_matcher_ = bm;
        matcher_type_ = StereoMatcherType::BLOCK_MATCHING;

        std::cout << "Block Matching parameters updated" << std::endl;
    }

} // namespace stereo_vision
