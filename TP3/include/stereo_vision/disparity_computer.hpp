#ifndef STEREO_VISION_DISPARITY_COMPUTER_HPP
#define STEREO_VISION_DISPARITY_COMPUTER_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

namespace stereo_vision
{

    /**
     * @brief Stereo matching algorithm type
     */
    enum class StereoMatcherType
    {
        BLOCK_MATCHING, // StereoBM
        SGBM,           // StereoSGBM - Semi-Global Block Matching
        SGBM_3WAY       // StereoSGBM with 3-way mode
    };

    /**
     * @brief Class for computing disparity maps from stereo images
     */
    class DisparityComputer
    {
    public:
        DisparityComputer(StereoMatcherType type = StereoMatcherType::SGBM);

        /**
         * @brief Compute disparity map
         */
        void compute(const cv::Mat &left_rect, const cv::Mat &right_rect,
                     cv::Mat &disparity);

        /**
         * @brief Get normalized disparity for visualization
         */
        cv::Mat getVisualizedDisparity(const cv::Mat &disparity) const;

        /**
         * @brief Set SGBM parameters
         */
        void setSGBMParams(int min_disparity = 0,
                           int num_disparities = 128,
                           int block_size = 5,
                           int P1 = 0,
                           int P2 = 0,
                           int disp_12_max_diff = 1,
                           int pre_filter_cap = 63,
                           int uniqueness_ratio = 10,
                           int speckle_window_size = 100,
                           int speckle_range = 32,
                           int mode = cv::StereoSGBM::MODE_SGBM);

        /**
         * @brief Set Block Matching parameters
         */
        void setBMParams(int min_disparity = 0,
                         int num_disparities = 128,
                         int block_size = 21,
                         int pre_filter_size = 9,
                         int pre_filter_cap = 31,
                         int texture_threshold = 10,
                         int uniqueness_ratio = 15,
                         int speckle_window_size = 100,
                         int speckle_range = 32);

        /**
         * @brief Get current matcher type
         */
        StereoMatcherType getMatcherType() const { return matcher_type_; }

    private:
        StereoMatcherType matcher_type_;
        cv::Ptr<cv::StereoMatcher> stereo_matcher_;

        void initializeMatcher();
    };

} // namespace stereo_vision

#endif // STEREO_VISION_DISPARITY_COMPUTER_HPP
