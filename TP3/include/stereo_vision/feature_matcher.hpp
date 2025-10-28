#ifndef STEREO_VISION_FEATURE_MATCHER_HPP
#define STEREO_VISION_FEATURE_MATCHER_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

namespace stereo_vision
{

    /**
     * @brief Matching method type
     */
    enum class MatcherType
    {
        BRUTE_FORCE,
        FLANN
    };

    /**
     * @brief Class for feature matching between stereo images
     */
    class FeatureMatcher
    {
    public:
        FeatureMatcher(MatcherType type = MatcherType::BRUTE_FORCE);

        /**
         * @brief Match features between two images
         */
        void match(const cv::Mat &descriptors1,
                   const cv::Mat &descriptors2,
                   std::vector<cv::DMatch> &matches);

        /**
         * @brief Match features with distance threshold filtering
         */
        void matchWithThreshold(const cv::Mat &descriptors1,
                                const cv::Mat &descriptors2,
                                std::vector<cv::DMatch> &matches,
                                float max_distance = 30.0f);

        /**
         * @brief Match using ratio test (Lowe's ratio test)
         */
        void matchWithRatioTest(const cv::Mat &descriptors1,
                                const cv::Mat &descriptors2,
                                std::vector<cv::DMatch> &matches,
                                float ratio = 0.75f);

        /**
         * @brief Filter matches using RANSAC with homography
         */
        void filterMatchesRANSAC(const std::vector<cv::KeyPoint> &keypoints1,
                                 const std::vector<cv::KeyPoint> &keypoints2,
                                 std::vector<cv::DMatch> &matches,
                                 cv::Mat &homography,
                                 std::vector<uchar> &inlier_mask);

        /**
         * @brief Visualize matches between two images
         */
        cv::Mat visualizeMatches(const cv::Mat &img1,
                                 const std::vector<cv::KeyPoint> &keypoints1,
                                 const cv::Mat &img2,
                                 const std::vector<cv::KeyPoint> &keypoints2,
                                 const std::vector<cv::DMatch> &matches,
                                 const std::vector<uchar> &mask = std::vector<uchar>()) const;

        /**
         * @brief Get statistics about matches
         */
        void getMatchStatistics(const std::vector<cv::DMatch> &matches,
                                float &mean_distance,
                                float &std_distance,
                                float &min_distance,
                                float &max_distance) const;

    private:
        MatcherType matcher_type_;
        cv::Ptr<cv::DescriptorMatcher> matcher_;

        void initializeMatcher(int norm_type = cv::NORM_HAMMING);
    };

} // namespace stereo_vision

#endif // STEREO_VISION_FEATURE_MATCHER_HPP
