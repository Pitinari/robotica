#ifndef STEREO_VISION_FEATURE_EXTRACTOR_HPP
#define STEREO_VISION_FEATURE_EXTRACTOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <string>
#include <memory>

namespace stereo_vision
{

    /**
     * @brief Supported feature detector types
     */
    enum class DetectorType
    {
        FAST,
        ORB,
        GFTT, // Good Features To Track
        SIFT,
        SURF
    };

    /**
     * @brief Supported feature descriptor types
     */
    enum class DescriptorType
    {
        ORB,
        BRISK,
        BRIEF,
        SIFT,
        SURF
    };

    /**
     * @brief Class for feature detection and description
     */
    class FeatureExtractor
    {
    public:
        FeatureExtractor(DetectorType detector_type = DetectorType::ORB,
                         DescriptorType descriptor_type = DescriptorType::ORB);

        /**
         * @brief Extract keypoints and descriptors from image
         */
        void extract(const cv::Mat &image,
                     std::vector<cv::KeyPoint> &keypoints,
                     cv::Mat &descriptors);

        /**
         * @brief Extract only keypoints
         */
        void detectKeypoints(const cv::Mat &image,
                             std::vector<cv::KeyPoint> &keypoints);

        /**
         * @brief Compute descriptors for given keypoints
         */
        void computeDescriptors(const cv::Mat &image,
                                std::vector<cv::KeyPoint> &keypoints,
                                cv::Mat &descriptors);

        /**
         * @brief Visualize keypoints on image
         */
        cv::Mat visualizeKeypoints(const cv::Mat &image,
                                   const std::vector<cv::KeyPoint> &keypoints) const;

        /**
         * @brief Set detector parameters
         */
        void setDetectorParams(int max_features = 1000,
                               float threshold = 10.0f);

        DetectorType getDetectorType() const { return detector_type_; }
        DescriptorType getDescriptorType() const { return descriptor_type_; }

    private:
        DetectorType detector_type_;
        DescriptorType descriptor_type_;

        cv::Ptr<cv::Feature2D> detector_;
        cv::Ptr<cv::Feature2D> descriptor_;

        void initializeDetector();
        void initializeDescriptor();
    };

} // namespace stereo_vision

#endif // STEREO_VISION_FEATURE_EXTRACTOR_HPP
