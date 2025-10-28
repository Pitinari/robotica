#include "stereo_vision/feature_extractor.hpp"
#include <iostream>

namespace stereo_vision
{

    FeatureExtractor::FeatureExtractor(DetectorType detector_type, DescriptorType descriptor_type)
        : detector_type_(detector_type), descriptor_type_(descriptor_type)
    {
        initializeDetector();
        initializeDescriptor();
    }

    void FeatureExtractor::initializeDetector()
    {
        switch (detector_type_)
        {
        case DetectorType::FAST:
            detector_ = cv::FastFeatureDetector::create(10);
            break;
        case DetectorType::ORB:
            detector_ = cv::ORB::create(1000);
            break;
        case DetectorType::GFTT:
            detector_ = cv::GFTTDetector::create(1000);
            break;
        case DetectorType::SIFT:
            detector_ = cv::SIFT::create(1000);
            break;
        case DetectorType::SURF:
            // SURF is in opencv_contrib
            std::cerr << "SURF not available in standard OpenCV build" << std::endl;
            detector_ = cv::ORB::create(1000);
            break;
        default:
            detector_ = cv::ORB::create(1000);
        }
    }

    void FeatureExtractor::initializeDescriptor()
    {
        switch (descriptor_type_)
        {
        case DescriptorType::ORB:
            descriptor_ = cv::ORB::create();
            break;
        case DescriptorType::BRISK:
            descriptor_ = cv::BRISK::create();
            break;
        case DescriptorType::BRIEF:
            // BRIEF requires a separate detector
            std::cerr << "BRIEF descriptor requires opencv_xfeatures2d" << std::endl;
            descriptor_ = cv::ORB::create();
            break;
        case DescriptorType::SIFT:
            descriptor_ = cv::SIFT::create();
            break;
        case DescriptorType::SURF:
            std::cerr << "SURF not available in standard OpenCV build" << std::endl;
            descriptor_ = cv::ORB::create();
            break;
        default:
            descriptor_ = cv::ORB::create();
        }
    }

    void FeatureExtractor::extract(const cv::Mat &image,
                                   std::vector<cv::KeyPoint> &keypoints,
                                   cv::Mat &descriptors)
    {
        detectKeypoints(image, keypoints);
        computeDescriptors(image, keypoints, descriptors);
    }

    void FeatureExtractor::detectKeypoints(const cv::Mat &image,
                                           std::vector<cv::KeyPoint> &keypoints)
    {
        if (!detector_)
        {
            std::cerr << "Detector not initialized!" << std::endl;
            return;
        }

        detector_->detect(image, keypoints);
        std::cout << "Detected " << keypoints.size() << " keypoints" << std::endl;
    }

    void FeatureExtractor::computeDescriptors(const cv::Mat &image,
                                              std::vector<cv::KeyPoint> &keypoints,
                                              cv::Mat &descriptors)
    {
        if (!descriptor_)
        {
            std::cerr << "Descriptor not initialized!" << std::endl;
            return;
        }

        descriptor_->compute(image, keypoints, descriptors);
        std::cout << "Computed descriptors: " << descriptors.size() << std::endl;
    }

    cv::Mat FeatureExtractor::visualizeKeypoints(const cv::Mat &image,
                                                 const std::vector<cv::KeyPoint> &keypoints) const
    {
        cv::Mat output;
        cv::drawKeypoints(image, keypoints, output, cv::Scalar(0, 255, 0),
                          cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        return output;
    }

    void FeatureExtractor::setDetectorParams(int max_features, float threshold)
    {
        switch (detector_type_)
        {
        case DetectorType::FAST:
            detector_ = cv::FastFeatureDetector::create(static_cast<int>(threshold));
            break;
        case DetectorType::ORB:
            detector_ = cv::ORB::create(max_features);
            break;
        case DetectorType::GFTT:
            detector_ = cv::GFTTDetector::create(max_features);
            break;
        case DetectorType::SIFT:
            detector_ = cv::SIFT::create(max_features);
            break;
        default:
            break;
        }
    }

} // namespace stereo_vision
