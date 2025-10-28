#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "stereo_vision/feature_extractor.hpp"
#include "stereo_vision/feature_matcher.hpp"

class FeatureMatcherNode : public rclcpp::Node
{
public:
    FeatureMatcherNode() : Node("feature_matcher_node")
    {
        // Declare parameters
        this->declare_parameter("distance_threshold", 30.0);
        this->declare_parameter("detector_type", "ORB");
        this->declare_parameter("descriptor_type", "ORB");
        this->declare_parameter("max_features", 1000);
        this->declare_parameter("threshold", 10.0);

        // Get parameters
        double distance_threshold = this->get_parameter("distance_threshold").as_double();
        int max_features = this->get_parameter("max_features").as_int();
        double threshold = this->get_parameter("threshold").as_double();

        // Initialize feature extractor
        extractor_ = std::make_shared<stereo_vision::FeatureExtractor>(
            stereo_vision::DetectorType::ORB, stereo_vision::DescriptorType::ORB);
        extractor_->setDetectorParams(max_features, threshold);

        // Initialize feature matcher
        matcher_ = std::make_shared<stereo_vision::FeatureMatcher>();
        distance_threshold_ = distance_threshold;

        // Create subscribers for rectified images
        left_sub_.subscribe(this, "/stereo/left/rect");
        right_sub_.subscribe(this, "/stereo/right/rect");

        // Create synchronizer
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), left_sub_, right_sub_);
        sync_->registerCallback(&FeatureMatcherNode::callback, this);

        // Create publishers
        all_matches_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/matches/all", 10);
        filtered_matches_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/matches/filtered", 10);
        ransac_matches_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/matches/ransac", 10);
        homography_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/matches/homography", 10);

        RCLCPP_DEBUG(this->get_logger(), "Feature matcher node initialized");
    }

private:
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
                  const sensor_msgs::msg::Image::ConstSharedPtr &right_msg)
    {
        try
        {
            cv_bridge::CvImageConstPtr left_ptr = cv_bridge::toCvShare(left_msg, "bgr8");
            cv_bridge::CvImageConstPtr right_ptr = cv_bridge::toCvShare(right_msg, "bgr8");

            // Extract features from both images
            std::vector<cv::KeyPoint> left_keypoints, right_keypoints;
            cv::Mat left_descriptors, right_descriptors;

            extractor_->extract(left_ptr->image, left_keypoints, left_descriptors);
            extractor_->extract(right_ptr->image, right_keypoints, right_descriptors);

            // Match features
            std::vector<cv::DMatch> all_matches, filtered_matches, ransac_matches;
            matcher_->match(left_descriptors, right_descriptors, all_matches);
            matcher_->matchWithThreshold(left_descriptors, right_descriptors, filtered_matches, distance_threshold_);

            // Apply RANSAC filtering
            cv::Mat homography;
            std::vector<uchar> inlier_mask;
            ransac_matches = all_matches; // Copy for RANSAC filtering
            matcher_->filterMatchesRANSAC(left_keypoints, right_keypoints, ransac_matches, homography, inlier_mask);

            // Create visualizations
            cv::Mat all_matches_img = createMatchesVisualization(left_ptr->image, right_ptr->image,
                                                                 left_keypoints, right_keypoints, all_matches, "All Matches");
            cv::Mat filtered_matches_img = createMatchesVisualization(left_ptr->image, right_ptr->image,
                                                                      left_keypoints, right_keypoints, filtered_matches, "Filtered Matches");
            cv::Mat ransac_matches_img = createMatchesVisualization(left_ptr->image, right_ptr->image,
                                                                    left_keypoints, right_keypoints, ransac_matches, "RANSAC Matches");

            // Create homography visualization (transformed points)
            cv::Mat homography_img = createHomographyVisualization(left_ptr->image, right_ptr->image,
                                                                   left_keypoints, right_keypoints, ransac_matches, homography);

            // Publish visualizations
            publishImage(all_matches_img, left_msg->header, all_matches_pub_);
            publishImage(filtered_matches_img, left_msg->header, filtered_matches_pub_);
            publishImage(ransac_matches_img, left_msg->header, ransac_matches_pub_);
            publishImage(homography_img, left_msg->header, homography_pub_);

            // Log results
            // Only log every 50th frame to reduce spam
            static int log_counter = 0;
            if (++log_counter % 50 == 0)
            {
                RCLCPP_DEBUG(this->get_logger(), "All matches: %zu, Filtered matches: %zu, RANSAC matches: %zu",
                             all_matches.size(), filtered_matches.size(), ransac_matches.size());
            }
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV error in feature matching: %s", e.what());
        }
    }

    cv::Mat createMatchesVisualization(const cv::Mat &left_img, const cv::Mat &right_img,
                                       const std::vector<cv::KeyPoint> &left_kp, const std::vector<cv::KeyPoint> &right_kp,
                                       const std::vector<cv::DMatch> &matches, const std::string &title)
    {
        // Create side-by-side image
        cv::Mat combined;
        cv::hconcat(left_img, right_img, combined);

        // Draw matches
        cv::Mat matches_img;
        cv::drawMatches(left_img, left_kp, right_img, right_kp, matches, matches_img,
                        cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(),
                        cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        // Add title and match count
        std::string text = title + " (" + std::to_string(matches.size()) + " matches)";
        cv::putText(matches_img, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        return matches_img;
    }

    cv::Mat createHomographyVisualization(const cv::Mat &left_img, const cv::Mat &right_img,
                                          const std::vector<cv::KeyPoint> &left_kp, const std::vector<cv::KeyPoint> & /*right_kp*/,
                                          const std::vector<cv::DMatch> &matches, const cv::Mat &homography)
    {
        // Create side-by-side image
        cv::Mat combined;
        cv::hconcat(left_img, right_img, combined);

        // Extract matched points from left image
        std::vector<cv::Point2f> left_points;
        for (const auto &match : matches)
        {
            left_points.push_back(left_kp[match.queryIdx].pt);
        }

        // Transform left points using homography
        std::vector<cv::Point2f> transformed_points;
        if (!homography.empty())
        {
            cv::perspectiveTransform(left_points, transformed_points, homography);
        }

        // Draw original points on left image (green circles)
        for (const auto &pt : left_points)
        {
            cv::circle(combined, pt, 3, cv::Scalar(0, 255, 0), -1);
        }

        // Draw transformed points on right image (red circles)
        for (const auto &pt : transformed_points)
        {
            cv::Point2f right_pt(pt.x + left_img.cols, pt.y); // Offset for right image
            cv::circle(combined, right_pt, 3, cv::Scalar(0, 0, 255), -1);
        }

        // Draw lines connecting original and transformed points
        for (size_t i = 0; i < left_points.size() && i < transformed_points.size(); ++i)
        {
            cv::Point2f start = left_points[i];
            cv::Point2f end(transformed_points[i].x + left_img.cols, transformed_points[i].y);
            cv::line(combined, start, end, cv::Scalar(255, 0, 0), 1);
        }

        // Add title
        std::string text = "Homography Transform (" + std::to_string(matches.size()) + " points)";
        cv::putText(combined, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);

        return combined;
    }

    void publishImage(const cv::Mat &image, const std_msgs::msg::Header &header,
                      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub)
    {
        cv_bridge::CvImage out;
        out.header = header;
        out.encoding = "bgr8";
        out.image = image;
        pub->publish(*out.toImageMsg());
    }

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>
        SyncPolicy;
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr all_matches_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr filtered_matches_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ransac_matches_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr homography_pub_;

    std::shared_ptr<stereo_vision::FeatureExtractor> extractor_;
    std::shared_ptr<stereo_vision::FeatureMatcher> matcher_;
    double distance_threshold_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FeatureMatcherNode>());
    rclcpp::shutdown();
    return 0;
}
