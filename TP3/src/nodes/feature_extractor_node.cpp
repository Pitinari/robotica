#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "stereo_vision/feature_extractor.hpp"

class FeatureExtractorNode : public rclcpp::Node
{
public:
    FeatureExtractorNode() : Node("feature_extractor_node")
    {
        // Declare parameters
        this->declare_parameter("detector_type", "FAST");
        this->declare_parameter("descriptor_type", "ORB");
        this->declare_parameter("max_features", 1000);
        this->declare_parameter("threshold", 10.0);

        // Get parameters
        std::string detector_type = this->get_parameter("detector_type").as_string();
        std::string descriptor_type = this->get_parameter("descriptor_type").as_string();
        int max_features = this->get_parameter("max_features").as_int();
        double threshold = this->get_parameter("threshold").as_double();

        // For now, we'll always use FAST detector as requested
        // (The parameter parsing is there for future flexibility)

        // Initialize extractor with ORB detector
        extractor_ = std::make_shared<stereo_vision::FeatureExtractor>(
            stereo_vision::DetectorType::ORB, stereo_vision::DescriptorType::ORB);

        // Set parameters
        extractor_->setDetectorParams(max_features, threshold);

        // Create subscribers and publishers
        left_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/stereo/left/rect", 10,
            std::bind(&FeatureExtractorNode::left_callback, this, std::placeholders::_1));
        right_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/stereo/right/rect", 10,
            std::bind(&FeatureExtractorNode::right_callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("/features/image", 10);

        RCLCPP_DEBUG(this->get_logger(), "Feature extractor node initialized with ORB detector");
    }

private:
    void left_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        process_image(msg, true);
    }

    void right_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        process_image(msg, false);
    }

    void process_image(const sensor_msgs::msg::Image::ConstSharedPtr &msg, bool is_left)
    {
        try
        {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            std::vector<cv::KeyPoint> keypoints;
            cv::Mat descriptors;

            // Extract features
            extractor_->extract(cv_ptr->image, keypoints, descriptors);

            // Store the image and features
            if (is_left)
            {
                left_image_ = cv_ptr->image.clone();
                left_keypoints_ = keypoints;
                left_header_ = msg->header;
            }
            else
            {
                right_image_ = cv_ptr->image.clone();
                right_keypoints_ = keypoints;
                right_header_ = msg->header;
            }

            // Process if we have both images
            if (!left_image_.empty() && !right_image_.empty())
            {
                // Create combined visualization
                cv::Mat combined = createCombinedVisualization();

                // Publish visualization
                cv_bridge::CvImage out;
                out.header = left_header_;
                out.encoding = "bgr8";
                out.image = combined;
                pub_->publish(*out.toImageMsg());

                // Log feature counts
                // Only log every 50th frame to reduce spam
                static int log_counter = 0;
                if (++log_counter % 50 == 0)
                {
                    RCLCPP_DEBUG(this->get_logger(), "Left: %zu features, Right: %zu features",
                                 left_keypoints_.size(), right_keypoints_.size());
                }
            }
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV error in feature extraction: %s", e.what());
        }
    }

    cv::Mat createCombinedVisualization()
    {
        // Create smaller, cleaner keypoint visualization
        cv::Mat left_vis, right_vis;

        // Draw keypoints with smaller circles
        cv::drawKeypoints(left_image_, left_keypoints_, left_vis,
                          cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::drawKeypoints(right_image_, right_keypoints_, right_vis,
                          cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        // Resize circles to be smaller
        for (auto &kp : left_keypoints_)
            kp.size = std::min(kp.size, 3.0f);
        for (auto &kp : right_keypoints_)
            kp.size = std::min(kp.size, 3.0f);

        // Redraw with smaller circles
        cv::drawKeypoints(left_image_, left_keypoints_, left_vis,
                          cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::drawKeypoints(right_image_, right_keypoints_, right_vis,
                          cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        // Ensure both images have the same height
        if (left_vis.rows != right_vis.rows)
        {
            int target_height = std::min(left_vis.rows, right_vis.rows);
            cv::resize(left_vis, left_vis, cv::Size(left_vis.cols * target_height / left_vis.rows, target_height));
            cv::resize(right_vis, right_vis, cv::Size(right_vis.cols * target_height / right_vis.rows, target_height));
        }

        // Combine images side by side
        cv::Mat combined;
        cv::hconcat(left_vis, right_vis, combined);

        // Add text labels
        std::string left_text = "Left (" + std::to_string(left_keypoints_.size()) + " features)";
        std::string right_text = "Right (" + std::to_string(right_keypoints_.size()) + " features)";

        cv::putText(combined, left_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        cv::putText(combined, right_text, cv::Point(left_vis.cols + 10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        return combined;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    std::shared_ptr<stereo_vision::FeatureExtractor> extractor_;

    cv::Mat left_image_, right_image_;
    std::vector<cv::KeyPoint> left_keypoints_, right_keypoints_;
    std_msgs::msg::Header left_header_, right_header_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FeatureExtractorNode>());
    rclcpp::shutdown();
    return 0;
}
