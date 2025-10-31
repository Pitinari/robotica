#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

class DisparityNode : public rclcpp::Node
{
public:
    DisparityNode() : Node("disparity_node")
    {
        // Declare parameters
        this->declare_parameter("method", "SGBM"); // "SGBM" or "BM"
        this->declare_parameter("min_disparity", 0);
        this->declare_parameter("num_disparities", 64);
        this->declare_parameter("block_size", 15);
        this->declare_parameter("speckle_window_size", 100);
        this->declare_parameter("speckle_range", 32);
        this->declare_parameter("uniqueness_ratio", 10);

        // Get parameters
        std::string method = this->get_parameter("method").as_string();
        int min_disparity = this->get_parameter("min_disparity").as_int();
        int num_disparities = this->get_parameter("num_disparities").as_int();
        int block_size = this->get_parameter("block_size").as_int();
        int speckle_window_size = this->get_parameter("speckle_window_size").as_int();
        int speckle_range = this->get_parameter("speckle_range").as_int();
        int uniqueness_ratio = this->get_parameter("uniqueness_ratio").as_int();

        // Ensure num_disparities is divisible by 16
        num_disparities = (num_disparities / 16) * 16;
        if (num_disparities == 0)
            num_disparities = 16;

        // Initialize stereo matcher
        if (method == "BM")
        {
            auto bm = cv::StereoBM::create(num_disparities, block_size);
            bm->setMinDisparity(min_disparity);
            bm->setSpeckleWindowSize(speckle_window_size);
            bm->setSpeckleRange(speckle_range);
            bm->setUniquenessRatio(uniqueness_ratio);
            matcher_ = bm;
            RCLCPP_INFO(this->get_logger(), "Using StereoBM method");
        }
        else // SGBM (default)
        {
            auto sgbm = cv::StereoSGBM::create(
                min_disparity,
                num_disparities,
                block_size,
                8 * block_size * block_size,  // P1
                32 * block_size * block_size, // P2
                1,                            // disp12MaxDiff
                0,                            // preFilterCap
                uniqueness_ratio,
                speckle_window_size,
                speckle_range,
                cv::StereoSGBM::MODE_SGBM_3WAY);
            matcher_ = sgbm;
            RCLCPP_INFO(this->get_logger(), "Using StereoSGBM method");
        }

        // Create subscribers for rectified images
        left_sub_.subscribe(this, "/stereo/left/rect");
        right_sub_.subscribe(this, "/stereo/right/rect");

        // Create synchronizer
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), left_sub_, right_sub_);
        sync_->registerCallback(&DisparityNode::callback, this);

        // Create publishers for disparity map (raw and colorized)
        disparity_raw_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/stereo/disparity_raw", 10);
        disparity_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/stereo/disparity", 10);

        RCLCPP_INFO(this->get_logger(), "Disparity node initialized");
        RCLCPP_INFO(this->get_logger(), "  Method: %s", method.c_str());
        RCLCPP_INFO(this->get_logger(), "  Min disparity: %d", min_disparity);
        RCLCPP_INFO(this->get_logger(), "  Num disparities: %d", num_disparities);
        RCLCPP_INFO(this->get_logger(), "  Block size: %d", block_size);
    }

private:
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
                  const sensor_msgs::msg::Image::ConstSharedPtr &right_msg)
    {
        try
        {
            // Convert ROS images to OpenCV (grayscale for disparity computation)
            cv_bridge::CvImageConstPtr left_ptr = cv_bridge::toCvShare(left_msg, sensor_msgs::image_encodings::MONO8);
            cv_bridge::CvImageConstPtr right_ptr = cv_bridge::toCvShare(right_msg, sensor_msgs::image_encodings::MONO8);

            // If images are not grayscale, convert them
            cv::Mat left_gray, right_gray;
            if (left_ptr->image.channels() == 3)
            {
                cv::cvtColor(left_ptr->image, left_gray, cv::COLOR_BGR2GRAY);
            }
            else
            {
                left_gray = left_ptr->image.clone();
            }

            if (right_ptr->image.channels() == 3)
            {
                cv::cvtColor(right_ptr->image, right_gray, cv::COLOR_BGR2GRAY);
            }
            else
            {
                right_gray = right_ptr->image.clone();
            }

            // Compute disparity map using cv::StereoMatcher::compute()
            cv::Mat disparity;
            matcher_->compute(left_gray, right_gray, disparity);

            // Normalize disparity for visualization (0-255)
            cv::Mat disparity_vis;
            double min_val, max_val;
            cv::minMaxLoc(disparity, &min_val, &max_val);

            if (max_val > min_val)
            {
                disparity.convertTo(disparity_vis, CV_8U, 255.0 / (max_val - min_val), -min_val * 255.0 / (max_val - min_val));
            }
            else
            {
                disparity_vis = cv::Mat::zeros(disparity.size(), CV_8U);
            }

            // Publish raw disparity map (for dense reconstruction)
            cv_bridge::CvImage disparity_raw_msg;
            disparity_raw_msg.header = left_msg->header;
            // Publish as 16SC1 (standard format for StereoBM/StereoSGBM output)
            if (disparity.type() == CV_16SC1)
            {
                disparity_raw_msg.encoding = sensor_msgs::image_encodings::TYPE_16SC1;
                disparity_raw_msg.image = disparity;
            }
            else if (disparity.type() == CV_32F)
            {
                disparity_raw_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
                disparity_raw_msg.image = disparity;
            }
            else
            {
                // Convert to 16SC1 if needed
                cv::Mat disparity_16s;
                disparity.convertTo(disparity_16s, CV_16SC1);
                disparity_raw_msg.encoding = sensor_msgs::image_encodings::TYPE_16SC1;
                disparity_raw_msg.image = disparity_16s;
            }
            disparity_raw_pub_->publish(*disparity_raw_msg.toImageMsg());

            // Apply colormap for better visualization
            cv::Mat disparity_colored;
            cv::applyColorMap(disparity_vis, disparity_colored, cv::COLORMAP_JET);

            // Convert to ROS message and publish colorized version
            cv_bridge::CvImage disparity_msg;
            disparity_msg.header = left_msg->header;
            disparity_msg.encoding = sensor_msgs::image_encodings::BGR8;
            disparity_msg.image = disparity_colored;
            disparity_pub_->publish(*disparity_msg.toImageMsg());
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV error in disparity computation: %s", e.what());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error processing images: %s", e.what());
        }
    }

    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>>>
        sync_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_pub_;
    cv::Ptr<cv::StereoMatcher> matcher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DisparityNode>());
    rclcpp::shutdown();
    return 0;
}
