#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "stereo_vision/stereo_rectifier.hpp"
#include "stereo_vision/utils.hpp"

class StereoRectifierNode : public rclcpp::Node
{
public:
    StereoRectifierNode() : Node("stereo_rectifier_node")
    {
        this->declare_parameter("left_calib_file", "config/camera_info_left.yaml");
        this->declare_parameter("right_calib_file", "config/camera_info_right.yaml");
        this->declare_parameter("extrinsics_file", "config/extrinsics.yaml");

        std::string left_calib = this->get_parameter("left_calib_file").as_string();
        std::string right_calib = this->get_parameter("right_calib_file").as_string();
        std::string extrinsics = this->get_parameter("extrinsics_file").as_string();

        stereo_vision::StereoCalibration stereo_calib;
        if (!stereo_vision::loadStereoCalibration(left_calib, right_calib, extrinsics, stereo_calib))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load calibration");
            return;
        }

        rectifier_ = std::make_shared<stereo_vision::StereoRectifier>();
        if (!rectifier_->initialize(stereo_calib))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize rectifier");
            return;
        }

        left_sub_.subscribe(this, "/cam0/image_raw");
        right_sub_.subscribe(this, "/cam1/image_raw");

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), left_sub_, right_sub_);
        sync_->registerCallback(&StereoRectifierNode::callback, this);

        left_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/stereo/left/rect", 10);
        right_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/stereo/right/rect", 10);

        RCLCPP_DEBUG(this->get_logger(), "Stereo rectifier node initialized");
    }

private:
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
                  const sensor_msgs::msg::Image::ConstSharedPtr &right_msg)
    {
        try
        {
            cv_bridge::CvImageConstPtr left_ptr = cv_bridge::toCvShare(left_msg, "bgr8");
            cv_bridge::CvImageConstPtr right_ptr = cv_bridge::toCvShare(right_msg, "bgr8");

            cv::Mat left_rect, right_rect;
            rectifier_->rectify(left_ptr->image, right_ptr->image, left_rect, right_rect);

            cv_bridge::CvImage left_out, right_out;
            left_out.header = left_msg->header;
            left_out.encoding = "bgr8";
            left_out.image = left_rect;

            right_out.header = right_msg->header;
            right_out.encoding = "bgr8";
            right_out.image = right_rect;

            left_pub_->publish(*left_out.toImageMsg());
            right_pub_->publish(*right_out.toImageMsg());
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV error: %s", e.what());
        }
    }

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>
        SyncPolicy;
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;
    std::shared_ptr<stereo_vision::StereoRectifier> rectifier_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoRectifierNode>());
    rclcpp::shutdown();
    return 0;
}
