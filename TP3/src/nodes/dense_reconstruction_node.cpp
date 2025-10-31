#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "stereo_vision/dense_reconstructor.hpp"
#include "stereo_vision/utils.hpp"

class DenseReconstructionNode : public rclcpp::Node
{
public:
    DenseReconstructionNode() : Node("dense_reconstruction_node")
    {
        // Declare parameters
        this->declare_parameter("left_calib_file", "config/camera_info_left.yaml");
        this->declare_parameter("right_calib_file", "config/camera_info_right.yaml");
        this->declare_parameter("extrinsics_file", "config/extrinsics.yaml");
        this->declare_parameter("min_depth", 0.1);
        this->declare_parameter("max_depth", 50.0);
        this->declare_parameter("filter_invalid", true);

        // Get parameters
        std::string left_calib_file = this->get_parameter("left_calib_file").as_string();
        std::string right_calib_file = this->get_parameter("right_calib_file").as_string();
        std::string extrinsics_file = this->get_parameter("extrinsics_file").as_string();
        double min_depth = this->get_parameter("min_depth").as_double();
        double max_depth = this->get_parameter("max_depth").as_double();
        bool filter_invalid = this->get_parameter("filter_invalid").as_bool();

        // Load camera calibrations
        stereo_vision::CameraCalibration left_calib, right_calib;
        stereo_vision::StereoCalibration stereo_calib;

        if (!stereo_vision::loadCameraCalibration(left_calib_file, left_calib) ||
            !stereo_vision::loadCameraCalibration(right_calib_file, right_calib) ||
            !stereo_vision::loadStereoCalibration(left_calib_file, right_calib_file, extrinsics_file, stereo_calib))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load camera calibrations");
            return;
        }

        // Compute Q matrix using cv::stereoRectify()
        cv::Mat R1, R2, P1, P2, Q;
        cv::stereoRectify(
            left_calib.camera_matrix,
            left_calib.distortion_coeffs,
            right_calib.camera_matrix,
            right_calib.distortion_coeffs,
            left_calib.image_size,
            stereo_calib.R,
            stereo_calib.T,
            R1, R2, P1, P2, Q,
            cv::CALIB_ZERO_DISPARITY,
            0, // Alpha
            left_calib.image_size);

        RCLCPP_INFO(this->get_logger(), "Computed Q matrix from stereoRectify");
        RCLCPP_DEBUG(this->get_logger(), "Q matrix: fx=%.2f, baseline=%.3f",
                     Q.at<double>(2, 3), -1.0 / Q.at<double>(3, 2));

        // Initialize dense reconstructor with Q matrix
        reconstructor_ = std::make_shared<stereo_vision::DenseReconstructor>();
        reconstructor_->initialize(Q);
        reconstructor_->setDepthLimits(static_cast<float>(min_depth), static_cast<float>(max_depth));
        reconstructor_->setFilterInvalid(filter_invalid);

        // Create subscribers for disparity map (raw) and rectified left image
        disparity_sub_.subscribe(this, "/stereo/disparity_raw");
        left_image_sub_.subscribe(this, "/stereo/left/rect");

        // Create synchronizer
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), disparity_sub_, left_image_sub_);
        sync_->registerCallback(&DenseReconstructionNode::callback, this);

        // Create publisher for dense point cloud
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/stereo/dense_cloud", 10);

        RCLCPP_INFO(this->get_logger(), "Dense reconstruction node initialized");
        RCLCPP_INFO(this->get_logger(), "  Min depth: %.2f m", min_depth);
        RCLCPP_INFO(this->get_logger(), "  Max depth: %.2f m", max_depth);
        RCLCPP_INFO(this->get_logger(), "  Filter invalid: %s", filter_invalid ? "true" : "false");
    }

private:
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr &disparity_msg,
                  const sensor_msgs::msg::Image::ConstSharedPtr &left_msg)
    {
        try
        {
            // Convert disparity image to OpenCV
            // Note: The disparity_node publishes a colorized disparity for visualization,
            // but we need the raw disparity map. We'll need to get it from the disparity_node
            // or compute it here. For now, let's assume we can extract the raw disparity.

            // First, try to get the raw disparity. If the topic publishes raw disparity,
            // we need to handle CV_16SC1 or CV_32F format.
            cv_bridge::CvImageConstPtr disparity_ptr;
            cv::Mat disparity;

            try
            {
                // Try to get as 16-bit signed (typical for StereoBM/StereoSGBM)
                disparity_ptr = cv_bridge::toCvShare(disparity_msg, sensor_msgs::image_encodings::TYPE_16SC1);
                // Convert to float for reprojectImageTo3D
                // Note: CV_16SC1 stores disparity*16, so we scale by 1/16.0 to get actual disparity in pixels
                // reprojectImageTo3D expects CV_32F with actual pixel disparity values
                disparity_ptr->image.convertTo(disparity, CV_32F, 1.0 / 16.0);

                // Also filter out invalid disparities (negative or zero) before reprojection
                cv::Mat mask = (disparity > 0.0f);
                disparity.setTo(0.0f, ~mask);
            }
            catch (cv_bridge::Exception &e)
            {
                // Try 32-bit float
                try
                {
                    disparity_ptr = cv_bridge::toCvShare(disparity_msg, sensor_msgs::image_encodings::TYPE_32FC1);
                    disparity = disparity_ptr->image.clone();
                }
                catch (cv_bridge::Exception &e2)
                {
                    // If disparity is published as a colorized image, we need the raw disparity
                    // For now, log an error - the disparity_node should publish raw disparity
                    RCLCPP_WARN(this->get_logger(),
                                "Could not convert disparity image. Make sure disparity_node publishes raw disparity (16SC1 or 32FC1)");
                    return;
                }
            }

            // Convert left rectified image to OpenCV
            cv_bridge::CvImageConstPtr left_ptr = cv_bridge::toCvShare(left_msg, "bgr8");

            // Reconstruct dense point cloud using cv::reprojectImageTo3D() via DenseReconstructor
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            reconstructor_->reconstruct(disparity, left_ptr->image, cloud);

            if (cloud->points.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No valid points in dense reconstruction");
                return;
            }

            // Convert to ROS message
            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*cloud, cloud_msg);
            cloud_msg.header = left_msg->header;
            cloud_msg.header.frame_id = "cam0"; // Points are in camera frame

            // Publish dense point cloud
            cloud_pub_->publish(cloud_msg);

            RCLCPP_DEBUG(this->get_logger(), "Published dense point cloud with %zu points", cloud->points.size());
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV error in dense reconstruction: %s", e.what());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in dense reconstruction: %s", e.what());
        }
    }

    message_filters::Subscriber<sensor_msgs::msg::Image> disparity_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> left_image_sub_;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>>>
        sync_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    std::shared_ptr<stereo_vision::DenseReconstructor> reconstructor_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DenseReconstructionNode>());
    rclcpp::shutdown();
    return 0;
}
