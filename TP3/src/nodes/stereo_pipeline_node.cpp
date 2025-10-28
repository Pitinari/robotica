#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "stereo_vision/stereo_rectifier.hpp"
#include "stereo_vision/feature_extractor.hpp"
#include "stereo_vision/feature_matcher.hpp"
#include "stereo_vision/triangulator.hpp"
#include "stereo_vision/disparity_computer.hpp"
#include "stereo_vision/dense_reconstructor.hpp"
#include "stereo_vision/pose_estimator.hpp"
#include "stereo_vision/utils.hpp"

class StereoPipelineNode : public rclcpp::Node
{
public:
    StereoPipelineNode() : Node("stereo_pipeline_node"), frame_count_(0)
    {
        // Declare parameters
        this->declare_parameter("left_calib_file", "config/camera_info_left.yaml");
        this->declare_parameter("right_calib_file", "config/camera_info_right.yaml");
        this->declare_parameter("extrinsics_file", "config/extrinsics.yaml");
        this->declare_parameter("output_dir", "output");
        this->declare_parameter("detector_type", "ORB");   // FAST, ORB, GFTT
        this->declare_parameter("descriptor_type", "ORB"); // ORB, BRISK
        this->declare_parameter("match_distance_threshold", 30.0);
        this->declare_parameter("use_ransac", true);
        this->declare_parameter("compute_disparity", true);
        this->declare_parameter("dense_reconstruction", true);
        this->declare_parameter("save_images", true);

        // Get parameters
        std::string left_calib = this->get_parameter("left_calib_file").as_string();
        std::string right_calib = this->get_parameter("right_calib_file").as_string();
        std::string extrinsics = this->get_parameter("extrinsics_file").as_string();
        output_dir_ = this->get_parameter("output_dir").as_string();
        save_images_ = this->get_parameter("save_images").as_bool();

        stereo_vision::ensureOutputDirectory(output_dir_);

        // Initialize components
        RCLCPP_INFO(this->get_logger(), "Initializing stereo vision pipeline...");

        // Load calibration
        stereo_vision::StereoCalibration stereo_calib;
        if (!stereo_vision::loadStereoCalibration(left_calib, right_calib, extrinsics, stereo_calib))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load calibration files");
            return;
        }

        // Initialize rectifier
        rectifier_ = std::make_shared<stereo_vision::StereoRectifier>();
        if (!rectifier_->initialize(stereo_calib))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize rectifier");
            return;
        }

        // Initialize feature extractor
        std::string detector_str = this->get_parameter("detector_type").as_string();
        std::string descriptor_str = this->get_parameter("descriptor_type").as_string();

        stereo_vision::DetectorType detector_type = stereo_vision::DetectorType::ORB;
        if (detector_str == "FAST")
            detector_type = stereo_vision::DetectorType::FAST;
        else if (detector_str == "GFTT")
            detector_type = stereo_vision::DetectorType::GFTT;

        stereo_vision::DescriptorType descriptor_type = stereo_vision::DescriptorType::ORB;
        if (descriptor_str == "BRISK")
            descriptor_type = stereo_vision::DescriptorType::BRISK;

        feature_extractor_ = std::make_shared<stereo_vision::FeatureExtractor>(detector_type, descriptor_type);

        // Initialize matcher
        matcher_ = std::make_shared<stereo_vision::FeatureMatcher>();

        // Initialize triangulator
        triangulator_ = std::make_shared<stereo_vision::Triangulator>();
        cv::Mat P1, P2;
        rectifier_->getProjectionMatrices(P1, P2);
        triangulator_->initialize(P1, P2);

        // Initialize disparity computer
        disparity_computer_ = std::make_shared<stereo_vision::DisparityComputer>(
            stereo_vision::StereoMatcherType::SGBM);

        // Initialize dense reconstructor
        dense_reconstructor_ = std::make_shared<stereo_vision::DenseReconstructor>();
        dense_reconstructor_->initialize(rectifier_->getQMatrix());

        // Initialize pose estimator
        pose_estimator_ = std::make_shared<stereo_vision::PoseEstimator>();
        cv::Mat K_left_new, K_right_new;
        rectifier_->getNewCameraMatrices(K_left_new, K_right_new);
        pose_estimator_->initialize(K_left_new);

        // Create subscribers
        left_sub_.subscribe(this, "/cam0/image_raw");
        right_sub_.subscribe(this, "/cam1/image_raw");

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), left_sub_, right_sub_);
        sync_->registerCallback(
            std::bind(&StereoPipelineNode::stereoCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Create publishers
        left_rect_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/stereo/left/rect", 10);
        right_rect_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/stereo/right/rect", 10);
        disparity_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/stereo/disparity", 10);
        sparse_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/stereo/sparse_cloud", 10);
        dense_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/stereo/dense_cloud", 10);
        trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("/stereo/trajectory", 10);

        // Initialize trajectory
        trajectory_.header.frame_id = "world";
        cumulative_pose_ = Eigen::Matrix4d::Identity();

        RCLCPP_INFO(this->get_logger(), "Stereo pipeline initialized successfully");
    }

private:
    void stereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
                        const sensor_msgs::msg::Image::ConstSharedPtr &right_msg)
    {
        frame_count_++;
        RCLCPP_INFO(this->get_logger(), "Processing frame %d", frame_count_);

        try
        {
            // Convert ROS images to OpenCV
            cv_bridge::CvImageConstPtr left_ptr = cv_bridge::toCvShare(left_msg, "bgr8");
            cv_bridge::CvImageConstPtr right_ptr = cv_bridge::toCvShare(right_msg, "bgr8");

            cv::Mat left_raw = left_ptr->image;
            cv::Mat right_raw = right_ptr->image;

            // Step 1: Rectify images
            cv::Mat left_rect, right_rect;
            rectifier_->rectify(left_raw, right_raw, left_rect, right_rect);

            if (save_images_ && frame_count_ == 1)
            {
                cv::imwrite(output_dir_ + "/left_rect.png", left_rect);
                cv::imwrite(output_dir_ + "/right_rect.png", right_rect);
            }

            // Publish rectified images
            publishImage(left_rect_pub_, left_rect, left_msg->header);
            publishImage(right_rect_pub_, right_rect, right_msg->header);

            // Step 2: Extract features
            std::vector<cv::KeyPoint> kp_left, kp_right;
            cv::Mat desc_left, desc_right;

            feature_extractor_->extract(left_rect, kp_left, desc_left);
            feature_extractor_->extract(right_rect, kp_right, desc_right);

            if (save_images_ && frame_count_ == 1)
            {
                cv::Mat left_features = feature_extractor_->visualizeKeypoints(left_rect, kp_left);
                cv::Mat right_features = feature_extractor_->visualizeKeypoints(right_rect, kp_right);
                cv::imwrite(output_dir_ + "/left_features.png", left_features);
                cv::imwrite(output_dir_ + "/right_features.png", right_features);
            }

            // Step 3: Match features
            std::vector<cv::DMatch> matches;
            float threshold = this->get_parameter("match_distance_threshold").as_double();
            matcher_->matchWithThreshold(desc_left, desc_right, matches, threshold);

            if (save_images_ && frame_count_ == 1)
            {
                cv::Mat all_matches = matcher_->visualizeMatches(
                    left_rect, kp_left, right_rect, kp_right, matches);
                cv::imwrite(output_dir_ + "/matches_all.png", all_matches);
            }

            // Step 4: RANSAC filtering
            if (this->get_parameter("use_ransac").as_bool() && !matches.empty())
            {
                cv::Mat homography;
                std::vector<uchar> inlier_mask;
                matcher_->filterMatchesRANSAC(kp_left, kp_right, matches, homography, inlier_mask);

                if (save_images_ && frame_count_ == 1)
                {
                    cv::Mat ransac_matches = matcher_->visualizeMatches(
                        left_rect, kp_left, right_rect, kp_right, matches, inlier_mask);
                    cv::imwrite(output_dir_ + "/matches_ransac.png", ransac_matches);

                    // Visualize perspective transform
                    std::vector<cv::Point2f> left_points, transformed_points;
                    for (const auto &m : matches)
                    {
                        left_points.push_back(kp_left[m.queryIdx].pt);
                    }
                    cv::perspectiveTransform(left_points, transformed_points, homography);

                    cv::Mat right_with_projected = right_rect.clone();
                    for (const auto &pt : transformed_points)
                    {
                        cv::circle(right_with_projected, pt, 3, cv::Scalar(0, 255, 0), -1);
                    }
                    cv::imwrite(output_dir_ + "/projected_points.png", right_with_projected);
                }
            }

            // Step 5: Triangulate sparse 3D points
            if (!matches.empty())
            {
                std::vector<cv::Point3f> points3d;
                triangulator_->triangulate(kp_left, kp_right, matches, points3d);

                // Publish sparse point cloud
                publishSparseCloud(points3d, left_msg->header);
            }

            // Step 6: Compute disparity map
            if (this->get_parameter("compute_disparity").as_bool())
            {
                cv::Mat disparity;
                disparity_computer_->compute(left_rect, right_rect, disparity);

                // Visualize and publish disparity
                cv::Mat disparity_vis = disparity_computer_->getVisualizedDisparity(disparity);
                publishImage(disparity_pub_, disparity_vis, left_msg->header);

                if (save_images_ && frame_count_ == 1)
                {
                    cv::imwrite(output_dir_ + "/disparity.png", disparity_vis);
                }

                // Step 7: Dense 3D reconstruction
                if (this->get_parameter("dense_reconstruction").as_bool())
                {
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dense_cloud(
                        new pcl::PointCloud<pcl::PointXYZRGB>);
                    dense_reconstructor_->reconstruct(disparity, left_rect, dense_cloud);

                    // Publish dense point cloud
                    sensor_msgs::msg::PointCloud2 cloud_msg;
                    pcl::toROSMsg(*dense_cloud, cloud_msg);
                    cloud_msg.header = left_msg->header;
                    cloud_msg.header.frame_id = "cam0";
                    dense_cloud_pub_->publish(cloud_msg);
                }
            }

            // Step 8: Pose estimation (temporal tracking)
            if (frame_count_ > 1 && !prev_keypoints_.empty() && !kp_left.empty())
            {
                std::vector<cv::DMatch> temporal_matches;
                matcher_->matchWithRatioTest(prev_descriptors_, desc_left, temporal_matches, 0.8f);

                if (temporal_matches.size() > 30)
                {
                    cv::Mat R, t;
                    std::vector<uchar> pose_mask;
                    if (pose_estimator_->estimateRelativePose(
                            prev_keypoints_, kp_left, temporal_matches, R, t, pose_mask))
                    {

                        // Scale translation using frame-to-frame motion (would use GT in practice)
                        double scale = 1.0; // Placeholder - should use ground truth
                        pose_estimator_->scaleTranslation(t, scale);

                        // Update cumulative pose
                        pose_estimator_->updatePose(R, t, cumulative_pose_);

                        // Add to trajectory
                        geometry_msgs::msg::PoseStamped pose_stamped;
                        pose_stamped.header = left_msg->header;
                        pose_stamped.header.frame_id = "world";
                        pose_stamped.pose.position.x = cumulative_pose_(0, 3);
                        pose_stamped.pose.position.y = cumulative_pose_(1, 3);
                        pose_stamped.pose.position.z = cumulative_pose_(2, 3);

                        Eigen::Matrix3d rot = cumulative_pose_.block<3, 3>(0, 0);
                        Eigen::Quaterniond quat(rot);
                        pose_stamped.pose.orientation.w = quat.w();
                        pose_stamped.pose.orientation.x = quat.x();
                        pose_stamped.pose.orientation.y = quat.y();
                        pose_stamped.pose.orientation.z = quat.z();

                        trajectory_.poses.push_back(pose_stamped);
                        trajectory_.header = left_msg->header;
                        trajectory_.header.frame_id = "world";
                        trajectory_pub_->publish(trajectory_);
                    }
                }
            }

            // Store current frame data for next iteration
            prev_keypoints_ = kp_left;
            prev_descriptors_ = desc_left.clone();
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV error: %s", e.what());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
        }
    }

    void publishImage(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub,
                      const cv::Mat &image, const std_msgs::msg::Header &header)
    {
        cv_bridge::CvImage cv_image;
        cv_image.header = header;
        cv_image.encoding = (image.channels() == 1) ? "mono8" : "bgr8";
        cv_image.image = image;
        pub->publish(*cv_image.toImageMsg());
    }

    void publishSparseCloud(const std::vector<cv::Point3f> &points3d,
                            const std_msgs::msg::Header &header)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.points.reserve(points3d.size());

        for (const auto &pt : points3d)
        {
            if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))
            {
                pcl::PointXYZ pcl_pt;
                pcl_pt.x = pt.x;
                pcl_pt.y = pt.y;
                pcl_pt.z = pt.z;
                cloud.points.push_back(pcl_pt);
            }
        }

        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = false;

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        cloud_msg.header = header;
        cloud_msg.header.frame_id = "cam0";
        sparse_cloud_pub_->publish(cloud_msg);
    }

    // ROS communication
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>
        SyncPolicy;
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_rect_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_rect_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sparse_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dense_cloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;

    // Processing components
    std::shared_ptr<stereo_vision::StereoRectifier> rectifier_;
    std::shared_ptr<stereo_vision::FeatureExtractor> feature_extractor_;
    std::shared_ptr<stereo_vision::FeatureMatcher> matcher_;
    std::shared_ptr<stereo_vision::Triangulator> triangulator_;
    std::shared_ptr<stereo_vision::DisparityComputer> disparity_computer_;
    std::shared_ptr<stereo_vision::DenseReconstructor> dense_reconstructor_;
    std::shared_ptr<stereo_vision::PoseEstimator> pose_estimator_;

    // State
    int frame_count_;
    std::string output_dir_;
    bool save_images_;
    std::vector<cv::KeyPoint> prev_keypoints_;
    cv::Mat prev_descriptors_;
    Eigen::Matrix4d cumulative_pose_;
    nav_msgs::msg::Path trajectory_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoPipelineNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
