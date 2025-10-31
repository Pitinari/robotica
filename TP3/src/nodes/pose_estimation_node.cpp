#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <cmath>

#include "stereo_vision/pose_estimator.hpp"
#include "stereo_vision/feature_extractor.hpp"
#include "stereo_vision/feature_matcher.hpp"
#include "stereo_vision/utils.hpp"

class PoseEstimationNode : public rclcpp::Node
{
public:
    PoseEstimationNode() : Node("pose_estimation_node")
    {
        // Parameters
        this->declare_parameter("left_calib_file", "config/camera_info_left.yaml");
        this->declare_parameter("right_calib_file", "config/camera_info_right.yaml");
        this->declare_parameter("extrinsics_file", "config/extrinsics.yaml");

        const auto left_calib_file = this->get_parameter("left_calib_file").as_string();
        const auto right_calib_file = this->get_parameter("right_calib_file").as_string();
        const auto extrinsics_file = this->get_parameter("extrinsics_file").as_string();

        // Load calibration
        stereo_vision::CameraCalibration left_calib, right_calib;
        stereo_vision::StereoCalibration stereo_calib;
        if (!stereo_vision::loadCameraCalibration(left_calib_file, left_calib) ||
            !stereo_vision::loadCameraCalibration(right_calib_file, right_calib) ||
            !stereo_vision::loadStereoCalibration(left_calib_file, right_calib_file, extrinsics_file, stereo_calib))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load camera calibrations");
            return;
        }

        baseline_ = stereo_calib.baseline;
        RCLCPP_INFO(this->get_logger(), "Loaded baseline: %.6f m", baseline_);

        // Initialize pose estimator with left camera matrix (for monocular estimation)
        pose_estimator_ = std::make_unique<stereo_vision::PoseEstimator>();
        pose_estimator_->initialize(left_calib.camera_matrix);
        camera_matrix_ = left_calib.camera_matrix.clone();

        // Initialize feature extractor and matcher
        extractor_ = std::make_unique<stereo_vision::FeatureExtractor>();
        extractor_->setDetectorParams(1000, 10.0);
        matcher_ = std::make_unique<stereo_vision::FeatureMatcher>();

        // Subscribers for stereo images
        left_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/stereo/left/rect");
        right_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/stereo/right/rect");

        // Ground-truth subscribers (for scale)
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu0", 10, std::bind(&PoseEstimationNode::imuCallback, this, std::placeholders::_1));
        gt_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/leica/position", 10, std::bind(&PoseEstimationNode::gtCallback, this, std::placeholders::_1));

        // Synchronizer for stereo images
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
        sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *left_sub_, *right_sub_);
        sync_->registerCallback(std::bind(&PoseEstimationNode::callback, this, std::placeholders::_1, std::placeholders::_2));

        // Publishers
        left_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_estimation/left_camera_pose", 10);
        right_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_estimation/right_camera_pose", 10);
        trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("/pose_estimation/trajectory", 10);
        poses_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/pose_estimation/poses_array", 10);

        // Transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Initialize trajectory
        trajectory_.header.frame_id = "map";
        trajectory_.header.stamp = this->now();

        // Initialize cumulative pose (identity)
        cumulative_pose_ = Eigen::Matrix4d::Identity();
        have_prev_frame_ = false;
        last_gt_data_ = {rclcpp::Time(0), Eigen::Vector3d::Zero()};
        prev_gt_data_ = {rclcpp::Time(0), Eigen::Vector3d::Zero()};
        have_last_gt_ = false;
        have_prev_gt_ = false;

        RCLCPP_INFO(this->get_logger(), "PoseEstimationNode started");
    }

private:
    // Calibration
    cv::Mat camera_matrix_;
    double baseline_;

    // Components
    std::unique_ptr<stereo_vision::PoseEstimator> pose_estimator_;
    std::unique_ptr<stereo_vision::FeatureExtractor> extractor_;
    std::unique_ptr<stereo_vision::FeatureMatcher> matcher_;

    // Subscribers
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr gt_sub_;
    std::unique_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> sync_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_array_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Trajectory
    nav_msgs::msg::Path trajectory_;
    Eigen::Matrix4d cumulative_pose_;

    // Previous frame data for temporal tracking
    cv::Mat prev_left_image_;
    std::vector<cv::KeyPoint> prev_left_kp_;
    cv::Mat prev_left_desc_;
    bool have_prev_frame_;

    // Ground-truth for scale (stored with timestamps)
    struct GtData
    {
        rclcpp::Time timestamp;
        Eigen::Vector3d position;
    };
    GtData last_gt_data_;
    GtData prev_gt_data_;
    bool have_last_gt_;
    bool have_prev_gt_;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr /* msg */)
    {
        // Store IMU data if needed
    }

    void gtCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        if (!msg)
            return;
        last_gt_data_.timestamp = rclcpp::Time(msg->header.stamp);
        last_gt_data_.position = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
        have_last_gt_ = true;
    }

    void callback(const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
                  const sensor_msgs::msg::Image::ConstSharedPtr &right_msg)
    {
        try
        {
            // Convert images
            cv_bridge::CvImageConstPtr left_ptr = cv_bridge::toCvShare(left_msg, "bgr8");
            cv_bridge::CvImageConstPtr right_ptr = cv_bridge::toCvShare(right_msg, "bgr8");

            // Extract features
            std::vector<cv::KeyPoint> left_kp, right_kp;
            cv::Mat left_desc, right_desc;
            extractor_->extract(left_ptr->image, left_kp, left_desc);
            extractor_->extract(right_ptr->image, right_kp, right_desc);

            if (left_kp.empty() || right_kp.empty())
            {
                RCLCPP_DEBUG(this->get_logger(), "No features found");
                return;
            }

            // Match features
            std::vector<cv::DMatch> matches;
            matcher_->matchWithThreshold(left_desc, right_desc, matches, 30.0f);

            if (matches.size() < 8)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Not enough matches: %zu (need at least 8)", matches.size());
                return;
            }

            // Estimate relative pose between left and right cameras
            cv::Mat R, t;
            std::vector<uchar> inlier_mask;
            if (!pose_estimator_->estimateRelativePose(left_kp, right_kp, matches, R, t, inlier_mask))
            {
                RCLCPP_WARN(this->get_logger(), "Failed to estimate pose");
                return;
            }

            // Scale translation using baseline (cv::recoverPose returns unit vector)
            pose_estimator_->scaleTranslation(t, baseline_);

            // Publish left camera pose (identity, as it's the reference)
            geometry_msgs::msg::PoseStamped left_pose;
            left_pose.header.frame_id = "map";
            left_pose.header.stamp = left_msg->header.stamp;
            left_pose.pose.position.x = 0.0;
            left_pose.pose.position.y = 0.0;
            left_pose.pose.position.z = 0.0;
            left_pose.pose.orientation.w = 1.0;
            left_pose.pose.orientation.x = 0.0;
            left_pose.pose.orientation.y = 0.0;
            left_pose.pose.orientation.z = 0.0;
            left_pose_pub_->publish(left_pose);

            // Compute right camera pose relative to left
            Eigen::Matrix4d T_left_right = pose_estimator_->computeTransformation(R, t);

            // Publish right camera pose
            geometry_msgs::msg::PoseStamped right_pose;
            right_pose.header.frame_id = "map";
            right_pose.header.stamp = right_msg->header.stamp;
            right_pose.pose.position.x = T_left_right(0, 3);
            right_pose.pose.position.y = T_left_right(1, 3);
            right_pose.pose.position.z = T_left_right(2, 3);

            Eigen::Matrix3d R_eigen = T_left_right.block<3, 3>(0, 0);
            Eigen::Quaterniond q(R_eigen);
            right_pose.pose.orientation.w = q.w();
            right_pose.pose.orientation.x = q.x();
            right_pose.pose.orientation.y = q.y();
            right_pose.pose.orientation.z = q.z();
            right_pose_pub_->publish(right_pose);

            // Temporal tracking: estimate pose between consecutive left camera frames
            if (have_prev_frame_)
            {
                // Match features between previous and current left frame
                std::vector<cv::DMatch> temporal_matches;
                matcher_->matchWithThreshold(prev_left_desc_, left_desc, temporal_matches, 30.0f);

                if (temporal_matches.size() >= 8)
                {
                    // Estimate relative pose between frames
                    cv::Mat R_temporal, t_temporal;
                    std::vector<uchar> temporal_inlier_mask;
                    if (pose_estimator_->estimateRelativePose(prev_left_kp_, left_kp, temporal_matches, R_temporal, t_temporal, temporal_inlier_mask))
                    {
                        // Scale translation using ground-truth
                        // cv::recoverPose returns a unit vector, so norm(t_temporal) ≈ 1.0
                        if (have_last_gt_ && have_prev_gt_)
                        {
                            // Calculate GT distance between frames
                            double gt_distance = (last_gt_data_.position - prev_gt_data_.position).norm();
                            if (gt_distance > 0.01 && gt_distance < 5.0) // Threshold to avoid noise and outliers
                            {
                                // Scale the unit translation vector directly by GT distance
                                // cv::recoverPose returns unit vector, so we just scale by GT distance
                                pose_estimator_->scaleTranslation(t_temporal, gt_distance);

                                RCLCPP_DEBUG(this->get_logger(),
                                             "Scaled translation: GT distance=%.3f", gt_distance);
                            }
                            else
                            {
                                // If GT not available or invalid, don't update trajectory
                                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                                     "GT distance invalid: %.3f, skipping trajectory update", gt_distance);
                                return; // Skip this frame
                            }
                        }
                        else
                        {
                            // No GT available, skip trajectory update
                            return;
                        }

                        // Update cumulative pose
                        Eigen::Matrix4d T_rel = pose_estimator_->computeTransformation(R_temporal, t_temporal);
                        cumulative_pose_ = cumulative_pose_ * T_rel;

                        // Add to trajectory
                        geometry_msgs::msg::PoseStamped trajectory_pose;
                        trajectory_pose.header.frame_id = "map";
                        trajectory_pose.header.stamp = left_msg->header.stamp;
                        trajectory_pose.pose.position.x = cumulative_pose_(0, 3);
                        trajectory_pose.pose.position.y = cumulative_pose_(1, 3);
                        trajectory_pose.pose.position.z = cumulative_pose_(2, 3);

                        Eigen::Matrix3d R_cum = cumulative_pose_.block<3, 3>(0, 0);
                        Eigen::Quaterniond q_cum(R_cum);
                        trajectory_pose.pose.orientation.w = q_cum.w();
                        trajectory_pose.pose.orientation.x = q_cum.x();
                        trajectory_pose.pose.orientation.y = q_cum.y();
                        trajectory_pose.pose.orientation.z = q_cum.z();

                        trajectory_.poses.push_back(trajectory_pose);
                        trajectory_.header.stamp = left_msg->header.stamp;
                        trajectory_pub_->publish(trajectory_);
                    }
                }
            }

            // Store current frame for next iteration
            prev_left_image_ = left_ptr->image.clone();
            prev_left_kp_ = left_kp;
            prev_left_desc_ = left_desc.clone();
            have_prev_frame_ = true;

            // Update ground-truth tracking (store previous GT data for next frame)
            if (have_last_gt_)
            {
                prev_gt_data_ = last_gt_data_;
                have_prev_gt_ = true;
            }

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Estimated pose: R valid, t = [%.3f, %.3f, %.3f], baseline=%.3f",
                                 t.at<double>(0), t.at<double>(1), t.at<double>(2), baseline_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in pose estimation: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseEstimationNode>());
    rclcpp::shutdown();
    return 0;
}
