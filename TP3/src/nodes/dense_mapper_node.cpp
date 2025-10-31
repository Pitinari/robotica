#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <opencv2/core.hpp>
#include <cmath>

#include "stereo_vision/dense_reconstructor.hpp"
#include "stereo_vision/utils.hpp"

class DenseMapperNode : public rclcpp::Node
{
public:
    DenseMapperNode() : Node("dense_mapper_node")
    {
        // Parameters
        this->declare_parameter("left_calib_file", "config/camera_info_left.yaml");
        this->declare_parameter("right_calib_file", "config/camera_info_right.yaml");
        this->declare_parameter("extrinsics_file", "config/extrinsics.yaml");
        this->declare_parameter("min_depth", 0.1);
        this->declare_parameter("max_depth", 50.0);
        this->declare_parameter("filter_invalid", true);
        this->declare_parameter("point_subsample_factor", 16); // Keep only every Nth point (1=all, 16=6.25%)
        this->declare_parameter("max_distance", 20.0);         // Filter points beyond this distance in meters
        this->declare_parameter("min_point_distance", 0.05);   // Minimum distance between points (5cm) to avoid duplicates

        const auto left_calib_file = this->get_parameter("left_calib_file").as_string();
        const auto right_calib_file = this->get_parameter("right_calib_file").as_string();
        const auto extrinsics_file = this->get_parameter("extrinsics_file").as_string();
        double min_depth = this->get_parameter("min_depth").as_double();
        double max_depth = this->get_parameter("max_depth").as_double();
        bool filter_invalid = this->get_parameter("filter_invalid").as_bool();
        point_subsample_factor_ = this->get_parameter("point_subsample_factor").as_int();
        max_distance_ = this->get_parameter("max_distance").as_double();
        min_point_distance_ = this->get_parameter("min_point_distance").as_double();

        // Load calibration
        stereo_vision::CameraCalibration left_calib, right_calib;
        stereo_vision::StereoCalibration stereo_calib;
        if (!stereo_vision::loadCameraCalibration(left_calib_file, left_calib) ||
            !stereo_vision::loadCameraCalibration(right_calib_file, right_calib) ||
            !stereo_vision::loadStereoCalibration(left_calib_file, right_calib_file, extrinsics_file, stereo_calib))
        {
            throw std::runtime_error("Failed to load calibration");
        }

        // IMU->cam0 (EuRoC default)
        T_imu_cam0_ = cv::Mat::eye(4, 4, CV_64F);
        T_imu_cam0_.at<double>(0, 0) = 0.0148655429818;
        T_imu_cam0_.at<double>(0, 1) = -0.999880929698;
        T_imu_cam0_.at<double>(0, 2) = -0.00414029679422;
        T_imu_cam0_.at<double>(0, 3) = -0.0216401454975;
        T_imu_cam0_.at<double>(1, 0) = 0.999557249008;
        T_imu_cam0_.at<double>(1, 1) = 0.0149672133247;
        T_imu_cam0_.at<double>(1, 2) = -0.025715529948;
        T_imu_cam0_.at<double>(1, 3) = -0.064676986768;
        T_imu_cam0_.at<double>(2, 0) = 0.0257744366974;
        T_imu_cam0_.at<double>(2, 1) = -0.00375618835797;
        T_imu_cam0_.at<double>(2, 2) = 0.999660727178;
        T_imu_cam0_.at<double>(2, 3) = 0.00981073058949;

        // Compute Q matrix for dense reconstruction
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
            0,
            left_calib.image_size);

        // Initialize dense reconstructor
        reconstructor_ = std::make_unique<stereo_vision::DenseReconstructor>();
        reconstructor_->initialize(Q);
        reconstructor_->setDepthLimits(static_cast<float>(min_depth), static_cast<float>(max_depth));
        reconstructor_->setFilterInvalid(filter_invalid);

        // Subscribers
        disparity_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/stereo/disparity_raw");
        left_image_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/stereo/left/rect");
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu0", 10, std::bind(&DenseMapperNode::imuCb, this, std::placeholders::_1));
        gt_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/leica/position", 10, std::bind(&DenseMapperNode::gtCb, this, std::placeholders::_1));

        using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
        sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *disparity_sub_, *left_image_sub_);
        sync_->registerCallback(std::bind(&DenseMapperNode::callback, this, std::placeholders::_1, std::placeholders::_2));

        // Publishers (reliable + transient so RViz sees last map)
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.reliable();
        qos.transient_local();
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/dense_mapping/point_cloud", qos);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/dense_mapping/camera_pose", 10);

        global_map_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        global_map_->header.frame_id = "map";

        // Initialize KD-tree for fast spatial lookups
        kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZRGB>());

        // Create a timer to periodically publish the map even if no new data arrives
        // This ensures RViz can see the topic
        map_pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            [this]()
            { this->publishMap(); });

        // Publish empty map initially so RViz can see it (with transient_local QoS)
        publishMap();

        RCLCPP_INFO(this->get_logger(), "DenseMapperNode started (GT localization)");
    }

private:
    // Calib
    cv::Mat T_imu_cam0_;

    // Components
    std::unique_ptr<stereo_vision::DenseReconstructor> reconstructor_;

    // IO
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> disparity_sub_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_image_sub_;
    std::unique_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> sync_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr gt_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;

    // Map
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_;

    // Ground-truth data
    geometry_msgs::msg::PoseStamped last_imu_pose_{};
    bool have_imu_{false};
    geometry_msgs::msg::PointStamped last_gt_pos_{};
    bool have_gt_{false};

    // Performance: reduce number of points
    int point_subsample_factor_{1};
    double max_distance_{50.0};
    double min_point_distance_{0.05}; // Minimum distance between points (5cm)

    // KD-tree for fast spatial lookups (to avoid duplicate points)
    pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr kdtree_;

    void imuCb(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!msg)
            return;
        last_imu_pose_.header = msg->header;
        last_imu_pose_.pose.orientation = msg->orientation;
        if (!have_imu_)
        {
            have_imu_ = true;
            RCLCPP_INFO(this->get_logger(), "Received first IMU data");
        }
    }

    void gtCb(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        if (!msg)
            return;
        last_gt_pos_ = *msg;
        if (!have_gt_)
        {
            have_gt_ = true;
            RCLCPP_INFO(this->get_logger(), "Received first ground-truth position");
        }
    }

    void callback(const sensor_msgs::msg::Image::ConstSharedPtr &disparity_msg,
                  const sensor_msgs::msg::Image::ConstSharedPtr &left_msg)
    {
        try
        {
            RCLCPP_DEBUG(this->get_logger(), "Received synchronized disparity and left image");
            // Convert disparity to OpenCV
            cv_bridge::CvImageConstPtr disparity_ptr;
            cv::Mat disparity;

            try
            {
                disparity_ptr = cv_bridge::toCvShare(disparity_msg, sensor_msgs::image_encodings::TYPE_16SC1);
                disparity_ptr->image.convertTo(disparity, CV_32F, 1.0 / 16.0);
                // Filter invalid disparities
                cv::Mat mask = (disparity > 0.0f);
                disparity.setTo(0.0f, ~mask);
            }
            catch (cv_bridge::Exception &e)
            {
                try
                {
                    disparity_ptr = cv_bridge::toCvShare(disparity_msg, sensor_msgs::image_encodings::TYPE_32FC1);
                    disparity = disparity_ptr->image.clone();
                    cv::Mat mask = (disparity > 0.0f);
                    disparity.setTo(0.0f, ~mask);
                }
                catch (cv_bridge::Exception &e2)
                {
                    RCLCPP_WARN(this->get_logger(), "Could not convert disparity: %s", e2.what());
                    return;
                }
            }

            // Convert left image
            cv_bridge::CvImageConstPtr left_ptr = cv_bridge::toCvShare(left_msg, "bgr8");

            // Build world transform T_world_cam = T_world_imu * T_imu_cam0
            if (!(have_imu_ && have_gt_))
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Waiting for ground-truth data: have_imu=%d, have_gt=%d",
                                     have_imu_, have_gt_);
                // Still publish existing map even if waiting for GT data
                publishMap();
                return;
            }

            tf2::Transform T_world_imu;
            T_world_imu.setOrigin(tf2::Vector3(last_gt_pos_.point.x, last_gt_pos_.point.y, last_gt_pos_.point.z));
            tf2::Quaternion q(last_imu_pose_.pose.orientation.x, last_imu_pose_.pose.orientation.y,
                              last_imu_pose_.pose.orientation.z, last_imu_pose_.pose.orientation.w);
            T_world_imu.setRotation(q);

            tf2::Matrix3x3 R_ic(
                T_imu_cam0_.at<double>(0, 0), T_imu_cam0_.at<double>(0, 1), T_imu_cam0_.at<double>(0, 2),
                T_imu_cam0_.at<double>(1, 0), T_imu_cam0_.at<double>(1, 1), T_imu_cam0_.at<double>(1, 2),
                T_imu_cam0_.at<double>(2, 0), T_imu_cam0_.at<double>(2, 1), T_imu_cam0_.at<double>(2, 2));
            tf2::Vector3 t_ic(T_imu_cam0_.at<double>(0, 3), T_imu_cam0_.at<double>(1, 3), T_imu_cam0_.at<double>(2, 3));
            tf2::Transform T_ic(R_ic, t_ic);
            tf2::Transform T_wc = T_world_imu * T_ic;

            // Reconstruct dense point cloud
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            reconstructor_->reconstruct(disparity, left_ptr->image, local_cloud);

            if (local_cloud->points.empty())
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "No valid points after dense reconstruction");
                return;
            }

            RCLCPP_DEBUG(this->get_logger(), "Reconstructed %zu local points", local_cloud->points.size());

            // Subsample points to reduce total number (keep every Nth point)
            if (point_subsample_factor_ > 1)
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr subsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
                subsampled_cloud->points.reserve(local_cloud->points.size() / point_subsample_factor_);
                for (size_t i = 0; i < local_cloud->points.size(); i += point_subsample_factor_)
                {
                    subsampled_cloud->points.push_back(local_cloud->points[i]);
                }
                subsampled_cloud->width = subsampled_cloud->points.size();
                subsampled_cloud->height = 1;
                subsampled_cloud->is_dense = false;
                local_cloud = subsampled_cloud;
                RCLCPP_DEBUG(this->get_logger(), "Subsampled to %zu points (factor: %d)",
                             local_cloud->points.size(), point_subsample_factor_);
            }

            // Transform points to world frame
            pcl::PointCloud<pcl::PointXYZRGB> world_cloud;
            world_cloud.header.frame_id = "map";
            world_cloud.width = 0;
            world_cloud.height = 1;
            world_cloud.is_dense = false;
            world_cloud.points.reserve(local_cloud->points.size());

            for (const auto &pt : local_cloud->points)
            {
                // Transform from camera frame to world frame
                // Note: DenseReconstructor outputs points in camera frame
                // Flip image Y (OpenCV image coords are down-positive)
                tf2::Vector3 pc(pt.x, -pt.y, pt.z);
                tf2::Vector3 pw = T_wc * pc;

                // Filter points that are too far away
                double distance = std::sqrt(pw.x() * pw.x() + pw.y() * pw.y() + pw.z() * pw.z());
                if (distance > max_distance_)
                {
                    continue; // Skip points beyond max_distance
                }

                pcl::PointXYZRGB world_pt;
                world_pt.x = pw.x();
                world_pt.y = pw.y();
                world_pt.z = pw.z();
                world_pt.r = pt.r;
                world_pt.g = pt.g;
                world_pt.b = pt.b;
                world_cloud.points.push_back(world_pt);
            }

            world_cloud.width = world_cloud.points.size();

            if (world_cloud.points.empty())
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "No points after transformation, skipping");
                // Still publish the existing map even if no new points
                publishMap();
                return;
            }

            // Add points to global map, filtering out points that are too close to existing ones
            if (kdtree_->getInputCloud() == nullptr || kdtree_->getInputCloud()->points.empty())
            {
                // First time: rebuild KD-tree with current map
                if (!global_map_->points.empty())
                {
                    kdtree_->setInputCloud(global_map_);
                }
            }

            // Filter new points: only add if not too close to existing points
            size_t added_count = 0;
            for (const auto &new_pt : world_cloud.points)
            {
                bool should_add = true;

                // Check if KD-tree is initialized and has points
                if (kdtree_->getInputCloud() != nullptr && !kdtree_->getInputCloud()->points.empty())
                {
                    std::vector<int> point_idx_search(1);
                    std::vector<float> point_squared_distance(1);

                    // Search for nearest neighbor within min_point_distance
                    if (kdtree_->nearestKSearch(new_pt, 1, point_idx_search, point_squared_distance) > 0)
                    {
                        float dist = std::sqrt(point_squared_distance[0]);
                        if (dist < min_point_distance_)
                        {
                            should_add = false; // Point too close to existing point, skip it
                        }
                    }
                }

                if (should_add)
                {
                    global_map_->points.push_back(new_pt);
                    added_count++;
                }
            }

            // Update KD-tree after adding new points (rebuild periodically for efficiency)
            if (added_count > 0)
            {
                // Update point cloud metadata
                global_map_->width = global_map_->points.size();
                global_map_->height = 1;
                global_map_->is_dense = false;
                kdtree_->setInputCloud(global_map_);
            }

            // Only limit total size to prevent memory issues (keep all accumulated points)
            // const size_t max_pts = 500000; // Very large limit for dense mapping (2M points)
            // if (global_map_->points.size() > max_pts)
            // {
            //     // Instead of trimming old points, just stop adding new ones
            //     // This way the map keeps growing and doesn't erase previous points
            //     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            //                          "Map size limit reached (%zu points), stopping accumulation", max_pts);
            //     return; // Stop adding more points
            // }

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Global map: %zu points | Attempted: %zu | Added: %zu (filtered %zu duplicates)",
                                 global_map_->points.size(), world_cloud.points.size(), added_count,
                                 world_cloud.points.size() - added_count);

            // Publish map and pose
            publishMap();
            publishPose(T_wc);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Published dense map with %zu points", global_map_->points.size());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in dense mapping: %s", e.what());
        }
    }

    void publishMap()
    {
        sensor_msgs::msg::PointCloud2 map_msg;
        // Always publish, even if map is empty (for RViz to see the topic)
        if (global_map_->points.empty())
        {
            // Create empty point cloud message
            pcl::PointCloud<pcl::PointXYZRGB> empty_cloud;
            empty_cloud.header.frame_id = "map";
            empty_cloud.width = 0;
            empty_cloud.height = 1;
            empty_cloud.is_dense = false;
            pcl::toROSMsg(empty_cloud, map_msg);
        }
        else
        {
            // Ensure width and height are correctly set before conversion
            global_map_->width = global_map_->points.size();
            global_map_->height = 1;
            global_map_->is_dense = false;
            global_map_->header.frame_id = "map";
            pcl::toROSMsg(*global_map_, map_msg);
        }
        map_msg.header.frame_id = "map";
        map_msg.header.stamp = this->now();
        map_pub_->publish(map_msg);
    }

    void publishPose(const tf2::Transform &T_wc)
    {
        geometry_msgs::msg::PoseStamped cam_pose;
        cam_pose.header.frame_id = "map";
        cam_pose.header.stamp = this->now();
        cam_pose.pose.position.x = T_wc.getOrigin().x();
        cam_pose.pose.position.y = T_wc.getOrigin().y();
        cam_pose.pose.position.z = T_wc.getOrigin().z();
        tf2::Quaternion qwc = T_wc.getRotation();
        cam_pose.pose.orientation.x = qwc.x();
        cam_pose.pose.orientation.y = qwc.y();
        cam_pose.pose.orientation.z = qwc.z();
        cam_pose.pose.orientation.w = qwc.w();
        pose_pub_->publish(cam_pose);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DenseMapperNode>());
    rclcpp::shutdown();
    return 0;
}
