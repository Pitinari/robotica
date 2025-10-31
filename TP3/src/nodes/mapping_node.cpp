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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/core.hpp>

#include "stereo_vision/feature_extractor.hpp"
#include "stereo_vision/feature_matcher.hpp"
#include "stereo_vision/triangulator.hpp"
#include "stereo_vision/utils.hpp"

class MappingNode : public rclcpp::Node
{
public:
    MappingNode() : Node("mapping_node")
    {
        // Parameters
        this->declare_parameter("left_calib_file", "config/camera_info_left.yaml");
        this->declare_parameter("right_calib_file", "config/camera_info_right.yaml");
        this->declare_parameter("extrinsics_file", "config/extrinsics.yaml");
        this->declare_parameter("distance_threshold", 50.0);
        this->declare_parameter("max_features", 1000);
        this->declare_parameter("threshold", 10.0);

        const auto left_calib_file = this->get_parameter("left_calib_file").as_string();
        const auto right_calib_file = this->get_parameter("right_calib_file").as_string();
        const auto extrinsics_file = this->get_parameter("extrinsics_file").as_string();
        distance_threshold_ = this->get_parameter("distance_threshold").as_double();
        max_features_ = this->get_parameter("max_features").as_int();
        threshold_ = this->get_parameter("threshold").as_double();

        // Load calibration
        if (!stereo_vision::loadCameraCalibration(left_calib_file, left_calib_) ||
            !stereo_vision::loadCameraCalibration(right_calib_file, right_calib_) ||
            !stereo_vision::loadStereoCalibration(left_calib_file, right_calib_file, extrinsics_file, stereo_calib_))
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

        // Components
        extractor_ = std::make_unique<stereo_vision::FeatureExtractor>();
        extractor_->setDetectorParams(max_features_, threshold_);
        matcher_ = std::make_unique<stereo_vision::FeatureMatcher>();
        triangulator_ = std::make_unique<stereo_vision::Triangulator>();

        computeProjectionMatrices();
        triangulator_->initialize(P1_, P2_);

        // Subscribers
        left_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/stereo/left/rect");
        right_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/stereo/right/rect");
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu0", 10, std::bind(&MappingNode::imuCb, this, std::placeholders::_1));
        gt_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/leica/position", 10, std::bind(&MappingNode::gtCb, this, std::placeholders::_1));

        using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
        sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *left_sub_, *right_sub_);
        sync_->registerCallback(std::bind(&MappingNode::callback, this, std::placeholders::_1, std::placeholders::_2));

        // Publishers (reliable + transient so RViz sees last map)
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.reliable();
        qos.transient_local();
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/mapping/point_cloud", qos);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mapping/camera_pose", 10);

        global_map_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        global_map_->header.frame_id = "map";

        RCLCPP_INFO(this->get_logger(), "MappingNode started (GT localization)");
    }

private:
    // Params
    double distance_threshold_{};
    int max_features_{};
    double threshold_{};

    // Calib
    stereo_vision::CameraCalibration left_calib_, right_calib_;
    stereo_vision::StereoCalibration stereo_calib_;
    cv::Mat P1_, P2_;

    // Transforms
    cv::Mat T_imu_cam0_;
    geometry_msgs::msg::PoseStamped last_imu_pose_{};
    bool have_imu_{false};
    geometry_msgs::msg::PointStamped last_gt_pos_{};
    bool have_gt_{false};

    // Components
    std::unique_ptr<stereo_vision::FeatureExtractor> extractor_;
    std::unique_ptr<stereo_vision::FeatureMatcher> matcher_;
    std::unique_ptr<stereo_vision::Triangulator> triangulator_;

    // IO
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub_;
    std::unique_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> sync_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr gt_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    // Map
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_;

    void computeProjectionMatrices()
    {
        if (!left_calib_.projection_matrix.empty() && !right_calib_.projection_matrix.empty())
        {
            P1_ = left_calib_.projection_matrix.clone();
            P2_ = right_calib_.projection_matrix.clone();
            return;
        }
        cv::Mat R1, R2, P1, P2, Q;
        cv::stereoRectify(left_calib_.camera_matrix, left_calib_.distortion_coeffs,
                          right_calib_.camera_matrix, right_calib_.distortion_coeffs,
                          left_calib_.image_size, stereo_calib_.R, stereo_calib_.T,
                          R1, R2, P1, P2, Q);
        P1_ = P1.clone();
        P2_ = P2.clone();
    }

    void imuCb(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!msg)
            return;
        last_imu_pose_.header = msg->header;
        last_imu_pose_.pose.orientation = msg->orientation;
        have_imu_ = true;
    }

    void gtCb(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        if (!msg)
            return;
        last_gt_pos_ = *msg;
        have_gt_ = true;
    }

    void callback(const sensor_msgs::msg::Image::ConstSharedPtr left_msg,
                  const sensor_msgs::msg::Image::ConstSharedPtr right_msg)
    {
        // Convert
        cv_bridge::CvImageConstPtr left_ptr, right_ptr;
        try
        {
            left_ptr = cv_bridge::toCvShare(left_msg, "bgr8");
            right_ptr = cv_bridge::toCvShare(right_msg, "bgr8");
        }
        catch (...)
        {
            return;
        }

        // Features
        std::vector<cv::KeyPoint> kpL, kpR;
        cv::Mat descL, descR;
        extractor_->extract(left_ptr->image, kpL, descL);
        extractor_->extract(right_ptr->image, kpR, descR);
        if (kpL.empty() || kpR.empty())
            return;

        // Matching + RANSAC
        std::vector<cv::DMatch> matches, inliers;
        matcher_->matchWithThreshold(descL, descR, matches, threshold_);
        if (matches.empty())
            return;
        cv::Mat H;
        std::vector<unsigned char> inlier_mask;
        inliers = matches; // pass container
        matcher_->filterMatchesRANSAC(kpL, kpR, inliers, H, inlier_mask);
        if (inliers.empty())
            return;

        // Triangulate
        std::vector<cv::Point3f> pts3;
        triangulator_->triangulate(kpL, kpR, inliers, pts3);
        if (pts3.empty())
            return;

        // Depth filter
        std::vector<int> keep;
        triangulator_->filterByDepth(pts3, keep, 0.1f, static_cast<float>(distance_threshold_));
        if (keep.empty())
            return;

        // Build world transform T_world_cam = T_world_imu * T_imu_cam0
        if (!(have_imu_ && have_gt_))
            return;
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

        // Local cloud
        pcl::PointCloud<pcl::PointXYZRGB> local;
        local.header.frame_id = "map";
        local.width = keep.size();
        local.height = 1;
        local.is_dense = false;
        local.points.reserve(keep.size());
        for (int idx : keep)
        {
            if (idx < 0 || idx >= static_cast<int>(pts3.size()))
                continue;
            const auto &p = pts3[idx];
            // Flip image Y (OpenCV image coords are down-positive)
            tf2::Vector3 pc(p.x, -p.y, p.z);
            tf2::Vector3 pw = T_wc * pc;
            pcl::PointXYZRGB pt;
            pt.x = pw.x();
            pt.y = pw.y();
            pt.z = pw.z();
            // color from left
            const auto &kp = kpL[inliers[idx].queryIdx];
            int x = std::max(0, std::min((int)kp.pt.x, left_ptr->image.cols - 1));
            int y = std::max(0, std::min((int)kp.pt.y, left_ptr->image.rows - 1));
            cv::Vec3b c = left_ptr->image.at<cv::Vec3b>(y, x);
            pt.r = c[2];
            pt.g = c[1];
            pt.b = c[0];
            local.points.push_back(pt);
        }
        if (local.points.empty())
            return;

        // Accumulate with trim
        *global_map_ += local;
        const size_t max_pts = 100000;
        if (global_map_->points.size() > max_pts)
        {
            pcl::PointCloud<pcl::PointXYZRGB> trimmed;
            trimmed.points.reserve(max_pts);
            size_t start = global_map_->points.size() - max_pts;
            for (size_t i = start; i < global_map_->points.size(); ++i)
                trimmed.points.push_back(global_map_->points[i]);
            trimmed.width = trimmed.points.size();
            trimmed.height = 1;
            trimmed.is_dense = false;
            trimmed.header = global_map_->header;
            *global_map_ = trimmed;
        }

        // Publish map
        geometry_msgs::msg::PoseStamped cam_pose;
        cam_pose.header.frame_id = "map";
        cam_pose.header.stamp = this->now();
        cam_pose.pose.position.x = T_wc.getOrigin().x();
        cam_pose.pose.position.y = T_wc.getOrigin().y();
        cam_pose.pose.position.z = T_wc.getOrigin().z();
        tf2::Quaternion qwc = T_wc.getRotation();
        cam_pose.pose.orientation = tf2::toMsg(qwc);
        pose_pub_->publish(cam_pose);

        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*global_map_, map_msg);
        map_msg.header.frame_id = "map";
        map_msg.header.stamp = this->now();
        map_pub_->publish(map_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MappingNode>());
    rclcpp::shutdown();
    return 0;
}
