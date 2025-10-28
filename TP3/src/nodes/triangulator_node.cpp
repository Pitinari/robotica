#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "stereo_vision/feature_extractor.hpp"
#include "stereo_vision/feature_matcher.hpp"
#include "stereo_vision/triangulator.hpp"
#include "stereo_vision/utils.hpp"

class TriangulatorNode : public rclcpp::Node
{
public:
    TriangulatorNode() : Node("triangulator_node")
    {
        // Declare parameters
        this->declare_parameter("left_calib_file", "config/camera_info_left.yaml");
        this->declare_parameter("right_calib_file", "config/camera_info_right.yaml");
        this->declare_parameter("extrinsics_file", "config/extrinsics.yaml");
        this->declare_parameter("distance_threshold", 30.0);
        this->declare_parameter("max_features", 1000);
        this->declare_parameter("threshold", 10.0);

        // Get parameters
        std::string left_calib_file = this->get_parameter("left_calib_file").as_string();
        std::string right_calib_file = this->get_parameter("right_calib_file").as_string();
        std::string extrinsics_file = this->get_parameter("extrinsics_file").as_string();
        double distance_threshold = this->get_parameter("distance_threshold").as_double();
        int max_features = this->get_parameter("max_features").as_int();
        double threshold = this->get_parameter("threshold").as_double();

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

        // Initialize feature extractor
        extractor_ = std::make_shared<stereo_vision::FeatureExtractor>(
            stereo_vision::DetectorType::ORB, stereo_vision::DescriptorType::ORB);
        extractor_->setDetectorParams(max_features, threshold);

        // Initialize feature matcher
        matcher_ = std::make_shared<stereo_vision::FeatureMatcher>();
        distance_threshold_ = distance_threshold;

        // Compute projection matrices for triangulation
        cv::Mat P1, P2;
        computeProjectionMatrices(left_calib, right_calib, stereo_calib, P1, P2);

        // Initialize triangulator
        triangulator_ = std::make_shared<stereo_vision::Triangulator>();
        triangulator_->initialize(P1, P2);

        // Create subscribers for rectified images
        left_sub_.subscribe(this, "/stereo/left/rect");
        right_sub_.subscribe(this, "/stereo/right/rect");

        // Create synchronizer
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), left_sub_, right_sub_);
        sync_->registerCallback(&TriangulatorNode::callback, this);

        // Create publishers
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/triangulation/point_cloud", 10);
        matches_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/triangulation/matches", 10);

        RCLCPP_INFO(this->get_logger(), "Triangulator node initialized");
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

            // Match features and apply RANSAC filtering
            std::vector<cv::DMatch> all_matches, matches;
            matcher_->match(left_descriptors, right_descriptors, all_matches);

            if (all_matches.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No matches found for triangulation");
                return;
            }

            // Apply RANSAC filtering
            cv::Mat homography;
            std::vector<uchar> inlier_mask;
            matches = all_matches; // Copy for RANSAC filtering
            matcher_->filterMatchesRANSAC(left_keypoints, right_keypoints, matches, homography, inlier_mask);

            if (matches.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No RANSAC inliers found for triangulation");
                return;
            }

            // Triangulate 3D points
            std::vector<cv::Point3f> points3d;
            triangulator_->triangulate(left_keypoints, right_keypoints, matches, points3d);

            if (points3d.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No 3D points triangulated");
                return;
            }

            // Filter points by depth to remove outliers (relaxed range)
            std::vector<int> valid_indices;
            triangulator_->filterByDepth(points3d, valid_indices, 0.1, 200.0); // 10cm to 200m range

            if (valid_indices.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No valid 3D points after depth filtering");
                return;
            }

            // Keep only valid points
            std::vector<cv::Point3f> filtered_points3d;
            std::vector<cv::DMatch> filtered_matches;
            for (int idx : valid_indices)
            {
                filtered_points3d.push_back(points3d[idx]);
                filtered_matches.push_back(matches[idx]);
            }
            points3d = filtered_points3d;
            matches = filtered_matches;

            // Create point cloud
            pcl::PointCloud<pcl::PointXYZRGB> cloud;
            cloud.header.frame_id = "cam0";
            cloud.width = points3d.size();
            cloud.height = 1;
            cloud.is_dense = false;
            cloud.points.resize(points3d.size());

            // Convert 3D points to PCL format with colors
            // Use camera coordinate system directly (no transformation)
            for (size_t i = 0; i < points3d.size(); ++i)
            {
                const cv::Point3f &pt = points3d[i];
                // Use camera coordinate system: X-right, Y-down, Z-forward
                cloud.points[i].x = pt.x; // X (right)
                cloud.points[i].y = pt.y; // Y (down)
                cloud.points[i].z = pt.z; // Z (forward)

                // Get color from left image
                const cv::KeyPoint &kp = left_keypoints[matches[i].queryIdx];
                int x = static_cast<int>(kp.pt.x);
                int y = static_cast<int>(kp.pt.y);

                if (x >= 0 && x < left_ptr->image.cols && y >= 0 && y < left_ptr->image.rows)
                {
                    cv::Vec3b color = left_ptr->image.at<cv::Vec3b>(y, x);
                    cloud.points[i].r = color[2];
                    cloud.points[i].g = color[1];
                    cloud.points[i].b = color[0];
                }
                else
                {
                    cloud.points[i].r = 255;
                    cloud.points[i].g = 255;
                    cloud.points[i].b = 255;
                }
            }

            // Publish point cloud
            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(cloud, cloud_msg);
            cloud_msg.header.stamp = left_msg->header.stamp;
            cloud_msg.header.frame_id = "cam0";
            cloud_pub_->publish(cloud_msg);

            // Create matches visualization
            cv::Mat matches_img;
            cv::drawMatches(left_ptr->image, left_keypoints, right_ptr->image, right_keypoints,
                            matches, matches_img, cv::Scalar::all(-1), cv::Scalar::all(-1),
                            std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

            // Add text overlay
            std::string text = "Triangulated: " + std::to_string(points3d.size()) + " points";
            cv::putText(matches_img, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

            // Publish matches visualization
            cv_bridge::CvImage out;
            out.header = left_msg->header;
            out.encoding = "bgr8";
            out.image = matches_img;
            matches_pub_->publish(*out.toImageMsg());

            // Log results
            // Only log every 10th frame to reduce spam
            static int log_counter = 0;
            if (++log_counter % 50 == 0)
            {
                RCLCPP_DEBUG(this->get_logger(), "Triangulated %zu 3D points from %zu matches",
                             points3d.size(), matches.size());
            }
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV error in triangulation: %s", e.what());
        }
    }

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>
        SyncPolicy;
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr matches_pub_;

    std::shared_ptr<stereo_vision::FeatureExtractor> extractor_;
    std::shared_ptr<stereo_vision::FeatureMatcher> matcher_;
    std::shared_ptr<stereo_vision::Triangulator> triangulator_;
    double distance_threshold_;

    void computeProjectionMatrices(const stereo_vision::CameraCalibration &left_calib,
                                   const stereo_vision::CameraCalibration &right_calib,
                                   const stereo_vision::StereoCalibration &stereo_calib,
                                   cv::Mat &P1, cv::Mat &P2)
    {
        // Use pre-computed rectified projection matrices from calibration files
        if (!left_calib.projection_matrix.empty() && !right_calib.projection_matrix.empty())
        {
            P1 = left_calib.projection_matrix.clone();
            P2 = right_calib.projection_matrix.clone();
            RCLCPP_DEBUG(this->get_logger(), "Using pre-computed rectified projection matrices from calibration files");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Projection matrices not found in calibration files, computing from stereo calibration");

            // Compute rectified projection matrices using stereo calibration
            cv::Mat R1, R2, P1_rect, P2_rect, Q;
            cv::stereoRectify(left_calib.camera_matrix, left_calib.distortion_coeffs,
                              right_calib.camera_matrix, right_calib.distortion_coeffs,
                              left_calib.image_size, stereo_calib.R, stereo_calib.T,
                              R1, R2, P1_rect, P2_rect, Q);
            P1 = P1_rect;
            P2 = P2_rect;
        }

        // Log projection matrix information (debug level)
        RCLCPP_DEBUG(this->get_logger(), "P1: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                     P1.at<double>(0, 0), P1.at<double>(1, 1), P1.at<double>(0, 2), P1.at<double>(1, 2));
        RCLCPP_DEBUG(this->get_logger(), "P2: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f, baseline=%.3f",
                     P2.at<double>(0, 0), P2.at<double>(1, 1), P2.at<double>(0, 2), P2.at<double>(1, 2),
                     -P2.at<double>(0, 3) / P2.at<double>(0, 0));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TriangulatorNode>());
    rclcpp::shutdown();
    return 0;
}
