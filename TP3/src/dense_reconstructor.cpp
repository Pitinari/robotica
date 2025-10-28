#include "stereo_vision/dense_reconstructor.hpp"
#include <iostream>

namespace stereo_vision
{

    DenseReconstructor::DenseReconstructor()
        : initialized_(false), filter_invalid_(true), min_depth_(0.1f), max_depth_(50.0f) {}

    void DenseReconstructor::initialize(const cv::Mat &Q)
    {
        Q_ = Q.clone();
        initialized_ = true;
        std::cout << "Dense reconstructor initialized" << std::endl;
        std::cout << "Q matrix:\n"
                  << Q_ << std::endl;
    }

    void DenseReconstructor::reconstruct(const cv::Mat &disparity,
                                         const cv::Mat &left_image,
                                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
    {
        if (!initialized_)
        {
            std::cerr << "Dense reconstructor not initialized!" << std::endl;
            return;
        }

        if (disparity.empty() || left_image.empty())
        {
            std::cerr << "Empty disparity or image!" << std::endl;
            return;
        }

        // Reproject to 3D
        cv::Mat points3d;
        cv::reprojectImageTo3D(disparity, points3d, Q_, true);

        // Convert to color image if grayscale
        cv::Mat color_image;
        if (left_image.channels() == 1)
        {
            cv::cvtColor(left_image, color_image, cv::COLOR_GRAY2BGR);
        }
        else
        {
            color_image = left_image;
        }

        // Create point cloud
        cloud->clear();
        cloud->is_dense = false;

        int valid_points = 0;
        for (int y = 0; y < points3d.rows; ++y)
        {
            for (int x = 0; x < points3d.cols; ++x)
            {
                cv::Vec3f point = points3d.at<cv::Vec3f>(y, x);

                if (filter_invalid_ && !isValidPoint(point))
                {
                    continue;
                }

                pcl::PointXYZRGB pcl_point;
                pcl_point.x = point[0];
                pcl_point.y = point[1];
                pcl_point.z = point[2];

                // Add color
                cv::Vec3b color = color_image.at<cv::Vec3b>(y, x);
                pcl_point.r = color[2];
                pcl_point.g = color[1];
                pcl_point.b = color[0];

                cloud->push_back(pcl_point);
                valid_points++;
            }
        }

        cloud->width = valid_points;
        cloud->height = 1;

        std::cout << "Reconstructed dense point cloud with " << valid_points << " points" << std::endl;
    }

    void DenseReconstructor::reconstruct(const cv::Mat &disparity,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        if (!initialized_)
        {
            std::cerr << "Dense reconstructor not initialized!" << std::endl;
            return;
        }

        if (disparity.empty())
        {
            std::cerr << "Empty disparity!" << std::endl;
            return;
        }

        // Reproject to 3D
        cv::Mat points3d;
        cv::reprojectImageTo3D(disparity, points3d, Q_, true);

        // Create point cloud
        cloud->clear();
        cloud->is_dense = false;

        int valid_points = 0;
        for (int y = 0; y < points3d.rows; ++y)
        {
            for (int x = 0; x < points3d.cols; ++x)
            {
                cv::Vec3f point = points3d.at<cv::Vec3f>(y, x);

                if (filter_invalid_ && !isValidPoint(point))
                {
                    continue;
                }

                pcl::PointXYZ pcl_point;
                pcl_point.x = point[0];
                pcl_point.y = point[1];
                pcl_point.z = point[2];

                cloud->push_back(pcl_point);
                valid_points++;
            }
        }

        cloud->width = valid_points;
        cloud->height = 1;

        std::cout << "Reconstructed dense point cloud with " << valid_points << " points" << std::endl;
    }

    void DenseReconstructor::setDepthLimits(float min_depth, float max_depth)
    {
        min_depth_ = min_depth;
        max_depth_ = max_depth;
        std::cout << "Depth limits set: [" << min_depth_ << ", " << max_depth_ << "]" << std::endl;
    }

    void DenseReconstructor::disparityToDepth(const cv::Mat &disparity, cv::Mat &depth)
    {
        if (!initialized_)
        {
            std::cerr << "Dense reconstructor not initialized!" << std::endl;
            return;
        }

        // Extract focal length and baseline from Q matrix
        // Q(3,2) contains -1/Tx where Tx is the baseline
        // Q(2,3) contains focal length
        float focal_length = Q_.at<double>(2, 3);
        float baseline = -1.0 / Q_.at<double>(3, 2);

        depth = cv::Mat::zeros(disparity.size(), CV_32F);

        for (int y = 0; y < disparity.rows; ++y)
        {
            for (int x = 0; x < disparity.cols; ++x)
            {
                float disp = disparity.at<float>(y, x);
                if (disp > 0)
                {
                    depth.at<float>(y, x) = (focal_length * baseline) / disp;
                }
            }
        }
    }

    bool DenseReconstructor::isValidPoint(const cv::Vec3f &point) const
    {
        // Check for NaN or Inf
        if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || !std::isfinite(point[2]))
        {
            return false;
        }

        // Check depth bounds
        float depth = point[2];
        if (depth < min_depth_ || depth > max_depth_)
        {
            return false;
        }

        return true;
    }

} // namespace stereo_vision
