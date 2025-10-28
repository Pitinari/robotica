#ifndef STEREO_VISION_DENSE_RECONSTRUCTOR_HPP
#define STEREO_VISION_DENSE_RECONSTRUCTOR_HPP

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace stereo_vision
{

    /**
     * @brief Class for dense 3D reconstruction from disparity maps
     */
    class DenseReconstructor
    {
    public:
        DenseReconstructor();

        /**
         * @brief Initialize with Q matrix from stereo rectification
         */
        void initialize(const cv::Mat &Q);

        /**
         * @brief Reconstruct 3D points from disparity map
         */
        void reconstruct(const cv::Mat &disparity,
                         const cv::Mat &left_image,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

        /**
         * @brief Reconstruct without color information
         */
        void reconstruct(const cv::Mat &disparity,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

        /**
         * @brief Set depth filtering parameters
         */
        void setDepthLimits(float min_depth = 0.1f, float max_depth = 50.0f);

        /**
         * @brief Enable/disable filtering of invalid points
         */
        void setFilterInvalid(bool filter) { filter_invalid_ = filter; }

        /**
         * @brief Convert disparity to depth map
         */
        void disparityToDepth(const cv::Mat &disparity, cv::Mat &depth);

        bool isInitialized() const { return initialized_; }

    private:
        bool initialized_;
        bool filter_invalid_;
        float min_depth_;
        float max_depth_;
        cv::Mat Q_; // Disparity-to-depth mapping matrix

        bool isValidPoint(const cv::Vec3f &point) const;
    };

} // namespace stereo_vision

#endif // STEREO_VISION_DENSE_RECONSTRUCTOR_HPP
