#include "stereo_vision/utils.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <sys/stat.h>
#include <sys/types.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

namespace stereo_vision
{

    bool loadCameraCalibration(const std::string &yaml_path, CameraCalibration &calib)
    {
        try
        {
            // Use YAML-CPP to parse the ROS camera_info format
            YAML::Node config = YAML::LoadFile(yaml_path);

            // Load camera matrix
            std::vector<double> cam_data = config["camera_matrix"]["data"].as<std::vector<double>>();
            calib.camera_matrix = cv::Mat(3, 3, CV_64F, cam_data.data()).clone();

            // Load distortion coefficients
            std::vector<double> dist_data = config["distortion_coefficients"]["data"].as<std::vector<double>>();
            calib.distortion_coeffs = cv::Mat(dist_data.size(), 1, CV_64F, dist_data.data()).clone();

            // Load image size
            int width = config["image_width"].as<int>();
            int height = config["image_height"].as<int>();
            calib.image_size = cv::Size(width, height);

            // Load camera model
            if (config["camera_model"])
            {
                calib.camera_model = config["camera_model"].as<std::string>();
            }
            else
            {
                calib.camera_model = "unknown";
            }

            // Load projection matrix if available (for rectified cameras)
            if (config["projection_matrix"])
            {
                std::vector<double> proj_data = config["projection_matrix"]["data"].as<std::vector<double>>();
                calib.projection_matrix = cv::Mat(3, 4, CV_64F, proj_data.data()).clone();
            }
            else
            {
                // Create identity projection matrix if not available
                calib.projection_matrix = cv::Mat::eye(3, 4, CV_64F);
            }

            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error loading camera calibration from " << yaml_path << ": " << e.what() << std::endl;
            return false;
        }
    }

    bool loadStereoCalibration(const std::string &left_yaml,
                               const std::string &right_yaml,
                               const std::string &extrinsics_yaml,
                               StereoCalibration &stereo_calib)
    {
        // Load left camera
        if (!loadCameraCalibration(left_yaml, stereo_calib.left))
        {
            return false;
        }

        // Load right camera
        if (!loadCameraCalibration(right_yaml, stereo_calib.right))
        {
            return false;
        }

        // Load extrinsics using YAML-CPP
        YAML::Node extrinsics = YAML::LoadFile(extrinsics_yaml);

        std::vector<double> R_data = extrinsics["R"]["data"].as<std::vector<double>>();
        stereo_calib.R = cv::Mat(3, 3, CV_64F, R_data.data()).clone();

        std::vector<double> T_data = extrinsics["T"]["data"].as<std::vector<double>>();
        stereo_calib.T = cv::Mat(3, 1, CV_64F, T_data.data()).clone();

        if (extrinsics["baseline"])
        {
            stereo_calib.baseline = extrinsics["baseline"].as<double>();
        }
        else
        {
            // Compute baseline from translation
            stereo_calib.baseline = cv::norm(stereo_calib.T);
        }

        return true;
    }

    std::vector<GroundTruthPose> readGroundTruthPoses(const std::string &csv_path)
    {
        std::vector<GroundTruthPose> poses;
        std::ifstream file(csv_path);

        if (!file.is_open())
        {
            std::cerr << "Failed to open ground truth file: " << csv_path << std::endl;
            return poses;
        }

        std::string line;
        // Skip header lines starting with #
        while (std::getline(file, line))
        {
            if (line[0] != '#')
            {
                // Put the line back
                file.seekg(-line.length() - 1, std::ios_base::cur);
                break;
            }
        }

        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string token;
            std::vector<double> values;

            while (std::getline(ss, token, ','))
            {
                values.push_back(std::stod(token));
            }

            if (values.size() >= 8)
            {
                GroundTruthPose pose;
                pose.timestamp = values[0] / 1e9; // Convert nanoseconds to seconds

                // Position: tx, ty, tz
                pose.position = Eigen::Vector3d(values[1], values[2], values[3]);

                // Quaternion: qw, qx, qy, qz
                pose.orientation = Eigen::Quaterniond(values[4], values[5], values[6], values[7]);

                // Build transformation matrix
                pose.transformation_matrix = Eigen::Matrix4d::Identity();
                pose.transformation_matrix.block<3, 3>(0, 0) = pose.orientation.toRotationMatrix();
                pose.transformation_matrix.block<3, 1>(0, 3) = pose.position;

                poses.push_back(pose);
            }
        }

        file.close();
        return poses;
    }

    void eigenToCV(const Eigen::Matrix4d &transform, cv::Mat &R, cv::Mat &t)
    {
        R = cv::Mat::eye(3, 3, CV_64F);
        t = cv::Mat::zeros(3, 1, CV_64F);

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                R.at<double>(i, j) = transform(i, j);
            }
            t.at<double>(i) = transform(i, 3);
        }
    }

    Eigen::Matrix4d cvToEigen(const cv::Mat &R, const cv::Mat &t)
    {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                transform(i, j) = R.at<double>(i, j);
            }
            transform(i, 3) = t.at<double>(i);
        }

        return transform;
    }

    void ensureOutputDirectory(const std::string &path)
    {
        struct stat info;
        if (stat(path.c_str(), &info) != 0)
        {
            // Directory doesn't exist, create it
            mkdir(path.c_str(), 0755);
        }
    }

    void saveImage(const cv::Mat &image, const std::string &prefix, const std::string &output_dir)
    {
        ensureOutputDirectory(output_dir);

        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                             now.time_since_epoch())
                             .count();

        std::stringstream filename;
        filename << output_dir << "/" << prefix << "_" << timestamp << ".png";

        cv::imwrite(filename.str(), image);
        std::cout << "Saved image: " << filename.str() << std::endl;
    }

    cv::Mat drawKeypoints(const cv::Mat &image, const std::vector<cv::KeyPoint> &keypoints)
    {
        cv::Mat output;
        cv::drawKeypoints(image, keypoints, output, cv::Scalar(0, 255, 0),
                          cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        return output;
    }

    cv::Mat drawMatches(const cv::Mat &img1, const std::vector<cv::KeyPoint> &kp1,
                        const cv::Mat &img2, const std::vector<cv::KeyPoint> &kp2,
                        const std::vector<cv::DMatch> &matches)
    {
        cv::Mat output;
        cv::drawMatches(img1, kp1, img2, kp2, matches, output,
                        cv::Scalar::all(-1), cv::Scalar::all(-1),
                        std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        return output;
    }

} // namespace stereo_vision
