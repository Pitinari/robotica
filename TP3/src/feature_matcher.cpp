#include "stereo_vision/feature_matcher.hpp"
#include <iostream>
#include <numeric>

namespace stereo_vision
{

    FeatureMatcher::FeatureMatcher(MatcherType type) : matcher_type_(type)
    {
        initializeMatcher();
    }

    void FeatureMatcher::initializeMatcher(int norm_type)
    {
        if (matcher_type_ == MatcherType::BRUTE_FORCE)
        {
            matcher_ = cv::BFMatcher::create(norm_type, true); // crossCheck enabled
        }
        else
        {
            matcher_ = cv::FlannBasedMatcher::create();
        }
    }

    void FeatureMatcher::match(const cv::Mat &descriptors1,
                               const cv::Mat &descriptors2,
                               std::vector<cv::DMatch> &matches)
    {
        if (!matcher_ || descriptors1.empty() || descriptors2.empty())
        {
            std::cerr << "Matcher not initialized or empty descriptors!" << std::endl;
            return;
        }

        matcher_->match(descriptors1, descriptors2, matches);
        std::cout << "Found " << matches.size() << " matches" << std::endl;
    }

    void FeatureMatcher::matchWithThreshold(const cv::Mat &descriptors1,
                                            const cv::Mat &descriptors2,
                                            std::vector<cv::DMatch> &matches,
                                            float max_distance)
    {
        std::vector<cv::DMatch> all_matches;
        match(descriptors1, descriptors2, all_matches);

        // Filter by distance threshold
        matches.clear();
        for (const auto &m : all_matches)
        {
            if (m.distance < max_distance)
            {
                matches.push_back(m);
            }
        }

        std::cout << "Filtered to " << matches.size() << " matches (threshold="
                  << max_distance << ")" << std::endl;
    }

    void FeatureMatcher::matchWithRatioTest(const cv::Mat &descriptors1,
                                            const cv::Mat &descriptors2,
                                            std::vector<cv::DMatch> &matches,
                                            float ratio)
    {
        if (!matcher_ || descriptors1.empty() || descriptors2.empty())
        {
            std::cerr << "Matcher not initialized or empty descriptors!" << std::endl;
            return;
        }

        // Use knnMatch for ratio test
        std::vector<std::vector<cv::DMatch>> knn_matches;
        auto temp_matcher = cv::BFMatcher::create(cv::NORM_HAMMING, false);
        temp_matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);

        // Apply Lowe's ratio test
        matches.clear();
        for (const auto &m : knn_matches)
        {
            if (m.size() == 2 && m[0].distance < ratio * m[1].distance)
            {
                matches.push_back(m[0]);
            }
        }

        std::cout << "Ratio test: " << matches.size() << " good matches" << std::endl;
    }

    void FeatureMatcher::filterMatchesRANSAC(const std::vector<cv::KeyPoint> &keypoints1,
                                             const std::vector<cv::KeyPoint> &keypoints2,
                                             std::vector<cv::DMatch> &matches,
                                             cv::Mat &homography,
                                             std::vector<uchar> &inlier_mask)
    {
        if (matches.size() < 4)
        {
            std::cerr << "Not enough matches for RANSAC (need at least 4)" << std::endl;
            return;
        }

        // Extract matched points
        std::vector<cv::Point2f> points1, points2;
        for (const auto &match : matches)
        {
            points1.push_back(keypoints1[match.queryIdx].pt);
            points2.push_back(keypoints2[match.trainIdx].pt);
        }

        // Find homography with RANSAC
        homography = cv::findHomography(points1, points2, cv::RANSAC, 3.0, inlier_mask);

        // Count inliers
        int inlier_count = std::accumulate(inlier_mask.begin(), inlier_mask.end(), 0);
        std::cout << "RANSAC: " << inlier_count << " inliers out of "
                  << matches.size() << " matches ("
                  << (100.0 * inlier_count / matches.size()) << "%)" << std::endl;

        // Filter matches to keep only inliers
        std::vector<cv::DMatch> inlier_matches;
        for (size_t i = 0; i < matches.size(); ++i)
        {
            if (inlier_mask[i])
            {
                inlier_matches.push_back(matches[i]);
            }
        }
        matches = inlier_matches;
    }

    cv::Mat FeatureMatcher::visualizeMatches(const cv::Mat &img1,
                                             const std::vector<cv::KeyPoint> &keypoints1,
                                             const cv::Mat &img2,
                                             const std::vector<cv::KeyPoint> &keypoints2,
                                             const std::vector<cv::DMatch> &matches,
                                             const std::vector<uchar> &mask) const
    {
        cv::Mat output;

        if (mask.empty())
        {
            cv::drawMatches(img1, keypoints1, img2, keypoints2, matches, output,
                            cv::Scalar::all(-1), cv::Scalar::all(-1),
                            std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        }
        else
        {
            std::vector<char> match_mask(mask.begin(), mask.end());
            cv::drawMatches(img1, keypoints1, img2, keypoints2, matches, output,
                            cv::Scalar::all(-1), cv::Scalar::all(-1),
                            match_mask, cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        }

        return output;
    }

    void FeatureMatcher::getMatchStatistics(const std::vector<cv::DMatch> &matches,
                                            float &mean_distance,
                                            float &std_distance,
                                            float &min_distance,
                                            float &max_distance) const
    {
        if (matches.empty())
        {
            mean_distance = std_distance = min_distance = max_distance = 0.0f;
            return;
        }

        min_distance = matches[0].distance;
        max_distance = matches[0].distance;
        double sum = 0.0;

        for (const auto &match : matches)
        {
            sum += match.distance;
            min_distance = std::min(min_distance, match.distance);
            max_distance = std::max(max_distance, match.distance);
        }

        mean_distance = sum / matches.size();

        // Calculate standard deviation
        double var_sum = 0.0;
        for (const auto &match : matches)
        {
            double diff = match.distance - mean_distance;
            var_sum += diff * diff;
        }
        std_distance = std::sqrt(var_sum / matches.size());

        std::cout << "Match statistics - Mean: " << mean_distance
                  << ", Std: " << std_distance
                  << ", Min: " << min_distance
                  << ", Max: " << max_distance << std::endl;
    }

} // namespace stereo_vision
