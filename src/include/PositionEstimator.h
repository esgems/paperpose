#pragma once

#include <opencv2/core/core.hpp>

#include <vector>

namespace paper_pos
{
    class PositionEstimator
    {
        public:
            void calcPosition(const std::vector<cv::Point2f>& corners, const std::vector<cv::Point2f>& points, const cv::Mat& camMat, const cv::Mat& dist, cv::Mat& outR, cv::Mat& outT);
            cv::Vec3d rotationMatrixToEulerAngles(const cv::Mat &R, bool inDegrees);
    };
}