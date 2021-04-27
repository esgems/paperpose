#pragma once

#include <opencv2/core/core.hpp>

#include <vector>

namespace paper_pos
{
    class Visualizer
    {
        public:
            void drawCorners(cv::Mat& image, const std::vector<cv::Point2f>& corners, bool firstOnly = false);
            void drawAxis(cv::Mat& image, const cv::Mat& camMat, const cv::Mat& dist, const cv::Mat& R, const cv::Mat& t);
            cv::Mat segmentation(const cv::Mat& image, const std::vector<cv::Point2f>& corners);
    };
}