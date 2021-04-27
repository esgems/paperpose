#pragma once

#include <opencv2/core/core.hpp>

#include <vector>

namespace paper_pos
{
    class PaperFinder
    {
        public:
            bool findCorners(const cv::Mat& image, std::vector<cv::Point2f>& outCorners);
            std::vector<cv::Point2f> getWorldPoints();
            bool normalizeCorners(std::vector<cv::Point2f>& corners);
    };
}