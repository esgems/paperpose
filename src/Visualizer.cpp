#include "src/include/Visualizer.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

namespace paper_pos
{
    void Visualizer::drawCorners(cv::Mat& image, const std::vector<cv::Point2f>& corners, bool firstOnly)
    {
        if (firstOnly)
        {
            cv::circle(image, corners[0], 12, cv::Scalar(0, 255, 0), cv::FILLED);

            return;
        }

        for (int i = 0; i < corners.size(); ++i)
        {
            cv::circle(image, corners[i], 16, cv::Scalar(i * 50, 0, 255), 2, cv::FILLED);
        }
    }
    
    void Visualizer::drawAxis(cv::Mat& image, const cv::Mat& camMat, const cv::Mat& dist, const cv::Mat& R, const cv::Mat& t)
    {
        cv::Mat rvec;
        cv::Rodrigues(R, rvec);
        cv::drawFrameAxes(image, camMat, dist, rvec, t, 2);
    }
    
    cv::Mat Visualizer::segmentation(const cv::Mat& image, const std::vector<cv::Point2f>& corners)
    {
        std::vector<cv::Point> cornerCenters;

        for (int i = 0; i < corners.size(); ++i)
        {
            cornerCenters.push_back(cv::Point(corners[i].x, corners[i].y));
        }

        cv::Mat segmMask(image.size(), CV_8UC3, cv::Scalar(0));
        cv::drawContours(segmMask, std::vector<std::vector<cv::Point>> { cornerCenters }, -1, cv::Scalar(255, 0, 255), cv::FILLED, cv::LINE_AA);

        cv::Mat outImg;
        cv::addWeighted(image, 0.7, segmMask, 1, 0.0, outImg);

        return outImg.clone();
    }

} // namespace paper_pos
