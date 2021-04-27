#include "src/include/PaperFinder.h"

#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

namespace paper_pos
{
    double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0 )
    {
        double dx1 = pt1.x - pt0.x;
        double dy1 = pt1.y - pt0.y;
        double dx2 = pt2.x - pt0.x;
        double dy2 = pt2.y - pt0.y;

        return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
    }

    bool PaperFinder::findCorners(const cv::Mat& image, std::vector<cv::Point2f>& outCorners)
    {
        std::vector<cv::Point2f> corners(4);

        cv::Mat blurred(image);
        // cv::medianBlur(image, blurred, 9);
        cv::GaussianBlur(image, blurred, cv::Size(3, 3), 1.0);
        cv::GaussianBlur(blurred, blurred, cv::Size(3, 3), 2.0);
        cv::GaussianBlur(blurred, blurred, cv::Size(3, 3), 3.0);

        cv::Mat gray0(blurred.size(), CV_8U), gray;
        std::vector<std::vector<cv::Point> > contours;

        float minCos = MAXFLOAT;
        float maxArea = 100;

        for (int c = 0; c < 3; c++)
        {
            int ch[] = {c, 0};
            cv::mixChannels(&blurred, 1, &gray0, 1, ch, 1);

            cv::Canny(gray0, gray, 10, 20, 3);
            cv::findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

            std::vector<cv::Point> approx;
        
            for (size_t i = 0; i < contours.size(); i++)
            {
                float per = cv::arcLength(cv::Mat(contours[i]), true);
                cv::approxPolyDP(cv::Mat(contours[i]), approx, per * 0.02, true);

                auto area = cv::contourArea(cv::Mat(approx));

                if (approx.size() == 4 && fabs(area) > maxArea && cv::isContourConvex(approx))
                {
                    double cosineSum = 0;
                    for( int j = 2; j < 5; j++ )
                    {
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        cosineSum += cosine;
                    }

                    // if (cosineSum < minCos)
                    {
                        minCos = cosineSum;

                        for (int j = 0; j < 4; ++j)
                            corners[j] = cv::Point2f(approx[j].x, approx[j].y);

                        maxArea = area;
                    }
                }
            }
        }

        bool found = maxArea > 100;

        if (found)
        {
            outCorners = corners;
        }

        return found;
    }

    std::vector<cv::Point2f> PaperFinder::getWorldPoints()
    {
        float cx = 0;
        float cy = 0;
        float pw = cx + 0.210f;
        float ph = cy + 0.297f;

        std::vector<cv::Point2f> corners2_h(4);
        corners2_h[0] = cv::Point2f(cx, cy);
        corners2_h[1] = cv::Point2f(cx, ph);
        corners2_h[2] = cv::Point2f(pw, ph);
        corners2_h[3] = cv::Point2f(pw, cy);

        return corners2_h;
    }

    /*
    first corner is always top left
    and normal paper orientation is vertical
    */
    bool PaperFinder::normalizeCorners(std::vector<cv::Point2f>& corners)
    {
        std::vector<cv::Point2f> cornersNew;

        if (corners[0].y * 1.2 < corners[1].y && corners[1].x * 1.2 < corners[2].x && corners[2].y > corners[3].y * 1.2 && corners[3].x > corners[0].x * 1.2)
        {
            std::cout << "top left start\n";
            cornersNew.push_back(corners[0]);
            cornersNew.push_back(corners[1]);
            cornersNew.push_back(corners[2]);
            cornersNew.push_back(corners[3]);
        }
        else if (corners[0].x * 1.2 < corners[1].x && corners[1].y > corners[2].y * 1.2 && corners[2].x > corners[3].x * 1.2 && corners[3].y * 1.2 < corners[0].y)
        {
            std::cout << "bottom left start\n";
            cornersNew.push_back(corners[3]);
            cornersNew.push_back(corners[0]);
            cornersNew.push_back(corners[1]);
            cornersNew.push_back(corners[2]);
        }
        else if (corners[0].y > corners[1].y * 1.2 && corners[1].x > corners[2].x * 1.2 && corners[2].y * 1.2 < corners[3].y && corners[3].x * 1.2 < corners[0].x)
        {
            std::cout << "bottom right start\n";
            cornersNew.push_back(corners[2]);
            cornersNew.push_back(corners[3]);
            cornersNew.push_back(corners[0]);
            cornersNew.push_back(corners[1]);
        }
        else if (corners[0].x > corners[1].x * 1.2 && corners[1].y * 1.2 < corners[2].y && corners[2].x * 1.2 < corners[3].x && corners[3].y > corners[0].y * 1.2)
        {
            std::cout << "top right start\n";
            cornersNew.push_back(corners[1]);
            cornersNew.push_back(corners[2]);
            cornersNew.push_back(corners[3]);
            cornersNew.push_back(corners[0]);
        }
        else
        {
            std::cout << corners[0] << "\n";
            std::cout << corners[1] << "\n";
            std::cout << corners[2] << "\n";
            std::cout << corners[3] << "\n";
            std::cerr << "ERROR: corners not found\n";

            return false;
        }

        corners = cornersNew;

        auto h1 = (corners[0] - corners[1]); 
        auto h2 = (corners[0] - corners[3]); 
        auto dist1 = sqrt(h1.x * h1.x + h1.y * h1.y);
        auto dist2 = sqrt(h2.x * h2.x + h2.y * h2.y);

        // is horizontal
        if (dist1 < dist2)
        {
            std::cout << "sq_w\n";

            std::vector<cv::Point2f> cornersNew2(4);
            cornersNew2[0] = corners[3];
            cornersNew2[1] = corners[0];
            cornersNew2[2] = corners[1];
            cornersNew2[3] = corners[2];
            corners = cornersNew2;
        }

        return true;
    }
    
} // namespace paper_pos
