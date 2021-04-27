#include "src/include/PaperFinder.h"
#include "src/include/PositionEstimator.h"
#include "src/include/Visualizer.h"

#include <iostream>
#include <vector>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace paper_pos;

const float DEFAULT_FX = 500.0;
const int DEFAULT_IMAGE_WIDTH = 640;

void resizeImage(cv::Mat& image, int width)
{
    double height = std::min(width, image.rows);
    cv::Size imageSize(int(height / image.rows * image.cols), height);
    cv::resize(image, image, imageSize);
}

int main(int argc, char* argv[])
{
    double fx = -1.0f;
    std::string path = "./data/0_Color.png";
    
    if (argc > 1)
    {
        for(int i = 1; i < argc; ++i)
        {
            if (i >= argc - 1)
            {
                break;
            }

            if (strcmp(argv[i], "-f") == 0)
            {
                fx = std::stof(argv[i + 1]);
            }
            else if (strcmp(argv[i], "-image") == 0)
            {
                path = argv[i + 1];
            }
            
        }
    }

    std::cout << "path: " << path << std::endl;
    std::cout << "f: " << fx << std::endl;

    cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);

    if (image.cols == 0)
    {
        std::cout << "image not found: " << path << std::endl;
        return -1;
    }

    if (fx < 0)
    {
        resizeImage(image, DEFAULT_IMAGE_WIDTH);

        fx = DEFAULT_FX;
    }

    PaperFinder pf;
    std::vector<cv::Point2f> corners;

    if (!pf.findCorners(image, corners) || !pf.normalizeCorners(corners))
    {
        std::cerr << "findCorners error\n";
        return -1;
    }

    std::vector<cv::Point2f> worldPoints2d = pf.getWorldPoints();

    std::vector<double> K_vec = {fx, 0, image.cols / 2.0f, 
                                0, fx, image.rows / 2.0f, 
                                0, 0, 1};
    cv::Mat K_mat(3, 3, CV_64FC1, K_vec.data());
    cv::Mat dist(1, 4, CV_64FC1, cv::Scalar(0));

    cv::Mat R;
    cv::Mat t;

    PositionEstimator pe;
    pe.calcPosition(corners, worldPoints2d, K_mat, dist, R, t);
    cv::Vec3d angles = pe.rotationMatrixToEulerAngles(R, true);
    std::cout << "R: " << angles << std::endl;
    std::cout << "T: " << t.t() << std::endl;

    // cv out
    Visualizer viz;
    viz.drawCorners(image, corners, true);
    // viz.drawAxis(image, K_mat, dist, R, t);
    cv::Mat segmImage = viz.segmentation(image, corners);

    resizeImage(segmImage, DEFAULT_IMAGE_WIDTH);
    cv::imshow("segmImage", segmImage);
    cv::waitKey(0);

    return -1;
}
