#include "src/include/PositionEstimator.h"

#include <opencv2/calib3d.hpp>

#include <iostream>

namespace paper_pos
{
    double rad2deg(double rad)
    {
        return rad * 180.0 / M_PI;    
    }

    void decompositionH(cv::Mat K, cv::Mat H, cv::Mat& R, cv::Mat& t)
    {
        cv::Mat h1 = H.col(0);
        cv::Mat h2 = H.col(1);
        cv::Mat h3 = H.col(2);

        cv::Mat invK = K.inv();

        double L = 1.0f / cv::norm(invK * h1);
        cv::Mat r1 = (L * (invK * h1)).t();
        cv::Mat r2 = (L * (invK * h2)).t();
        cv::Mat r3 = (r1.cross(r2));

        t = L * (invK * h3);

        cv::Mat R_mat(3, 3, CV_64FC1, cv::Scalar(0));
        r1.copyTo(R_mat.row(0));
        r2.copyTo(R_mat.row(1));
        r3.copyTo(R_mat.row(2));

        cv::Mat W, U, Vt;
        cv::SVDecomp(R_mat, W, U, Vt);

        R = U * Vt;
    }

    void PositionEstimator::calcPosition(const std::vector<cv::Point2f>& corners, const std::vector<cv::Point2f>& points, const cv::Mat& camMat, const cv::Mat& dist, cv::Mat& outR, cv::Mat& outT)
    {
        cv::Mat H = cv::findHomography(points, corners);

        decompositionH(camMat, H, outR, outT);
    }

    cv::Vec3d PositionEstimator::rotationMatrixToEulerAngles(const cv::Mat &R, bool inDegrees)
    {
        cv::Mat R2 = R.t();
        float sy = sqrt(R.at<double>(2,1) * R.at<double>(2,1) + R.at<double>(2,2) * R.at<double>(2,2));

        float x, y, z;
        if (sy > 1e-6)
        {
            x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
            y = atan2(-R.at<double>(2,0), sy);
            z = atan2(R.at<double>(1,0), R.at<double>(0,0));
        }
        else
        {
            x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
            y = atan2(-R.at<double>(2,0), sy);
            z = 0;
        }

        std::cout << "sy: " << sy << std::endl;
        std::cout << "R.at<double>(2,1): " << R.at<double>(2,1) << std::endl;
        std::cout << "R.at<double>(2,2): " << R.at<double>(2,2) << std::endl;
        std::cout << "x: " << x << std::endl;
        std::cout << "y: " << y << std::endl;
        std::cout << "z: " << z << std::endl;

        if (inDegrees)
        {
            return cv::Vec3d(rad2deg(x), rad2deg(y), rad2deg(z));
        }

        return cv::Vec3f(x, y, z);

    }
    
} // namespace paper_pos
