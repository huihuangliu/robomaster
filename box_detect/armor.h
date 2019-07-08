#pragma once
#pragma once
#ifndef ARMOR_H
#define ARMOR_H
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>


using namespace std;
using namespace cv;



class ArmorDetector
{
public:
    ArmorDetector()
    {
        intrinsic_.create(3, 3, CV_64FC1);// to store the data for transfering to angle
        intrinsic_.at<double>(0, 0) = 1063.48;
        intrinsic_.at<double>(0, 2) = 391.562;
        intrinsic_.at<double>(1, 1) = 1070.71;
        intrinsic_.at<double>(1, 2) = 300.102;

        intrinsic_.at<double>(0, 1) = 0;
        intrinsic_.at<double>(1, 0) = 0;
        intrinsic_.at<double>(2, 0) = 0;
        intrinsic_.at<double>(2, 1) = 0;
        intrinsic_.at<double>(2, 2) = 1;
    }

    void findArmor(const cv::Mat &src);

    void setImage(const cv::Mat & src);

    void findTarget(const cv::Mat & src, const cv::Mat &mask);

    void target_track(void);

    void tmp(const cv::Point2f &Target_point);

    void Aimtarget(void);

private:
    cv::RotatedRect _res_last;      // last detect result
    cv::Rect _dect_rect;            // detect roi of original image
    cv::Mat _binary_template;       // armor template binary image
    cv::Mat _binary_template_small; // small armor template binay image
    cv::Mat _src;                   // source image
    cv::Mat _g;                     // green component of source image
    cv::Mat _ec;                    // enemy color
    cv::Size _size;
    cv::Mat intrinsic_;

    Point2f convert_dirct;

    Point2f tmp_direction;
};



#endif // ARMOR_H

