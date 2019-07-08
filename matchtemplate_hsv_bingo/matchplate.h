#ifndef MATCHTEMPLATE_INCLUDED
#define MATCHTEMPLATE_INCLUDED

#include <cv.hpp>
#include<cctype>
#include<iostream>

using namespace cv;
using namespace std;


class MatchTemplate
{
public:
    cv::Rect templRect;                     //模板矩形框
    cv::Mat templa;                        //模板
    bool drawing_rect;              //是否绘制矩形框

    void matching(cv::Mat frame, cv::Mat &templ, cv::Rect &rect);
    /*
    MatchTemplate()
    {
        templRect = Rect::Rect_(0, 0, 20, 20);    //初始化模板矩形框
        templa = Mat::zeros(20, 20, CV_32F);      //初始化模板
        drawing_rect = false;
    }
    */
};

#endif // MATCHTEMPLATE_INCLUDED
