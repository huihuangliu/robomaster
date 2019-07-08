#include<opencv2/highgui/highgui.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<iostream>
#include<cctype>
#include<cv.hpp>
#include<cv.hpp>
#include "matchplate.h"

using namespace cv;
using namespace std;
RNG g_rng(12345);

bool add_matchtemplate = true;    //加上模板匹配方法辅助检测
bool stop_contours = true;         //不用轮廓查找的方法

int main()
{
    int countFrame = 0;
    MatchTemplate match;
    //读入视频
    VideoCapture capture("/home/robomaster/LHH/matchtemplate_hsv_bingo/out.avi");
    if (!capture.isOpened())
    {
        return 0;
    }


    Mat srcImage, outImage, maskImage, grayImage, hsvImage;

    while (capture.read(srcImage))
    {
        bool add_matchtemplate = true;
        countFrame++;
        double start = (double)getTickCount();
        outImage = srcImage.clone();//复制图片

        cvtColor(outImage, grayImage, CV_RGB2GRAY);//灰度化
        cvtColor(outImage, hsvImage, CV_RGB2HSV);    //转换到hsv空间
        blur(grayImage, maskImage, Size(3, 3)); //均值滤波
        threshold(maskImage, maskImage, 100, 255, CV_THRESH_BINARY);//二值化

                                                                    //形态学闭运算处理
        Mat element1 = getStructuringElement(MORPH_RECT, Size(2, 2));
        Mat element2 = getStructuringElement(MORPH_RECT, Size(2, 2));
        erode(maskImage, maskImage, element2, cv::Point(-1, -1), 1);
        dilate(maskImage, maskImage, element1, cv::Point(-1, -1), 1);

        //遍历寻找外轮廓
        vector<vector<Point> >contours, contours2;
        findContours(maskImage, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_L1, Point(0, 0));

        //比较每个轮廓的面积
        vector<Moments>mu(contours.size());
        for (int i = 0; i < contours.size(); i++)
        {
            Rect rect = boundingRect(contours[i]);
            if (rect.width + 2 < rect.height&&rect.height>1.5*rect.width)//去除一些噪点和横条
                mu[i] = moments(contours[i], 0);
        }
        vector<Point2f>mc(contours.size());
        for (int i = 0; i < contours.size(); i++)
        {
            mc[i] = Point2f(static_cast<float> (mu[i].m10 / mu[i].m00), static_cast<float>(mu[i].m10 / mu[i].m00));
        }

        int i, max = 0, second = 0;//找出面积最大的两个值，即两个竖条
        for (i = 0; i < contours.size(); i++)
        {
            if (mu[i].m00>max)
                max = mu[i].m00;
        }

        for (i = 0; i < contours.size(); i++)  //找到第二大的轮廓
        {
            if (mu[i].m00>second&&mu[i].m00 < max)
                second = mu[i].m00;
        }
        for (i = 0; i < contours.size(); i++)
        {
            if (abs(mu[i].m00 - max) < 1)
                contours2.push_back(contours[i]);
            if (abs(mu[i].m00 - second) < 1)
                contours2.push_back(contours[i]);
        }
        int e = 0;
        for (i = 0; i < contours2.size(); i++)
        {
            e++;
        }
        //cout <<max<< "\t"<<second<<"\t"<< e<<endl;

        if ((contours2.size() >= 2) && stop_contours)     //当检测到灯条的两个轮廓，就画出矩形框
        {
            Rect rect1 = boundingRect(contours2[0]);
            Rect rect2 = boundingRect(contours2[1]);
            Point point1 = Point(rect1.x, rect1.y);
            Point point2 = Point(rect2.x, rect2.y + rect2.height);
            Point point3 = Point(rect1.x + rect1.width, rect1.y);
            Point point4 = Point(rect2.x + rect2.width, rect2.y + rect2.height);

            vector<Point2f> rectpoint;      //轮廓查找所找到的矩形框四个顶点坐标
            rectpoint.push_back(point1);
            rectpoint.push_back(point2);
            rectpoint.push_back(point3);
            rectpoint.push_back(point4);

            vector<Point> track_points;
            //防止矩形框超出边界
            for (int i = 0; i < 4; i++)
            {
                if (rectpoint[i].x > hsvImage.cols)
                    rectpoint[i].x = hsvImage.cols - 1;
                if (rectpoint[i].y > hsvImage.rows)
                    rectpoint[i].y = hsvImage.rows - 1;
                if (rectpoint[i].x < 0)
                    rectpoint[i].x = 1;
                if (rectpoint[i].y < 0)
                    rectpoint[i].y = 1;
                track_points.push_back(rectpoint[i]);
            }

            //限制矩形长度，避免框到非对应的灯条
            if (abs(point2.y - point1.y) * 8 > abs(point2.x - point1.x))

                if (abs(point2.x - point1.x) < 122 && abs(point2.y - point1.y)*1.5 < abs(point2.x - point1.x))
                {
                    rectangle(srcImage, point1, point2, Scalar(0, 0, 255), 2, 8, 0);
                    add_matchtemplate = false;
                    cout << add_matchtemplate << endl;

                    //保存模板，并更新
                    /*
                    match.templRect.x = point1.x;
                    match.templRect.y = point1.y;
                    match.templRect.width = abs(point2.x - point1.x);
                    match.templRect.height = abs(point2.y - point1.y);
                    //match.templa = grayImage(match.templRect);      //模板图像,使用灰度模板匹配时
                    match.templa = hsvImage(match.templRect);        //模板图像,使用hsv模板匹配时
                    */
                    match.templRect = boundingRect(track_points);
                    /*
                    //防止模板矩形框超出边界
                    if (match.templRect.x <= 0)
                        match.templRect.x = 1;
                    if (match.templRect.x + match.templRect.width >= hsvImage.cols)
                        match.templRect.x = hsvImage.cols - match.templRect.width - 1;

                    if (match.templRect.y <= 0)
                        match.templRect.y = 1;
                    if (match.templRect.y + match.templRect.height >= hsvImage.rows)
                        match.templRect.y = hsvImage.rows - match.templRect.height - 1;
                    */
                    match.templa = hsvImage(match.templRect);     //更新模板
                    cout << "高：" << match.templa.rows << "宽：" << match.templa.cols << endl;

                }
        }
        if (add_matchtemplate)     //当没有检测到目标时，继续用模板匹配检测
        {
            cout << "出现掉帧" << endl;
            //参数：帧图像，模板图像，模板矩形框
            match.matching(outImage, match.templa, match.templRect);
            if (!match.templa.empty())
                imshow("template", match.templa);
            if (match.drawing_rect)
            {
                rectangle(srcImage, match.templRect, Scalar(0, 255, 0), 3);
                cout << "模板匹配画出矩形框" << endl;

            }
        }
        imshow("原图", srcImage);
        imshow("掩膜图", maskImage);
        //imshow("hsv图像", hsvImage);
        double end = (double)getTickCount();
        cout << "Frame: " << countFrame << "\tTime: ";
        cout << (end - start) * 1000 / (getTickFrequency()) << "ms" << endl;

        waitKey(1);
    }
    //手动释放视频捕获资源
    capture.release();
    waitKey(0);
    return 0;
}
