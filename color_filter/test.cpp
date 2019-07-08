#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>
#include<iostream>
#include<string>

using namespace std;
using namespace cv;

//输入图像
Mat img;
//灰度值归一化
Mat bgr;
//HSV图像
Mat hsv;
//色相
int hmin = 215;
int hmin_Max = 360;
int hmax = 360;
int hmax_Max = 360;
//饱和度
int smin = 46;
int smin_Max = 255;
int smax = 255;
int smax_Max = 255;
//亮度
int vmin = 46;
int vmin_Max = 255;
int vmax = 250;
int vmax_Max = 255;
//显示原图的窗口
string windowName = "src";
//输出图像的显示窗口
string dstName = "dst";
//输出图像
Mat dst;

int main()
{
    int countFrame = 0;
    //读入视频
    VideoCapture capture("/home/robomaster/LHH/box_detect/box_42.mp4");
    if (!capture.isOpened())
    {
        return 0;
    }

    Mat srcImage, outImage;

    while (capture.read(srcImage))
    {
        countFrame++;
        resize(srcImage, img, Size(floor(srcImage.cols/2), floor(srcImage.rows/2)));
        //输入图像
        //img = imread("/home/robomaster/LHH/color_filter/test.jpg", IMREAD_COLOR);   //template_42.jpg
        //resize(img, img, Size(img.cols/4, img.rows/4), 0, 0,INTER_LINEAR);            //hsv:223,46,46
        if (!img.data || img.channels() != 3)
            return -1;
        imshow(windowName, img);
        //彩色图像的灰度值归一化
        img.convertTo(bgr, CV_32FC3, 1.0 / 255, 0);
        //颜色空间转换
        cvtColor(bgr, hsv, COLOR_BGR2HSV);
        //定义输出图像的显示窗口
        namedWindow(dstName, WINDOW_GUI_EXPANDED);
        //输出图像分配内存
        dst = Mat::zeros(img.size(), CV_32FC3);
        //掩码
        Mat mask;
        //inRange(hsv, Scalar(hmin, smin, vmin ), Scalar(hmax, smax , vmax ), mask);

        inRange(hsv, Scalar(hmin, smin / float(smin_Max), vmin / float(vmin_Max)), Scalar(hmax, smax / float(smax_Max), vmax / float(vmax_Max)), mask);
        //只保留
        for (int r = 0; r < bgr.rows; r++)
        {
            for (int c = 0; c < bgr.cols; c++)
            {
                if (mask.at<uchar>(r, c) == 255)
                {
                    dst.at<Vec3f>(r, c) = bgr.at<Vec3f>(r, c);
                }
            }
        }
        //输出图像
        dst.convertTo(dst, CV_8UC3, 255.0, 0);
        //imshow(dstName, dst);
        //waitKey(0);

        Mat gray, maskImage;
        cvtColor(dst, gray, CV_BGR2GRAY);


        GaussianBlur(gray, gray, Size(5, 5), 2, 2);
        threshold(gray, maskImage, 5, 255, THRESH_BINARY);

        Mat element1 = getStructuringElement(MORPH_RECT, Size(2, 2));
        Mat element2 = getStructuringElement(MORPH_RECT, Size(7, 7));
        morphologyEx(maskImage, maskImage, MORPH_OPEN, element1);
        morphologyEx(maskImage, maskImage, MORPH_OPEN, element1);
        //erode(maskImage, maskImage, element2, cv::Point(-1, -1), 2);
        dilate(maskImage, maskImage, element2, cv::Point(-1, -1), 5);

        imshow("maskImage", maskImage);
        waitKey(0);

        vector<vector<Point> > contours;

        findContours(maskImage, contours, CV_RETR_TREE, CV_CHAIN_APPROX_TC89_L1, Point(0, 0) );  //CV_CHAIN_APPROX_TC89_L1, CV_CHAIN_APPROX_SIMPLE

        // 寻找最大连通域
        double maxArea = 0;
        vector<cv::Point> maxContour;
        for(size_t i = 0; i < contours.size(); i++)
        {
            double area = cv::contourArea(contours[i]);
            if (area > maxArea)
            {
                maxArea = area;
                maxContour = contours[i];
            }
        }

        RotatedRect minRect = minAreaRect(maxContour);
        Point2f vertex[4];//用于存放最小矩形的四个顶点
        minRect.points(vertex);//返回矩形的四个顶点给vertex
        //绘制最小面积包围矩形
        for (int i = 0; i < 4; i++)
            line(img, vertex[i], vertex[(i + 1) % 4], Scalar(0, 0, 255), 1, 8);

        imshow("src", img);
        waitKey(0);

    }
    return 0;
}
