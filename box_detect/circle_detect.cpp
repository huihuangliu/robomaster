#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>


using namespace std;
using namespace cv;


void colorFilter(Mat inputImage, Mat outputImage)
{
    int i, j;
    Mat hsv;

    cvtColor(inputImage, hsv, CV_BGR2HSV);
    int width = inputImage.cols;
    int height = inputImage.rows;

    vector<Mat> spl;
    split(hsv, spl);
    h_channel = spl[0];

    for (i = 0; i < height; i++)
        for (j = 0; j < width; j++)
        {
            Scalar s_hsv = cvGet2D(hsv, i, j);//获取像素点为（j, i）点的HSV的值
            /*
                opencv 的H范围是0~180，红色的H范围大概是(0~8)∪(160,180)
                S是饱和度，一般是大于一个值,S过低就是灰色（参考值S>80)，
                V是亮度，过低就是黑色，过高就是白色(参考值220>V>50)。
            */
            Scalar s;
            if (!(((s_hsv.val[0]>0)&&(s_hsv.val[0]<8)) || (s_hsv.val[0]>120)&&(s_hsv.val[0]<180)))
            {
                s.val[0]=0;
                s.val[1]=0;
                s.val[2]=0;
                cvSet2D(hsv, i ,j, s);
            }
        }
    outputImage = cvCreateMat(hsv->height, hsv->width, CV_8UC3 );
    outputImage = hsv;
    imshow("filter", hsv);
    waitKey(0);
}




int main()
{
    Mat src, image, gray, maskImage;
    src = imread("/home/robomaster/LHH/box_detect/test.jpg");
    //resize(src, image, Size(src.cols/2,src.rows/2), 0, 0,INTER_LINEAR);
    resize(src, image, Size(src.cols/4,src.rows/4), 0, 0,INTER_LINEAR);
    imshow("src", image);
    waitKey(0);

    cvtColor(image, gray, CV_BGR2GRAY);
    GaussianBlur(gray, gray, Size(7, 7), 2, 2);
    threshold(gray, maskImage, 65, 255, THRESH_BINARY);  //50-80   65 is best
    imshow("maskImage", maskImage);
    waitKey(0);
    Mat element1 = getStructuringElement(MORPH_RECT, Size(2, 2));
    Mat element2 = getStructuringElement(MORPH_RECT, Size(2, 2));
    erode(maskImage, maskImage, element2, cv::Point(-1, -1), 2);
    dilate(maskImage, maskImage, element1, cv::Point(-1, -1), 1);
    //imshow("src", maskImage);
    //waitKey(0);
    vector<Vec3f> circles;
    //hough circle
    HoughCircles(maskImage, circles, CV_HOUGH_GRADIENT, 1.5, 20, 200, 170, 0, 0);
    for (size_t i = 0; i < circles.size(); i++)
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        //circle center
        circle(image, center, 3, Scalar(0, 255, 0), -1, 8, 0);
        //circle contours
        circle(image, center, radius, Scalar(0, 255, 0), 3, 8, 0);
    }
    imshow("src", image);
    waitKey(0);

    //image.release();
    return 0;
}
