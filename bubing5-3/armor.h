#ifndef ARMOR_H
#define ARMOR_H
#include <stdint.h>
#include <iostream>
#include <functional>
#include <vector>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <sys/shm.h>
#include <sys/ipc.h>
#include <stdarg.h>
#include <pthread.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include<pthread.h>
#include <thread>
#include <sstream>
#include <string>
#include <fstream>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <sys/signal.h>
#include <cstdlib>
#include <stdint.h>
#include <ctime>

#define PI 3.14159265358979
#define debug 0
#define filter3 0
#define filter4 0

//using namespace cv;
//using namespace std;

namespace armor {
/*************************************************
     * Maintainer: 桂云涛 (2018-04-03)
     * Description: 装甲板识别 面向对象+回调 封装示例
     * -----------------------------------------------
     * Maintainer: 朱世涛
     * Description: 装甲板识别
     *************************************************/

class KalmanFilter
   {
   public:
       KalmanFilter() :
           KF_(4, 2)
       {
           state_ = cv::Mat::zeros(2, 1, CV_32F);// [x, y]'
           measurement_ = cv::Mat::zeros(2, 1, CV_32F);
           // 1
           randn(state_, cv::Scalar::all(0), cv::Scalar::all(0.1));
           KF_.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0,
               0, 1, 0, 1,
               0, 0, 1, 0,
               0, 0, 0, 1);
           KF_.measurementMatrix = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0,
               0, 1, 0, 0);
           cv::setIdentity(KF_.measurementMatrix, cv::Scalar::all(1));
           cv::setIdentity(KF_.processNoiseCov, cv::Scalar::all(1e-5));//5
           cv::setIdentity(KF_.measurementNoiseCov, cv::Scalar::all(1e-3));//3
           cv::setIdentity(KF_.errorCovPost, cv::Scalar::all(1));

           randn(KF_.statePost, cv::Scalar::all(0), cv::Scalar::all(0.1));
       }
   public:
       cv::Point2f/*cv::Mat*/ run(float x, float y)
       {
           // 2
           cv::Mat prediction = KF_.predict();
           //std::cout << prediction << std::endl;

           // 3
           measurement_.at<float>(0, 0) = x;
           measurement_.at<float>(1, 0) = y;

           // 4
           KF_.correct(measurement_);

           // 5
           state_.at<float>(0, 0) = KF_.statePost.at<float>(0, 0);
           state_.at<float>(1, 0) = KF_.statePost.at<float>(1, 0);
           //
           cv::Point2f ret;
           ret.x = KF_.statePost.at<float>(0, 0);
           ret.y = KF_.statePost.at<float>(1, 0);

           //return state_;
           //std::cout << ret << std::endl;
           return ret;
       }
   private:
       cv::Mat state_;
       cv::Mat measurement_;
       cv::KalmanFilter KF_;
   };



class Shooter
{
public:

    Shooter()
    {
        //1280*720
//        intrinsic_.create(3, 3, CV_64FC1);// to store the data for transfering to angle
//        intrinsic_.at<double>(0, 0) = 512.48;
//        intrinsic_.at<double>(0, 2) = 642.760;
//        intrinsic_.at<double>(1, 1) = 840.71;
//        intrinsic_.at<double>(1, 2) = 368.102;

//        intrinsic_.at<double>(0, 1) = 0;
//        intrinsic_.at<double>(1, 0) = 0;
//        intrinsic_.at<double>(2, 0) = 0;
//        intrinsic_.at<double>(2, 1) = 0;
//        intrinsic_.at<double>(2, 2) = 1;
        //640*480
        intrinsic_.create(3, 3, CV_64FC1);// to store the data for transfering to angle
        intrinsic_.at<double>(0, 0) = 1081.161;
        intrinsic_.at<double>(0, 2) = 319.562;
        intrinsic_.at<double>(1, 1) = 1077.257;
        intrinsic_.at<double>(1, 2) = 240.102;

        intrinsic_.at<double>(0, 1) = 0;
        intrinsic_.at<double>(1, 0) = 0;
        intrinsic_.at<double>(2, 0) = 0;
        intrinsic_.at<double>(2, 1) = 0;
        intrinsic_.at<double>(2, 2) = 1;
    }

    int motion_predict(cv::Point &tatget_pixel);

    int angle_analysis(cv::Point &tatget_point,cv::Point2f &convert_angle, double &Distance);
private:

    cv::Mat intrinsic_;

    double transf_theta_;
    double transf_elpha_;
    double x_integrate;
    double y_integrate;


    cv::Point2f tmp_direction;
};

//class Serial_commu
//{
//public:

//    void usart_init();
//    int set_opt(struct termios newtio,int fd,int nSpeed, int nBits, char nEvent, int nStop);
//    int send_data(int fd_, const int16_t *buffer);
//    int send_dataarmor(int fd_, const int *buffer);
//    int serial_send(int fd_, double *float_data,int len);
//    int serial_sendarmor(int fd_, double *float_data,int len);

////    extern int model_flag;
////    extern int usart_0;
//};

//class V4L_capture
//{

//public:

//    std::uint8_t getImageFromMemory();
//    std::uint8_t getImageFromMemory(cv::Mat &image);
//    void printlog(const char *format,...);

//private:
//     long long frame_count_last=0;
//     FILE *flog;
//     std::uint8_t image_buffer[921600];        //640*480图像数据
//     int image_buffer_len;
//};

class ArmorDetector
{

public:


    void loopStart(Shooter &shooter,cv::VideoWriter &out);

    void loopEnd() {stop_flag_ = true;}

private:

    int preprocessImg(cv::Mat &src,cv::Mat &mask,cv::Mat &show);

    int findTargets(cv::Mat &src,cv::Mat &mask,cv::Mat &show);

    int target_track(cv::Mat &src,cv::Mat &show);

    bool makeRectSafe(cv::Rect & rect, cv::Size size)
    {
        if (rect.x < 0)
            rect.x = 0;
        if (rect.x + rect.width > size.width)
            rect.width = size.width - rect.x;
        if (rect.y < 0)
            rect.y = 0;
        if (rect.y + rect.height > size.height)
            rect.height = size.height - rect.y;
        if (rect.width <= 0 || rect.height <= 0)
            return false;
        return true;
    }

    //像素距离函数
    float dist(float ax, float ay, float bx, float by)
    {
        float d = sqrt((ax - bx)*(ax - bx) + (ay - by)*(ay - by));
        return d;
    }
    //像素之间角度函数
    float angle(float x1, float y1, float x2, float y2);

    void makeTemplate(int cols,int rows,cv::Mat src);

    void makeTemplatebig(int cols,int rows,cv::Mat src);

    int brightSums(cv::Mat src);

private:

    int get_one=0;
    int lost_frame=0;
    int getTarget=0;
    int continue_shot = 0;
    int Is_recognizing = 0;
    int TargetGet_times = 0;
    int TargetLost_times = 0;
    int track_object = 0;
    int min_height=10;
    int max_up_down_angle=17;

    const int const_lost_frame = 30; //最大失帧数
    const int max_half_w = 400;
    const int max_half_h = 300;
    const int const_lost_match_frame=20;

    uint min_contours_size=3;
    uint min_contour_height=3;
    uint min_angle=30;


    cv::Point target_pixel;      //目标像素坐标
    cv::Point last_point;
    cv::Point center;
    cv::Point up_left;
    cv::Point2f lu;
    cv::Point2f rd;
    cv::Point2f vertex_result[4];
    cv::Point2f track_lu;
    cv::Point2f track_rd;

    cv::Mat _src;
    cv::Mat standard_template;
    cv::Mat image_roix;
    cv::Mat result;
    cv::Mat gray_image;
    cv::Mat copy_image;
    cv::Mat threshold_image;
    cv::Mat track_Image;
    cv::Mat track_image_march;


    cv::RotatedRect _res_last;      // last detect result
    cv::Rect _dect_rect;            // detect roi of original image
    cv::Rect track_window;
    cv::Rect _track_rect;

    std::vector<cv::Point> contours_result;

    cv::Mat _g;                     // green component of source image
    cv::Mat _ec;                    // enemy color
    cv::Mat _rgb_color;             // binary image of sub between blue and red component
    cv::Size _size;
    int min_light_gray=180;
    int track_delta_wtidth;

private:

    bool stop_flag_=true;
    double sendData_armor[3];
};

}
#endif // ARMOR_H
