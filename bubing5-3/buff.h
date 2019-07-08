#ifndef BUFF
#define BUFF

#define debug 1

#define record2 1
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <python2.7/Python.h>
#include <iostream>
#include <unistd.h>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <sys/shm.h>
#include <sys/ipc.h>
#include <stdarg.h>
#include <stdio.h>
#include <pthread.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iterator>

using namespace cv;
using namespace std;
class send_image_data
{
    public:
        std::vector<cv::Mat> handwriting_image_data;
        cv::Mat Digital_tube_image_data;
        std::vector<cv::Point2f> handwriting_image_center;
        bool image_flag;
};

class camera_map
{
    public:
        Mat camera_map1;
        Mat camera_map2;
};


extern PyObject *pFunc_SMG, *pFunc_HW, *pFunc_fire;

namespace Buff{
/*************************************************
     * Maintainer:
     * Description:
     * -----------------------------------------------
     * Maintainer:
     * Description: buff识别
     *************************************************/

//一个手写体数字的应有信息


//通信模块
//class Serial_commu
//{

//public:
//    int serial_Init();                     //初始化串口
//    int serial_send(int fd, short *float_data);

////private:
//    int open_dev(const char *dev_name);
//    int set_port(const int &fd);
//    int send_data(const int &fd, const short *buffer);
//    int serial_Close(int fd);  //Close the serial
//    int serial_read(int fd, int *data);
//    int Select_Tart_Read(struct timeval tv,int  uart_fd,int* buff);
//    int monitor_routine( int  uart_fd, int* new_data);
//};

//视频读取
//class V4L_capture
//{

//public:

//    std::uint8_t getImageFromMemory();
//    std::uint8_t getImageFromMemory(Mat &image);
//    void printlog(const char *format,...);

//private:
//     long long frame_count_last=0;
//     FILE *flog;
//     std::uint8_t image_buffer[921600];        //640*480图像数据
//     int image_buffer_len;
//};

//
class Handle
{
public:
   send_image_data image_handle(Mat v4l_frame, camera_map & map_out);
private:
   vector<Mat> fire_context_finder(Mat image_RGB);
   vector<Mat> context_finder(Mat image_RGB);
   Mat title_finder(Mat image_RGB);

private:
   vector<cv::RotatedRect> SetSortPoint(vector<cv::RotatedRect> &arry);
   vector<cv::Rect> SetSortPoint(vector<cv::Rect> &arry);
   //两个坐标比较大小
   bool SetSortRule(Point2f &p1, Point2f &p2);
   Point getCenterPoint(cv::Rect &rect);
   int random(int a,int b);

private:
   Mat image_slice_title;
   Mat image_slice_context;

   Mat image_slice_checkout_title[5];
   Mat title_out_image;

   Mat image_slice_checkout_context[9];
   Mat context_out_image;

   std::vector<std::vector<cv::Point> > original_contours_title;
   std::vector<cv::Vec4i> hierarchy_title;
   std::vector<cv::Rect> original_rects_title;

   std::vector<std::vector<cv::Point> > original_contours_context;
   std::vector<cv::Vec4i> hierarchy_context;
   std::vector<cv::Rect> original_rects_context;

   std::vector<cv::Rect> rect_title;
   std::vector<cv::Point2f> rect_center;

   int context_hight_L = 20, context_hight_H = 120;
   int context_width_L = 20, context_width_H = 140;

   int fire_context_hight_L = 20, fire_context_hight_H = 120;
   int fire_context_width_L = 20, fire_context_width_H = 140;

   int title_hight_L = 10, title_hight_H = 70;
   int title_width_L = 5, title_width_H = 75;


};

class Angle_analysis
{
public:
    camera_map camera_matrix_init(Mat v4l_map);
    int angle_analysis(cv::Point2f &Target_point,cv::Point2f &convert_angle);

private:
    Mat intrinsic_;
    Mat distortion_coeff_;

    double transf_theta_;
    double transf_elpha_;

    cv::Point2f tmp_direction;

};


class BuffDetector
{
public:


    int loopStart(Handle &handle,Angle_analysis &angle,VideoWriter &out);

    void loopEnd() {stop_flag_ = true;}
public:
    //extern PyObject *pFunc_SMG, *pFunc_HW, *pFunc_fire;

private:
    int swich_context(vector<int> &context_num,vector<int> &SMG_num);
    int get_pixel(Mat img, Point pt);
private:
    cv::Point2f tar_center_cur;
    cv::Point2f convert_angle;
    vector<cv::Point2f> context_angle;
    vector<int> context = vector<int> (9);
    vector<int> SMG = vector<int> (5);
    vector<int> d_temp = vector<int> (9);
    vector<int> s_temp = vector<int> (5);
    vector<int> current_num = vector<int> (9);
    vector<int> current_SMG = vector<int> (5);
    vector<int> last_num = vector<int> (9);
    vector<int> last_SMG = vector<int> (5);
    camera_map map_out;
    PyObject *pArg_num, *pArg_SMG;
    PyObject *pList_num, *pList_SMG;
    PyObject *pValue_num, *pValue_SMG;
    int probability_of_num=0;
    int probability_of_SMG=0;

    int target_context;
private:

    bool stop_flag_=true;
    double sendData_buff[2];

};



}


#endif // BUFF

