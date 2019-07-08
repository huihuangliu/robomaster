#include "buff.h"
#include "serial_commu.hpp"
#include "Structure.h"

extern int model_flag;
extern double sendData_buff;
extern short sendzero[2]={0,0};

extern int debug_flag;

extern int title_h_L;//znj
extern int title_s_L;//znj
extern int title_v_L;//znj
extern int title_h_H;//znj
extern int title_s_H;//znj
extern int title_v_H;//znj
extern int title_hight_L;
extern int title_hight_H;
extern int title_width_L;
extern int title_width_H;
extern int SMG_height_difference;
extern int SMG_width_difference;
extern int SMG_x_difference;
extern int SMG_y_difference;

extern int context_h_L;//znj
extern int context_s_L;//znj
extern int context_v_L;//znj
extern int context_h_H;//znj
extern int context_s_H;//znj
extern int context_v_H;//znj
extern int context_hight_L;
extern int context_hight_H;
extern int context_width_L;
extern int context_width_H;
extern int HW_height_difference;
extern int HW_width_difference;
extern int HW_x_difference;
extern int HW_y_difference;

extern int fire_context_h_L;//znj
extern int fire_context_s_L;//znj
extern int fire_context_v_L;//znj
extern int fire_context_h_H;//znj
extern int fire_context_s_H;//znj
extern int fire_context_v_H;//znj
extern int fire_context_hight_L;
extern int fire_context_hight_H;
extern int fire_context_width_L;
extern int fire_context_width_H;
extern int fire_y_difference;
extern int fire_x_difference;

extern int adjust_title_flag;
extern int adjust_context_flag;
extern int usart_0;

extern std::vector<cv::Rect> rect_context_backup;
extern Mat image_RGB_backup;
extern int draw_target_context;
extern int rectangle_flag;

using namespace std;
using namespace cv;
#define DEBUG

namespace Buff{

int D_height=0;

void parameter_adjust()
{
    namedWindow("title");
    createTrackbar("title_h_L", "title", &title_h_L, 180);
    createTrackbar("title_h_H", "title", &title_h_H, 180);
    createTrackbar("title_s_L", "title", &title_s_L, 255);
    createTrackbar("title_s_H", "title", &title_s_H, 255);
    createTrackbar("title_v_L", "title", &title_v_L, 255);
    createTrackbar("title_v_H", "title", &title_v_H, 255);
    createTrackbar("title_hight_L", "title", &title_hight_L, 255);
    createTrackbar("title_hight_H", "title", &title_hight_H, 255);
    createTrackbar("title_width_L", "title", &title_width_L, 255);
    createTrackbar("title_width_H", "title", &title_width_H, 255);
    createTrackbar("SMG_height_difference", "title", &SMG_height_difference, 255);
    createTrackbar("SMG_width_difference", "title", &SMG_width_difference, 255);
    createTrackbar("SMG_x_difference", "title", &SMG_x_difference, 255);
    createTrackbar("SMG_y_difference", "title", &SMG_y_difference, 255);

    namedWindow("context");
    createTrackbar("context_h_L", "context", &context_h_L, 180);
    createTrackbar("context_h_H", "context", &context_h_H, 180);
    createTrackbar("context_s_L", "context", &context_s_L, 255);
    createTrackbar("context_s_H", "context", &context_s_H, 255);
    createTrackbar("context_v_L", "context", &context_v_L, 255);
    createTrackbar("context_v_H", "context", &context_v_H, 255);
    createTrackbar("context_hight_L", "context", &context_hight_L, 255);
    createTrackbar("context_hight_H", "context", &context_hight_H, 255);
    createTrackbar("context_width_L", "context", &context_width_L, 255);
    createTrackbar("context_width_H", "context", &context_width_H, 255);
    createTrackbar("HW_height_difference", "context", &HW_height_difference, 255);
    createTrackbar("HW_width_difference", "context", &HW_width_difference, 255);
    createTrackbar("HW_x_difference", "context", &HW_x_difference, 255);
    createTrackbar("HW_y_difference", "context", &HW_y_difference, 255);

    namedWindow("fire");
    createTrackbar("fire_context_h_L", "fire", &fire_context_h_L, 180);
    createTrackbar("fire_context_h_H", "fire", &fire_context_h_H, 180);
    createTrackbar("fire_context_s_L", "fire", &fire_context_s_L, 255);
    createTrackbar("fire_context_s_H", "fire", &fire_context_s_H, 255);
    createTrackbar("fire_context_v_L", "fire", &fire_context_v_L, 255);
    createTrackbar("fire_context_v_H", "fire", &fire_context_v_H, 255);
    createTrackbar("fire_context_hight_L", "fire", &fire_context_hight_L, 255);
    createTrackbar("fire_context_hight_H", "fire", &fire_context_hight_H, 255);
    createTrackbar("fire_context_width_L", "fire", &fire_context_width_L, 255);
    createTrackbar("fire_context_width_H", "fire", &fire_context_width_H, 255);
    createTrackbar("fire_y_difference", "fire", &fire_y_difference, 255);
    createTrackbar("fire_x_difference", "fire", &fire_x_difference, 255);
}
#if record2
int count = 0;
#endif
int BuffDetector::loopStart(Handle &handle,Angle_analysis &angle, VideoWriter &out)
{
    if(debug_flag)
        parameter_adjust();//znj
    static int stability_judge_title2 = 1;
    static int stability_judge_context2 = 1;
    static int corect_judge_context = 0;
    static int corect_judge_title = 0;
    //*********************python初始化***************

    //VideoCapture capture("/home/robomaster/桌面/num1.avi");
    //VideoCapture capture("/home/robomaster/桌面/num2.avi");
    //VideoCapture capture("/home/robomaster/桌面/rec_03-31_15-03-57_240.avi");
    //VideoCapture capture(0);
    //VideoCapture capture("/home/allspark003/1_2.avi");
    //VideoCapture capture("/home/allspark003/1_3.avi");
    //VideoCapture capture("/home/allspark003/9.avi");
    VideoCapture capture("/home/allspark003/build-bubing5-3-unknown-Debug/1.avi");
    Mat v4l_frame;
    send_image_data send_data;
    map_out = angle.camera_matrix_init(v4l_frame);

    //capture>>v4l_frame;

    do
    {

//        getImageFromMemory(v4l_frame);
//        if (v4l_frame.empty())
//        {
//            std::cout<<"frame is empty!Maybe you need to open V4L!"<<std::endl;
//            exit(1);
//        }

        //判别buff模式，大 or 小(model_flag),预处理

       capture>>v4l_frame;
#if record2
//               if(count%360==0)
//                  return -1;
                out<<v4l_frame;
                //waitKey(1);
                //count++;
               \
                //cout<<count<<"<<<<<<<<<<<<<<<<<<<<<"<<endl;

#endif
        send_data = handle.image_handle(v4l_frame, map_out);

        //去除错误预处理
        if(send_data.Digital_tube_image_data.size() !=Size(140,28)||
                send_data.handwriting_image_data.size() != 9 ||
                send_data.Digital_tube_image_data.data == 0 ||
                send_data.handwriting_image_center.size() != 9)
            continue;
        //imshow
        double startPre = (double)cv::getTickCount();
        Mat hconcat_image;
        hconcat_image = send_data.handwriting_image_data[0];

        for (int i = 0; i < send_data.handwriting_image_data.size() - 1; i++)
        {
            hconcat(hconcat_image, send_data.handwriting_image_data[i + 1], hconcat_image);
        }
        if(debug_flag)
            imshow("context_image_data",hconcat_image);
        if(debug_flag)
            imshow("Digital_tube_image_data", send_data.Digital_tube_image_data);

        //神经网络识别
        //context:

        pArg_num = PyTuple_New(2);
        pList_num = PyList_New(0);
        for (int i = 0; i < 28; i++)
        {
            for (int j = 0; j < 28*9; j++)
            {
                int number = get_pixel(hconcat_image,Point(j,i));
                PyList_Append(pList_num, Py_BuildValue("i", number));
            }
        }
        int size_num = PyList_Size(pList_num);
        PyTuple_SetItem(pArg_num, 0, pList_num);
        PyTuple_SetItem(pArg_num, 1, Py_BuildValue("i", size_num));
        //HW num
        if(send_data.image_flag == 0)
            pValue_num = PyEval_CallObject(pFunc_HW, pArg_num);
        //fire num
        else
            pValue_num = PyEval_CallObject(pFunc_fire, pArg_num);


        //target 寻找
        if (PyTuple_GET_SIZE(pValue_num) == 9)
        {
            corect_judge_context ++;
            if(corect_judge_context == 10)
            {
                adjust_context_flag = 0;
            }
            stability_judge_context2 = 5;
            PyArg_ParseTuple(pValue_num, "i|i|i|i|i|i|i|i|i", &context[0],&context[1],&context[2],&context[3],&context[4],&context[5],&context[6],&context[7],&context[8]);
            cout<<context[0]<<context[1]<<context[2]<<context[3]<<context[4]<<context[5]<<context[6]<<context[7]<<context[8]<<endl;
        }
        else
        {
//            cout<<"num identify failed!"<<endl;
            if(adjust_context_flag == 1)
            {
                stability_judge_context2 --;
            }

            if(stability_judge_context2 == 0)
            {
                context_v_L ++;
                //cout<<"context_v_L:"<<context_v_L<<endl;
                stability_judge_context2 = 2;
            }

        }

        Py_CLEAR(pList_num);
        Py_CLEAR(pArg_num);
        Py_CLEAR(pValue_num);

        //smg:
        pArg_SMG = PyTuple_New(2);
        pList_SMG = PyList_New(0);
        for (int i = 0; i < 28; i++)
        {
            for (int j = 0; j < 28*5; j++)
            {
                int number = get_pixel(send_data.Digital_tube_image_data,Point(j,i));
                PyList_Append(pList_SMG, Py_BuildValue("i", number));
            }
        }
        int size_SMG = PyList_Size(pList_SMG);

        PyTuple_SetItem(pArg_SMG, 0, pList_SMG);
        PyTuple_SetItem(pArg_SMG, 1, Py_BuildValue("i", size_SMG));

        pValue_SMG = PyEval_CallObject(pFunc_SMG, pArg_SMG);

        if (PyTuple_GET_SIZE(pValue_SMG) == 5)
        {
            //cout<<"SMG identify succeed!"<<endl;
            corect_judge_title ++;
            if(corect_judge_title == 10)
            {
                adjust_title_flag = 0;
            }
            stability_judge_title2 = 5;
            PyArg_ParseTuple(pValue_SMG, "i|i|i|i|i", &SMG[0],&SMG[1],&SMG[2],&SMG[3],&SMG[4]);
            cout<<SMG[0]<<SMG[1]<<SMG[2]<<SMG[3]<<SMG[4]<<endl;
        }
        //
        else
        {
            //cout<<"SMG identify failed!"<<endl;
            if(adjust_title_flag == 1)
            {
                stability_judge_title2 --;
            }
            if(stability_judge_title2 == 0)
            {
                title_v_L ++;
                //cout<<"title_v_L:"<<title_v_L<<endl;
                stability_judge_title2 = 2;
            }
            continue;
        }


        Py_CLEAR(pList_SMG);
        Py_CLEAR(pArg_SMG);
        Py_CLEAR(pValue_SMG);

        int Exit = 1;
        for(int time=0;time<9;time++)
        {
            if(context[time]==0)
                Exit = 0;
        }

        if(Exit == 0 )
        {
            Exit = 1;
            cout<<"aa"<<endl;
            continue;
        }

        target_context=swich_context(context,SMG);
        if(target_context!=9 && target_context!=-1)
        {
            draw_target_context = target_context;
        }


        //角度解析
        vector<cv::Point2f>().swap(context_angle);
        uint lenth = send_data.handwriting_image_center.size();
        for(uint i = 0; i < lenth ; i++)
        {
            tar_center_cur = cv::Point2f(send_data.handwriting_image_center[i]);// 2017.3.6
            angle.angle_analysis(tar_center_cur,convert_angle);
            context_angle.push_back(convert_angle);
            //cout<<context_angle;
        }
        //串口通信
        if(debug_flag)
            cout<<"target_context"<<target_context<<endl;
        if(target_context!=9 && target_context!=-1)
        {
            sendData_buff[0]=context_angle[target_context].x * 100;
            sendData_buff[1]=context_angle[target_context].y * 100;
            for(int send_time=0;send_time<10;send_time++)
                serial_sendarmor(usart_0, sendData_buff,2);
            if(debug_flag)
                std::cout<<"sending data: "<<sendData_buff[0]<<'\t'<<sendData_buff[1]<<endl;
            double endPre = (double)cv::getTickCount();
            if(debug_flag)
                std::cout << "preprocess time="<<(endPre - startPre) * 1000 / (cv::getTickFrequency()) << "ms" << std::endl <<endl;

        }

    }while(model_flag != -1);
    if(model_flag==-1)
    {
        destroyAllWindows();

    }

}

int BuffDetector::swich_context(vector<int>& context_num, vector<int>&SMG_num)
{
    static int compare_temp=0;
    static int temp_j=0;
    static int right_times = -1;

//    if(context_num!=vector<int>(9) && SMG_num!=vector<int>(5))
//    {

//        if (d_temp != context_num)
//        {
//            probability_of_num=0;
//            d_temp=context_num;
//        }
//        else
//        {
//            if(probability_of_num<1)
//                probability_of_num++;
//        }
//        if (s_temp != SMG_num)
//        {
//            probability_of_SMG=0;
//            s_temp=SMG_num;
//        }
//        else
//        {
//            if(probability_of_SMG<1)
//                probability_of_SMG++;

//        }

//        if(probability_of_num==1)
//        {
//            current_num=d_temp;
//        }
//        if(probability_of_SMG==1)
//        {
//            current_SMG=s_temp;
//        }

//        if(current_num!=vector<int>(9) && current_SMG!=vector<int>(5))
//        {
//            if(last_num!=current_num || last_SMG!=current_SMG)
//            {

//                if (last_SMG != current_SMG)
//                {
//                      if(debug_flag)
//                        cout<<"SMG is changed."<<endl;
//                      if(last_num==current_num)
//                      {
//                           copy(current_SMG.begin(),current_SMG.end(),ostream_iterator<int>(cout," "));
//                           if(debug_flag)
//                             cout<<endl<<"right_time:"<<right_times<<endl;
//                           return -1;
//                      }
//                      last_SMG=current_SMG;
//                      right_times=-1;
//                      copy(current_SMG.begin(),current_SMG.end(),ostream_iterator<int>(cout," "));
//                      if(debug_flag)
//                        cout<<endl;
//                }
//                right_times++;

//                compare_temp=current_SMG[right_times];
//                for(temp_j=0;temp_j<9;temp_j++)
//                {
//                    if(compare_temp == current_num[temp_j])
//                    break;
//                }
//                last_num=context_num;
//                if(debug_flag)
//                    cout<<"context is changed."<<endl;
//                copy(context_num.begin(),context_num.end(),ostream_iterator<int>(cout," "));
//                if(debug_flag)
//                    cout<<endl;
//                if(debug_flag)
//                    cout<<"right_time:"<<right_times<<endl;
//                if(right_times>=4)
//                    right_times = -1;
//                return temp_j;
//            }
//        }
//    }
//    return -1;
    compare_temp=SMG_num[0];
                    for(temp_j=0;temp_j<9;temp_j++)
                    {
                        if(compare_temp == context_num[temp_j])
                        break;
                    }
                    return temp_j;
}
int BuffDetector::get_pixel(Mat img, Point pt)
{
    int width = img.cols; //图片宽度
    uchar* ptr = (uchar*) img.data + pt.y * width; //获得灰度值数据指针
    int intensity = ptr[pt.x];
    return intensity;
}

//handle::
int Buff::Handle::random(int a,int b)
{
    return rand()%(b-a+1)+a;
}
Point Buff::Handle::getCenterPoint(cv::Rect &rect)
{
    Point cpt;
    cpt.x = rect.x + cvRound(rect.width/2.0);
    cpt.y = rect.y + cvRound(rect.height/2.0);
    return cpt;
}
vector<cv::Rect> Buff::Handle::SetSortPoint(vector<cv::Rect> &arry)
{
    vector<cv::Point2f> center;
    uint len = arry.size();

    for(uint i = 0; i<len; i++)
    {
        center.push_back(getCenterPoint(arry[i]));
    }
    for (uint i = 0; i < len - 1; i++)
    {
        for (uint j = 0; j < len - 1 - i; j++)
        {
            if (SetSortRule(center[j], center[j + 1]))
            {
                Point2f tmp = center[j];
                center[j] = center[j + 1];
                center[j + 1] = tmp;
                Rect rect_tmp = arry[j];
                arry[j] = arry[j + 1];
                arry[j + 1] = rect_tmp;
            }
        }
    }
    return (arry);
}
vector<cv::RotatedRect> Buff::Handle::SetSortPoint(vector<cv::RotatedRect> &arry)
{
    std::vector<cv::Point2f> center;
    uint len = arry.size();
    //cout << "len :" << len << endl;
    for(uint i = 0; i<len; i++)
    {
        center.push_back(arry[i].center);
    }
    for (uint i = 0; i < len - 1; i++)
    {
        for (uint j = 0; j < len - 1 - i; j++)
        {
            if (SetSortRule(center[j], center[j + 1]))
            {
                Point2f tmp = center[j];
                center[j] = center[j + 1];
                center[j + 1] = tmp;
                RotatedRect rect_tmp = arry[j];
                arry[j] = arry[j + 1];
                arry[j + 1] = rect_tmp;
            }
        }
    }
    return (arry);
}
bool Buff::Handle::SetSortRule(Point2f &p1, Point2f &p2)
{
    float judge_sum1 = p1.x + 6*p1.y;
    float judge_sum2 = p2.x + 6*p2.y;
    if (judge_sum1 < judge_sum2)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
vector<Mat> Buff::Handle::fire_context_finder(Mat image_RGB)
{
    static std::vector<cv::Rect> rect_context;
    static cv::Rect rect;
    static Mat image_hsv_context, image_hsv_context_gray, mask,image_hsv_context_copy;
    static Mat context_hsv_backup;
    vector<Mat> handwriting_image;
    static int stability_judge_context = 1;

    Mat mv[3];
    Mat image_H_context,image_S_context,image_V_context;
    Mat image_hsv;
    Mat image_RGB_temp;
    image_RGB.copyTo(image_RGB_temp);
    cvtColor(image_RGB_temp, image_hsv, CV_BGR2HSV);
    split(image_hsv, mv);
    inRange(mv[0], fire_context_h_L, fire_context_h_H, image_H_context);
    inRange(mv[1], fire_context_s_L, fire_context_s_H, image_S_context);
    inRange(mv[2], fire_context_v_L, fire_context_v_H, image_V_context);
    bitwise_and(image_H_context, image_S_context, image_S_context);
    bitwise_and(image_S_context, image_V_context, image_hsv_context);

    //tihuan
    threshold(image_hsv_context_gray,image_hsv_context_gray,150,255,THRESH_BINARY);
    //tihuan
    medianBlur(image_hsv_context,image_hsv_context,3);
    srand((unsigned)time(NULL));
    for(int i = 0; i<4 ; i++)
    {
        int rand_x = random(1,image_hsv_context.cols-1);
        int rand_y = random(1,10);
        floodFill(image_hsv_context,mask,Point(rand_x,rand_y),0);
    }

    for(int i = 0; i<4 ; i++)
    {
        int rand_x = random(1,10);
        int rand_y = random(1,image_hsv_context.rows-1);
        floodFill(image_hsv_context,mask,Point(rand_x,rand_y),0);
    }
    cv::erode(image_hsv_context, image_hsv_context, cv::Mat(3, 3, CV_8U), cv::Point(-1, -1), 1);
    image_hsv_context.copyTo(image_hsv_context_copy);
    if(debug_flag)
        imshow("handwriting num mask",image_hsv_context);
    cv::findContours(image_hsv_context_copy, original_contours_context, hierarchy_context, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_L1);

    if (rect_context.size() != 0)
        vector<cv::Rect>().swap(rect_context);

    if (handwriting_image.size() != 0)
        vector<Mat>().swap(handwriting_image);

    if (rect_center.size() != 0)
        vector<cv::Point2f>().swap(rect_center);

    std::vector<cv::Rect> box;

    for (uint i = 0; i < original_contours_context.size(); i++)
    {
        rect = cv::boundingRect(original_contours_context[i]);
        if (rect.height > fire_context_hight_L &&rect.height < fire_context_hight_H &&
            rect.width > fire_context_width_L&& rect.width < fire_context_width_H)
        {
//            double area = contourArea(original_contours_context[i]);
//            double isrect = area / (rect.width*rect.height);
////            if (isrect<0.6)
////                continue;
//            rect.x += 2; rect.width -= 4; rect.y += 3; rect.height -= 6;
            box.push_back(rect);
        }
    }

    for (uint i = 0; i < box.size(); i++)
    {
        int count = 0;
        int count2 = 0;
        for (uint j = 0; j < box.size(); j++)
        {
                if (abs(box[i].y - box[j].y) <fire_y_difference/10.0)
                {
                    count += 1;
                }
                else if(abs(box[i].x - box[j].x) <fire_x_difference/10.0)
                    count2 += 1;
        }
        if (count >= 2 && count2 >= 2)
        {
            box[i].x -= 5;box[i].width += 8; box[i].y -= 5; box[i].height += 8;
            rect_context.push_back(box[i]);
            cv::rectangle(image_RGB, box[i], cv::Scalar(0, 255, 0), 2);
        }
    }

    if(rect_context.size()==9)
    {
        rect_context = SetSortPoint(rect_context);
        uint len = rect_context.size();
        for(uint i = 0; i<len; i++)
        {
            rect_center.push_back(getCenterPoint(rect_context[i]));
        }
    }

    if (rect_context.size() == 9)
    {
        stability_judge_context = 20;
        int size = rect_context.size();
        for (uint i = 0; i < size; i++)
        {
            image_slice_checkout_context[i] = cv::Mat(image_hsv_context, rect_context[i]);
            cv::resize(image_slice_checkout_context[i], image_slice_checkout_context[i], cv::Size(28, 28), 0, 0, cv::INTER_LINEAR);
            handwriting_image.push_back(image_slice_checkout_context[i]);
        }
    }
    else
    {
        if(adjust_context_flag == 1)
        {
            stability_judge_context --;
        }
        if(stability_judge_context == 0)
        {
            context_v_L ++;
            stability_judge_context = 1;
        }
        if(context_v_L > 255)
            context_v_L = 70;
    }
    return handwriting_image;

}
vector<Mat> Buff::Handle::context_finder(Mat image_RGB)
{
    static std::vector<cv::Rect> rect_context;
    static cv::Rect rect;
    static Mat image_hsv_context, image_hsv_context_gray, mask,image_hsv_context_copy;
    static Mat context_hsv_backup;
    vector<Mat> handwriting_image;
    static int stability_judge_context = 1;

    Mat mv[3];
    Mat image_H_context,image_S_context,image_V_context;
    Mat image_hsv;
    Mat image_RGB_temp;
    image_RGB.copyTo(image_RGB_temp);
    image_RGB.copyTo(image_RGB_backup);
    cvtColor(image_RGB_temp, image_hsv, CV_BGR2HSV);
    split(image_hsv, mv);
    inRange(mv[0], context_h_L, context_h_H, image_H_context);
    inRange(mv[1], context_s_L, context_s_H, image_S_context);
    inRange(mv[2], context_v_L, context_v_H, image_V_context);
    bitwise_and(image_H_context, image_S_context, image_S_context);
    bitwise_and(image_S_context, image_V_context, image_hsv_context);

    //tihuan
    threshold(image_hsv_context_gray,image_hsv_context_gray,150,255,THRESH_BINARY);
    //tihuan
    medianBlur(image_hsv_context,image_hsv_context,3);
    srand((unsigned)time(NULL));
    for(int i = 0; i<4 ; i++)
    {
        int rand_x = random(1,image_hsv_context.cols-1);
        int rand_y = random(1,10);
        floodFill(image_hsv_context,mask,Point(rand_x,rand_y),0);
    }

    for(int i = 0; i<4 ; i++)
    {
        int rand_x = random(1,10);
        int rand_y = random(1,image_hsv_context.rows-1);
        floodFill(image_hsv_context,mask,Point(rand_x,rand_y),0);
    }
    cv::erode(image_hsv_context, image_hsv_context, cv::Mat(3, 3, CV_8U), cv::Point(-1, -1), 1);
    image_hsv_context.copyTo(image_hsv_context_copy);
    if(debug_flag)
        imshow("handwriting num mask",image_hsv_context);
    cv::findContours(image_hsv_context_copy, original_contours_context, hierarchy_context, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_L1);

    if (rect_context.size() != 0)
        vector<cv::Rect>().swap(rect_context);

    if (handwriting_image.size() != 0)
        vector<Mat>().swap(handwriting_image);

    if (rect_center.size() != 0)
        vector<cv::Point2f>().swap(rect_center);

    std::vector<cv::Rect> box;

    for (uint i = 0; i < original_contours_context.size(); i++)
    {
        rect = cv::boundingRect(original_contours_context[i]);
        if (rect.height > context_hight_L &&rect.height < context_hight_H &&
            rect.width > context_width_L&& rect.width < context_width_H &&
            rect.height < rect.width)
        {
            double area = contourArea(original_contours_context[i]);
            double isrect = area / (rect.width*rect.height);
            if (isrect<0.6)
                continue;
            rect.x += 2; rect.width -= 4; rect.y += 3; rect.height -= 6;
            box.push_back(rect);
        }
    }

    for (uint i = 0; i < box.size(); i++)
    {
        int count = 0;
        for (uint j = 0; j < box.size(); j++)
        {
              if (abs(box[i].height - box[j].height) <HW_height_difference/10.0
                      && abs(box[i].width - box[j].width) <HW_width_difference/10.0
                      && abs(box[i].x - box[j].x) < box[i].width*HW_x_difference/10.0
                      && abs(box[i].y - box[j].y) < box[i].height*HW_y_difference/10.0)
                count += 1;
        }
        if (count == 9)
        {
            rect_context.push_back(box[i]);
            rect_context_backup.push_back(box[i]);
            rectangle_flag = 1;
            //cv::rectangle(image_RGB, box[i], cv::Scalar(0, 255, 0), 2);
        }
    }

    if(rect_context.size()==9)
    {
        rect_context = SetSortPoint(rect_context);
        uint len = rect_context.size();
        for(uint i = 0; i<len; i++)
        {
            rect_center.push_back(getCenterPoint(rect_context[i]));
        }
    }

    if (rect_context.size() == 9)
    {
        stability_judge_context = 20;
        for (uint i = 0; i < rect_context.size(); i++)
        {
            image_slice_checkout_context[i] = cv::Mat(image_hsv_context, rect_context[i]);
            cv::resize(image_slice_checkout_context[i], image_slice_checkout_context[i], cv::Size(28, 28), 0, 0, cv::INTER_LINEAR);
            bitwise_not(image_slice_checkout_context[i], image_slice_checkout_context[i]);
            handwriting_image.push_back(image_slice_checkout_context[i]);
        }
    }
    else
    {
        if(adjust_context_flag == 1)
        {
            stability_judge_context --;
        }
        if(stability_judge_context == 0)
        {
            context_v_L ++;
            stability_judge_context = 1;
            //cout<<"context_v_L:"<<context_v_L<<endl;
        }
        if(context_v_L > 255)
            context_v_L = 70;
    }
    return handwriting_image;
}
Mat Buff::Handle::title_finder(Mat image_RGB)
{
    static Mat image_hsv_title,mask,image_hsv_title_copy;
    static Rect rect;
    //znj
    Mat mv[3];
    Mat image_H_title,image_S_title,image_V_title;
    Mat image_hsv;
    Mat image_RGB_temp;
    static int stability_judge_title = 1;

    image_RGB.copyTo(image_RGB_temp);
    cvtColor(image_RGB_temp, image_hsv, CV_BGR2HSV);
    split(image_hsv, mv);

    inRange(mv[0], title_h_L, title_h_H, image_H_title);
    inRange(mv[1], title_s_L, title_s_H, image_S_title);
    inRange(mv[2], title_v_L, title_v_H, image_V_title);

    bitwise_and(image_H_title, image_S_title, image_S_title);
    bitwise_and(image_S_title, image_V_title, image_hsv_title);
    //znj
    //tihuan
    threshold(image_hsv_title,image_hsv_title,150,255,THRESH_BINARY);
    //tihuan
    medianBlur(image_hsv_title,image_hsv_title,3);
    srand((unsigned)time(NULL));
    cv::erode(image_hsv_title, image_hsv_title, cv::Mat(3, 3, CV_8U), cv::Point(-1, -1), 1);
    //cv::erode(image_hsv_title, image_hsv_title, cv::Mat(3, 3, CV_8U), cv::Point(-1, -1), 1);
    image_hsv_title.copyTo(image_hsv_title_copy);
    if(debug_flag)
        imshow("SMG_mask",image_hsv_title);
    cv::findContours(image_hsv_title_copy, original_contours_title, hierarchy_title, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    if (rect_title.size() != 0)
        vector<cv::Rect>().swap(rect_title);
    if (image_slice_checkout_title[4].size() != Size(0,0) )
        image_slice_checkout_title[4].data = 0;

    std::vector<cv::Rect> box;

    for (uint i = 0; i < original_contours_title.size(); i++)
    {
        rect = cv::boundingRect(original_contours_title[i]);
        if (  rect.height > title_hight_L
            &&rect.height < title_hight_H
            && rect.width > title_width_L
            && rect.width < title_width_H
            &&rect.height > rect.width
            &&rect.height < rect.width*6)
        {
            if (rect.x > 4 && rect.x + rect.width  < 632)
            {
                rect.x -= 3;
                rect.width += 4;
            }
            if (rect.y > 4 && rect.y + rect.height < 472)
            {
                rect.y -= 3;
                rect.height += 4;
            }
            box.push_back(rect);
            //cv::rectangle(image_hsv_title, rect, cv::Scalar(0, 255, 0), 2);
        }
    }

    for (uint i = 0; i < box.size(); i++)
    {
        int count = 0;
        for (uint j = 0; j < box.size(); j++)
        {

              if (abs(box[i].height - box[j].height) <SMG_height_difference/10.0
                      && abs(box[i].width - box[j].width) <SMG_width_difference/10.0
                      && abs(box[i].x+box[i].width - box[j].x+box[j].width) < box[i].width*SMG_x_difference/10.0
                      && abs(box[i].y - box[j].y) < SMG_y_difference/10.0)
                count += 1;
        }
        if (count == 5)
        {
            rect_title.push_back(box[i]);
            //cv::rectangle(image_RGB, box[i], cv::Scalar(0, 255, 0), 2);
        }
    }

    if (rect_title.size() == 5)
    {
        stability_judge_title = 20;
        for (int i = 0; i < 5; i++)
            for (int j = 0; j<4 - i; j++)
                if (rect_title[j].x > rect_title[j + 1].x)
                {
                    rect = rect_title[j];
                    rect_title[j] = rect_title[j + 1];
                    rect_title[j + 1] = rect;
                }
        for (uint i = 0; i < rect_title.size(); i++)
        {
            image_slice_checkout_title[i] = cv::Mat(image_hsv_title, rect_title[i]);
            cv::resize(image_slice_checkout_title[i], image_slice_checkout_title[i], cv::Size(28, 28), 0, 0, cv::INTER_LINEAR);
        }
        for (int i = 0; i < 4; i++)
        {
            hconcat(image_slice_checkout_title[i], image_slice_checkout_title[i + 1], image_slice_checkout_title[i + 1]);
        }
    }
    else
    {
        if(adjust_title_flag == 1)
        {
            stability_judge_title --;
        }
        if(stability_judge_title == 0)
        {
            title_v_L ++;
            stability_judge_title = 1;
        }
        if (title_v_L > 255)
            title_v_L = 70;
    }
    return image_slice_checkout_title[4];
}
send_image_data Buff::Handle::image_handle(Mat v4l_frame, camera_map & map_out)
{
    Mat image;
    const int delay=1;
    static double TIME_COST[2];
    TIME_COST[1] =  (double)getTickCount();

    send_image_data return_image_data;
    //Mat frame;
    Mat title_data;
    vector<Mat> context_data;
    //capture >> frame;
    //resize(frame,frame,Size(frame.cols*0.5,frame.rows*2/3),0,0,INTER_LINEAR);
    //remap(frame, frame, map_out.camera_map1, map_out.camera_map2, INTER_LINEAR);
    v4l_frame.copyTo(image);
    //resize(image,image,Size(image.cols*0.5,image.rows*2/3),0,0,INTER_LINEAR);
    if(model_flag == 3)   //small
    {
        context_data = context_finder(image);
        return_image_data.image_flag = 0;
    }
    else if (model_flag == 4)   //big
    {
        context_data = fire_context_finder(image);
        return_image_data.image_flag = 1;
    }

    title_data = title_finder(image);
    circle(image,Point(320,240),3,Scalar(255,0,0));
//    if(debug_flag)
//        imshow("src_image", image);
    if(rectangle_flag==1)
    {
        cv::rectangle(image_RGB_backup, rect_context_backup[draw_target_context], cv::Scalar(0, 255, 0), 2);
        rectangle_flag=0;
    }
    if (rect_context_backup.size() != 0)
        vector<cv::Rect>().swap(rect_context_backup);
    if(debug_flag)
            imshow("src_image", image_RGB_backup);

    //?????????
    if(waitKey(delay)>=0)
        waitKey(0);


    TIME_COST[0] = (double)getTickCount();
    return_image_data.handwriting_image_data = context_data;
    return_image_data.Digital_tube_image_data = title_data;
    return_image_data.handwriting_image_center = rect_center;
    return return_image_data;
}


//angle_analysis :
int Buff::Angle_analysis::angle_analysis(cv::Point2f &Target_point,cv::Point2f &convert_angle)
{
    transf_theta_ = Target_point.x - intrinsic_.at<double>(0, 2);
    transf_theta_ = transf_theta_ / intrinsic_.at<double>(0, 0);
    transf_theta_ = atan(transf_theta_);
    transf_theta_ = -180 * transf_theta_ / M_PI;


    transf_elpha_ = Target_point.y - intrinsic_.at<double>(1, 2);
    transf_elpha_ = transf_elpha_ / intrinsic_.at<double>(1, 1);
    transf_elpha_ = atan(transf_elpha_);
    transf_elpha_ = -180 * transf_elpha_ /M_PI;

    tmp_direction.x = transf_theta_;
    tmp_direction.y = transf_elpha_;

    //    cout << ">>>>>>>>>>>>x_angle :" << tmp_direction.x << endl;
    //    cout << ">>>>>>>>>>>>y_angle :" << tmp_direction.y << endl;

    convert_angle.x=tmp_direction.x;
    convert_angle.y=tmp_direction.y;
    return 0;
}
camera_map Buff::Angle_analysis::camera_matrix_init(Mat v4l_map)
{
    Mat frame;
    //capture.read(frame);
    v4l_map.copyTo(frame);
    intrinsic_.create(3, 3, CV_64FC1);
    distortion_coeff_.create(5, 1, CV_64FC1);
    /*
    [fx 0  cx
     0  fy cy
     0  0  1
     0, 0, 1]

    for the Cook and Base, each RT Matrix for each camera is different.
    */

    intrinsic_.at<double>(0, 0) = 1081.161;
    intrinsic_.at<double>(0, 2) = 319.562;
    intrinsic_.at<double>(1, 1) = 1077.257;
    intrinsic_.at<double>(1, 2) = 240.102;

    intrinsic_.at<double>(0, 1) = 0;
    intrinsic_.at<double>(1, 0) = 0;
    intrinsic_.at<double>(2, 0) = 0;
    intrinsic_.at<double>(2, 1) = 0;
    intrinsic_.at<double>(2, 2) = 1;
    /*
    畸变系数：
    */
    distortion_coeff_.at<double>(0, 0) = -0.663922;   //k1
    distortion_coeff_.at<double>(1, 0) = 0.354435;   //k2
    distortion_coeff_.at<double>(2, 0) = -0.0242489;  //p1
    distortion_coeff_.at<double>(3, 0) = -0.0184213; //p2
    distortion_coeff_.at<double>(4, 0) = 0.256623;    //p3积分矫正

    camera_map map_in;
    Size imageSize;
    imageSize = frame.size();
    initUndistortRectifyMap(intrinsic_, distortion_coeff_, Mat(),
        getOptimalNewCameraMatrix(intrinsic_, distortion_coeff_, imageSize, 1, imageSize, 0),
        imageSize, CV_16SC2, map_in.camera_map1, map_in.camera_map2);
    return map_in;
}

}
