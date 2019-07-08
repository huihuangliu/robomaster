#include "buff.h"
#include "serial_commu.hpp"
#include "Structure.h"

extern int model_flag;
extern double sendData_buff;
extern short sendzero[2]={0,0};

extern int title_h_L;//znj
extern int title_s_L;//znj
extern int title_v_L;//znj
extern int title_h_H;//znj
extern int title_s_H;//znj
extern int title_v_H;//znj
extern int context_h_L;//znj
extern int context_s_L;//znj
extern int context_v_L;//znj
extern int context_h_H;//znj
extern int context_s_H;//znj
extern int context_v_H;//znj
extern int usart_0;

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

    namedWindow("context");
    createTrackbar("context_h_L", "context", &context_h_L, 180);
    createTrackbar("context_h_H", "context", &context_h_H, 180);
    createTrackbar("context_s_L", "context", &context_s_L, 255);
    createTrackbar("context_s_H", "context", &context_s_H, 255);
    createTrackbar("context_v_L", "context", &context_v_L, 255);
    createTrackbar("context_v_H", "context", &context_v_H, 255);
}

void BuffDetector::loopStart(Handle &handle,Angle_analysis &angle)
{
    parameter_adjust();//znj
    static int stability_judge_title2 = 1;
    static int stability_judge_context2 = 1;
    //*********************python初始化***************

    python_init();

    if (!stop_flag_) {
        std::cout << "loopStart stop." << std::endl;
    }
    //const int fd = serial_commu.serial_Init();
    stop_flag_ = false;

    //VideoCapture capture("/home/robomaster/桌面/num1.avi");
    //VideoCapture capture("/home/imagegroup1/桌面/num2.avi");
    //VideoCapture capture("/home/robomaster/桌面/rec_03-31_15-03-57_240.avi");
    //VideoCapture capture(0);
    cv::Mat v4l_frame;
    send_image_data send_data;
    map_out = angle.camera_matrix_init(v4l_frame);

    //capture>>v4l_frame;

    while (!stop_flag_)
    {

        getImageFromMemory(v4l_frame);

        if (v4l_frame.empty())
        {
            std::cout<<"frame is empty!Maybe you need to open V4L!"<<std::endl;
            exit(1);
        }


        //判别buff模式，大 or 小(model_flag),预处理

        //capture>>v4l_frame;
        send_data = handle.image_handle(v4l_frame, map_out);


        if(model_flag==-1)
        {
            destroyAllWindows();
            exit(0);
        }
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
        imshow("context_image_data",hconcat_image);
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
        cout<<"target 寻找:抛入神经网络"<<endl;
        if (PyTuple_GET_SIZE(pValue_num) == 9)
        {
            stability_judge_context2 = 5;
            PyArg_ParseTuple(pValue_num, "i|i|i|i|i|i|i|i|i", &context[0],&context[1],&context[2],&context[3],&context[4],&context[5],&context[6],&context[7],&context[8]);
            for (int i=0;i<9;i++)
                cout<< "context "<<i+1<<":"<<context[i]<<endl;
            //cout<<endl;
        }
        else
        {
            cout<<"num identify failed!"<<"   ";
            stability_judge_context2 --;
            if(stability_judge_context2 == 0)
            {
                context_v_L ++;
                cout<<"context_v_L:"<<context_v_L<<endl;
                stability_judge_context2 = 5;
            }
            continue;
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
            stability_judge_title2 = 5;
            PyArg_ParseTuple(pValue_SMG, "i|i|i|i|i", &SMG[0],&SMG[1],&SMG[2],&SMG[3],&SMG[4]);
            cout<< "the data of SMG is:";
            for (int i=0;i<5;i++)
                cout <<SMG[i]<<' ';

        }
        //
        else
        {
            cout<<"SMG identify failed!"<<"    ";
            stability_judge_title2 --;
            if(stability_judge_title2 == 0)
            {
                title_v_L ++;
                cout<<"title_v_L:"<<title_v_L<<endl;
                stability_judge_title2 = 5;
            }
            continue;
        }


        Py_CLEAR(pList_SMG);
        Py_CLEAR(pArg_SMG);
        Py_CLEAR(pValue_SMG);

        cout<<endl<<"start to find the target context"<<endl;
        target_context=swich_context(context,SMG);

        //角度解析
        vector<cv::Point2f>().swap(context_angle);
        uint lenth = send_data.handwriting_image_center.size();
        for(uint i = 0; i < lenth ; i++)
        {
            tar_center_cur = cv::Point2f(send_data.handwriting_image_center[i]);// 2017.3.6
            angle.angle_analysis(tar_center_cur,convert_angle);
            context_angle.push_back(convert_angle);
        }
        //串口通信
        if(target_context!=9 && target_context!=-1)
        {
            sendData_buff[0]=context_angle[target_context].x * 100;
            sendData_buff[1]=context_angle[target_context].y * 100;
            serial_sendarmor(usart_0, sendData_buff, 2);
            std::cout<<"sending data: "<<sendData_buff[0]<<'\t'<<sendData_buff[1]<<endl;
        }
//        else
//        {
//            serial_commu.serial_send(fd, sendzero);
//        }



        double endPre = (double)cv::getTickCount();
        std::cout << "preprocess time="<<(endPre - startPre) * 1000 / (cv::getTickFrequency()) << "ms" << std::endl <<endl;


    }

    Py_Finalize();
}
void BuffDetector::python_init(){
    Py_Initialize();

    // 检查初始化是否成功
    if (!Py_IsInitialized())
       exit(1);

    PyRun_SimpleString("import sys");
    PyRun_SimpleString("import os");
    PyRun_SimpleString("import ctypes");
    PyRun_SimpleString("sys.path.append('./')");
    PyRun_SimpleString("sys.path.append('/home/robomaster/build-version5-unknown-Debug')");
    PyRun_SimpleString("print(sys.path)");

    pModule = PyImport_ImportModule("identify");

    PyRun_SimpleString("import tensorflow");
    PyErr_Print();

    if (!pModule)
    {
        printf("can't find identify.py");
        getchar();
        exit(1);
    }
    pFunc_SMG = PyObject_GetAttrString(pModule, "run_SMG");
    pFunc_HW = PyObject_GetAttrString(pModule, "run_Hw");
    pFunc_fire = PyObject_GetAttrString(pModule, "run_fire");

    if (!(pFunc_HW&&pFunc_SMG))
    {
        printf("can't find function");
        getchar();
        exit(1);
    }
    if (! (PyCallable_Check(pFunc_HW) && PyCallable_Check(pFunc_SMG)) )
    {
        printf("is not callable");
        getchar();
        exit(1);
    }
}
int BuffDetector::swich_context(vector<int>& context_num, vector<int>&SMG_num)
{
    static int compare_temp=0;
    static int temp_j=0;

    //修改版  概率数字判别
//    if (d_temp != context_num)
//    {
//        if(probability_of_num == 0)
//            d_temp=context_num;
//        else
//            probability_of_num--;
//    }
//    else
//        probability_of_num++;


//    if (s_temp != SMG_num)
//    {
//        if(probability_of_SMG == 0)
//            d_temp=SMG_num;
//        else
//            probability_of_SMG--;
//    }
//    else
//        probability_of_SMG++;


//    if(probability_of_num > THRESHOLDS && probability_of_SMG > THRESHOLDS)
//    {
//        right_times++;
//        compare_temp=SMG_num[right_times];
//        for(temp_j=0;temp_j<9;temp_j++)
//        {
//            if(compare_temp == context_num[temp_j])
//                break;
//        }
//        if(right_times>=4)
//            right_times = -1;

//    }
    if(context_num!=vector<int>(9) && SMG_num!=vector<int>(5))
    {

        if (d_temp != context_num || s_temp != SMG_num )
        {
            if (s_temp != SMG_num)
            {
                right_times=-1;
                if(d_temp==context_num)
                    return -1;
                s_temp=SMG_num;
            }
            right_times++;
            compare_temp=SMG_num[right_times];
            for(temp_j=0;temp_j<9;temp_j++)
            {
                if(compare_temp == context_num[temp_j])
                    break;
            }
            if(right_times>=4)
                right_times = -1;
            d_temp=context_num;
        }
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
    static Mat image_hsv_context,mask,image_hsv_context_copy;
    static std::vector<cv::Rect> rect_context;
    vector<Mat> fire_image_num;
    image_RGB.copyTo(image_hsv_context);

    cvtColor(image_hsv_context, image_hsv_context, CV_RGB2GRAY);
    threshold(image_hsv_context,image_hsv_context,150,255,THRESH_BINARY);
    adaptiveThreshold(image_hsv_context,image_hsv_context,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,13,7);
    medianBlur(image_hsv_context,image_hsv_context,3);
    //double start =getTickCount();
    for(int i = 0; i<4 ; i++)
    {
        int rand_x = random(1,image_hsv_context.cols-1);
        int rand_y = random(1,10);
        floodFill(image_hsv_context,mask,Point(rand_x,rand_y),0);
    }
    //double end =getTickCount();
    //cout << (end - start) * 1000 / (getTickFrequency()) << "ms" << endl;
    image_hsv_context.copyTo(image_hsv_context_copy);
    imshow("fire_image_mask",image_hsv_context);
    cv::findContours(image_hsv_context_copy, original_contours_context, hierarchy_context, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    if (rect_context.size() != 0)
        vector<Rect>().swap(rect_context);

    if (fire_image_num.size() != 0)
        vector<Mat>().swap(fire_image_num);

    if (rect_center.size() != 0)
        vector<Point2f>().swap(rect_center);

    //concave_convex(image_hsv_context, original_contours_context);

    for (uint i = 0; i < original_contours_context.size(); i++)
    {
        cv::Rect rect = cv::boundingRect(original_contours_context[i]);

        if (rect.height > fire_context_hight_L && rect.height < fire_context_hight_H &&
            rect.width > fire_context_width_L && rect.width < fire_context_width_H )
        {
            double area = contourArea(original_contours_context[i]);
            double length = arcLength(original_contours_context[i],1);
            double isfire = area/(rect.width*rect.height);

            if (!((isfire>0.3)&&(isfire<0.9)))
                continue;
            if (rect.x > 2 && rect.x + rect.width  < 636)
            {
                rect.x -= 2;
                rect.width += 4;
            }
            if (rect.y > 2 && rect.y + rect.height < 476)
            {
                rect.y -= 2;
                rect.height += 4;
            }
            rect_context.push_back(rect);
            cv::rectangle(image_RGB, rect, cv::Scalar(0, 0, 255), 2);
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
        for (uint i = 0; i < rect_context.size(); i++)
        {
            image_slice_checkout_context[i] = cv::Mat(image_hsv_context, rect_context[i]);
            cv::resize(image_slice_checkout_context[i], image_slice_checkout_context[i], cv::Size(28, 28), 0, 0, cv::INTER_LINEAR);
            fire_image_num.push_back(image_slice_checkout_context[i]);
        }
    }
    return fire_image_num;
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
              if (abs(box[i].height - box[j].height) <15
                      && abs(box[i].width - box[j].width) <15
                      && abs(box[i].x - box[j].x) < box[i].width*4
                      && abs(box[i].y - box[j].y) < box[i].height*4)
                count += 1;
        }
        if (count == 9)
        {
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
        stability_judge_context = 100;
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
        stability_judge_context --;
        if(stability_judge_context == 0)
        {
            context_v_L ++;
            stability_judge_context = 1;
            cout<<"context_v_L:"<<context_v_L<<endl;
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
    image_hsv_title.copyTo(image_hsv_title_copy);
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
                rect.x -= 2;
                rect.width += 4;
            }
            if (rect.y > 4 && rect.y + rect.height < 472)
            {
                rect.y -= 2;
                rect.height += 4;
            }
            box.push_back(rect);
            cv::rectangle(image_hsv_title, rect, cv::Scalar(0, 255, 0), 2);
        }
    }

    for (uint i = 0; i < box.size(); i++)
    {
        int count = 0;
        for (uint j = 0; j < box.size(); j++)
        {

              if (abs(box[i].height - box[j].height) <7
                      && abs(box[i].width - box[j].width) <20
                      && abs(box[i].x+box[i].width - box[j].x+box[j].width) < box[i].width*9
                      && abs(box[i].y - box[j].y) < 10)
                count += 1;
        }
        if (count == 5)
        {
            rect_title.push_back(box[i]);
            cv::rectangle(image_RGB, box[i], cv::Scalar(0, 255, 0), 2);
        }
    }

    if (rect_title.size() == 5)
    {
        stability_judge_title = 10;
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
            hconcat(image_slice_checkout_title[i], image_slice_checkout_title[i + 1], image_slice_checkout_title[i + 1]);;
        }
    }
    else
    {
        stability_judge_title --;
        if(stability_judge_title == 0)
        {
            title_v_L ++;
            stability_judge_title = 1;
            cout<<"title_v_L:"<<title_v_L<<endl;
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
    if(model_flag == 3)  //small buff
    {
        context_data = context_finder(image);
        return_image_data.image_flag = 0;
    }
    else if (model_flag == 4)   //big buff
    {
        context_data = fire_context_finder(image);
        return_image_data.image_flag = 1;
    }

    title_data = title_finder(image);
    circle(image,Point(320,240),3,Scalar(255,0,0));
    imshow("src_image", image);

    //?????????
    if(waitKey(delay)>=0)
        waitKey(0);


    TIME_COST[0] = (double)getTickCount();
    //cout<< "image handle cost time: " <<1000*(TIME_COST[0]-TIME_COST[1]) / getTickFrequency()<<"ms"<<std::endl;
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
//    cout<<"Target_point.x :"<<Target_point.x <<endl;
//    cout<<"Target_point.y :"<<Target_point.y <<endl;
//    cout<<"transf_theta_ :"<<transf_theta_ <<endl;
//    cout<<"transf_elpha_ :"<<transf_elpha_ <<endl;
    //senddata0水平偏角，senddata1俯仰角，senddata2距离，都是乘100后发送
    /*
    sendData[0] = tmp_direction.x * 100;
    sendData[1] = tmp_direction.y * 100;
    sendData[2] = pow((4568.9 / D_height), 1/0.95);
    if(sendData[2]>500)
        sendData[2]=1000;*/
    //    cout<<"D_height"<<D_height<<endl;
    //cout<<"distance"<<sendData[2]<<endl;
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
