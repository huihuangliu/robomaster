#include "armor.h"
#include "serial_commu.hpp"
#include "Structure.h"

using namespace std;
using namespace cv;

extern int model_flag;
extern int usart_0;
extern double sendData_armor;

#define DEBUG
namespace armor {

int D_height=0;

void ArmorDetector::loopStart(Shooter &shooter,VideoWriter &out)
{
    //    if (!stop_flag_) {
    //        std::cout << "loopStart stop." << std::endl;
    //    }
    //stop_flag_ = false;

    do
    {
#if debug
        waitKey(0);
#endif

        // 读取图像
        cv::Mat frame;
        cv::Mat mask,show_srcImage;
        //video >> frame;
        getImageFromMemory(frame);

        if (frame.empty())
        {
            std::cout<<"frame is empty!Maybe you need to open V4L!"<<std::endl;
            continue;
        }
        out << frame;
        // 预处理
        double startPre = (double)cv::getTickCount();

        this->preprocessImg(frame, mask,show_srcImage);

        double endPre = (double)cv::getTickCount();
        //std::cout << "preprocess time="<<(endPre - startPre) * 1000 / (cv::getTickFrequency()) << "ms" << std::endl;


        //查找目标并跟踪
        //double startFind = (double)cv::getTickCount();

        this->findTargets(frame,mask,show_srcImage);
        this->target_track(frame,show_srcImage);
        //double endFind = (double)cv::getTickCount();
        //std::cout << "findTarget time="<<(endFind - startFind) * 1000 / (cv::getTickFrequency()) << "ms" << std::endl;
#ifdef DEBUG
        //debug show
        if(show_srcImage.cols>0&&mask.cols>0)
        {
            imshow("show",show_srcImage);
            imshow("mask",mask);
        }
        //std::cout<<"目标中心像素点坐标"<<target_pixel.x<<'\t'<<target_pixel.y<<std::endl;
#endif
        cv::Point target;
        target.x=target_pixel.x;
        target.y=target_pixel.y;
        //运动预测
        //shooter.motion_predict(target);

        //定义目标角度
        cv::Point2f convert_angle;
        double Distance;
        //角度解析
        shooter.angle_analysis(target,convert_angle,Distance);

        //std::cout<<"目标中心解析角度"<<convert_angle.x<<'\t'<<convert_angle.y<<std::endl;
        double sendZero[] = {0,0,0};
        //发送串口数据

        if(get_one==1)
        {
            sendData_armor[0] = convert_angle.x * 100;
            sendData_armor[1] = convert_angle.y * 100;

            ///距离函数在这里改
            sendData_armor[2] = Distance;
            if(sendData_armor[2]>500)
                sendData_armor[2]=1000;
            serial_sendarmor(usart_0, sendData_armor, 3);
            std::cout<<"sending data: "<<sendData_armor[0]<<'\t'<<sendData_armor[1]<<'\t'<<sendData_armor[2]<<std::endl;

        }
//        else
//        {
//            serial_commu.serial_sendarmor(usart_0, sendZero, 3);
//        }

    }
    while(model_flag != -1);

    if(model_flag==-1)
    {
        destroyAllWindows();
    }
}

int armor::ArmorDetector::preprocessImg(cv::Mat &src,cv::Mat &mask,cv::Mat &show) {

    if (src.channels() != 3) {
        return -1;
    }
    src.copyTo(show);
    //cv::blur(src, src, cv::Size(3, 3), cv::Point(-1, -1), cv::BORDER_DEFAULT);

    cv::Point center;
    center.x=src.cols/2;
    center.y=src.rows/2;
    circle(show, center, 3, cv::Scalar(0, 255, 255), 2);



    _src = src;

    //缩小搜索区域

    //    const cv::Point last_result = _res_last.center;
    //    if (last_result.x == 0 || last_result.y == 0)
    //    {
    //        //如果搜索框清0或者失帧大于30,进入全局搜索
    //        if (((_dect_rect.width&&_dect_rect.height) == 0)||(lost_frame >= const_lost_frame))
    //        {
    //            _src = src;
    //            _dect_rect = Rect(0, 0, src.cols, src.rows);
    //            src(_dect_rect).copyTo(_src);
    //        }
    //        else
    //        {
    //            src(_dect_rect).copyTo(_src);
    //        }
    //        lost_frame++;
    //    }
    //    //缩小搜索框
    //    else
    //    {
    //        lost_frame = 0;
    //        Rect res_rect = _res_last.boundingRect();

    //        double scale;
    //        if(res_rect.width<100)
    //            scale = 4;
    //        else
    //            scale = 3;

    //        int exp_half_w = min(max_half_w / 2, int(res_rect.width * scale));
    //        int exp_half_h = min(max_half_h / 2, int(res_rect.height * scale));

    //        int w = min(max_half_w, exp_half_w);
    //        int h = min(max_half_h, exp_half_h);
    //        Point center = last_result;
    //        int x = max(center.x - w, 0);
    //        int y = max(center.y - h, 0);
    //        lu = Point(x, y);
    //        up_left=lu;
    //        x = min(center.x + w, src.cols);
    //        y = min(center.y + h, src.rows);
    //        rd = Point(x, y);
    //        _dect_rect = Rect(lu, rd);
    //        if (ArmorDetector::makeRectSafe(_dect_rect, src.size()) == false)
    //        {
    //            _res_last = cv::RotatedRect();
    //            _dect_rect = Rect(0, 0, src.cols, src.rows);
    //            _src = src;
    //        }
    //        else
    //            src(_dect_rect).copyTo(_src);
    //    }

    //待修改的预处理
    //    cv::threshold(_src,_src,100,255,THRESH_BINARY);
    //    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    //    Mat imagehsv,imagergb,hsv_mask,maskImage;

    //    cvtColor(_src, imagehsv, CV_BGR2HSV);
    //    if (enemyColor == 0)
    //    {
    //        cv::inRange(imagehsv, cv::Scalar(40, 0, 50), cv::Scalar(180, 255, 255), hsv_mask);
    //        cv::dilate(hsv_mask, hsv_mask, element, cv::Point(-1, -1), 1);
    //        cv::inRange(_src, cv::Scalar(0, 0, 200), cv::Scalar(255, 255, 255), imagergb);
    //        cv::dilate(imagergb, imagergb, element, cv::Point(-1, -1), 1);
    //        cv::bitwise_and(hsv_mask, imagergb, maskImage);
    //        imshow("hsv_mask",hsv_mask);
    //        imshow("imagergb",imagergb);
    //    }
    //    if (enemyColor == 1)
    //    {
    //        cv::inRange(imagehsv, Scalar(78, 15, 80), Scalar(145, 255, 255), hsv_mask);
    //        cv::dilate(hsv_mask, hsv_mask, element, cv::Point(-1, -1), 1);
    //        cv::inRange(_src, cv::Scalar(200, 0, 0), cv::Scalar(255, 200, 200), imagergb);
    //        cv::dilate(imagergb, imagergb, element, cv::Point(-1, -1), 1);
    //        cv::bitwise_or(hsv_mask, imagergb, maskImage);
    //    }
    //    //腐蚀膨胀
    //    Mat elem1ent1 = getStructuringElement(MORPH_RECT, Size(2, 2));
    //    Mat element2 = getStructuringElement(MORPH_RECT, Size(2, 1));
    //    erode(maskImage, maskImage, element2, cv::Point(-1, -1), 1);
    //    dilate(maskImage, maskImage, element1, cv::Point(-1, -1), 1);
    //    mask=maskImage;
    //参考预处理
    cv::Mat imagehsv,hsv_mask,_max_color;
    cv::cvtColor(_src, imagehsv, CV_BGR2HSV);
    cv::Mat element_rgb = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1,1));

    int total_pixel = _src.cols * _src.rows;
    const uchar * ptr_src = _src.data;
    const uchar * ptr_src_end = _src.data + total_pixel * 3;

    _g.create(_src.size(), CV_8UC1);
    _ec.create(_src.size(), CV_8UC1);
    _rgb_color = cv::Mat(_src.size(), CV_8UC1, cv::Scalar(0));
    uchar *ptr_g = _g.data, *ptr_ec = _ec.data, *ptr_rgb_color = _rgb_color.data;
    if (model_flag == 0)  //red
    {
        std::cout <<"model_flag: " << model_flag << std::endl;
        for (; ptr_src != ptr_src_end; ++ptr_src, ++ptr_g, ++ptr_rgb_color, ++ptr_ec)
        {
            uchar b = *ptr_src;
            uchar g = *(++ptr_src);
            uchar r = *(++ptr_src);
            *ptr_g = g;
            *ptr_ec = r;
            //*ptr_g = b;
            if (r > min_light_gray)
                *ptr_rgb_color = 255;
            //            if (r - b > _para.br_threshold && r >= g)
            //                *ptr_max_color = 255;
        }
        cv::Mat element_hsv_r = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 3));
        cv::inRange(imagehsv, cv::Scalar(150, 40, 40), cv::Scalar(180, 255, 255), hsv_mask);
        cv::dilate(hsv_mask, hsv_mask, element_hsv_r, cv::Point(-1, -1), 1);
        cv::dilate(_rgb_color, _rgb_color, element_rgb, cv::Point(-1, -1), 1);

        cv::bitwise_and(hsv_mask, _rgb_color, _max_color);
    }
    else if (model_flag == 1)   //blue
    {
        for (; ptr_src != ptr_src_end; ++ptr_src, ++ptr_g, ++ptr_rgb_color, ++ptr_ec)
        {
            uchar b = *ptr_src;
            uchar g = *(++ptr_src);
            uchar r = *(++ptr_src);
            *ptr_g = g;
            *ptr_ec = b;
            //*ptr_g = r;
            if (b > 200)
                *ptr_rgb_color = 255;
        }
        cv::Mat element_hsv_b = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
        cv::Mat element_hsv_erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(12, 10));
        cv::inRange(imagehsv, cv::Scalar(90, 45, 40), cv::Scalar(135, 255, 255), hsv_mask);
        cv::dilate(hsv_mask, hsv_mask, element_hsv_b, cv::Point(-1, -1), 1);
        cv::erode(hsv_mask, hsv_mask, element_hsv_erode, cv::Point(-1, -1), 1);
        cv::dilate(_rgb_color, _rgb_color, element_rgb, cv::Point(-1, -1), 1);
        cv::bitwise_and(hsv_mask, _rgb_color, _max_color);
    }

    //        cv::imshow("hsv_mask",hsv_mask);
    //        cv::imshow("_rgb_color", _rgb_color);

    mask=_max_color;
    cv::waitKey(1);

    return 0;
}

int armor::ArmorDetector::findTargets(cv::Mat &src, cv::Mat &mask,cv::Mat &show) {
    cv::Mat mask_copy;

    mask.copyTo(mask_copy);
    //cv::imshow("mask",mask);
    getTarget = 0;
    std::vector<std::vector<cv::Point> >contours,contours0,contours1, contours2, contours3, contours4;

    findContours(mask_copy, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));


    for (size_t i = 0; i < contours.size(); i++)
    {
        float d[4];
        float height, width;
        cv::RotatedRect rect = cv::minAreaRect(contours[i]);
        cv::Rect rect_bound = cv::boundingRect(contours[i]);
        cv::Point2f vertex[4];
        rect.points(vertex);
        //Point center=rect.center;

        ///第一次用高大于宽筛选
        if (rect_bound.height>rect_bound.width)
        {
            for (int i = 0; i<4; i++)
            {
                d[i] = dist(vertex[i].x, vertex[i].y, vertex[(i + 1) % 4].x, vertex[(i + 1) % 4].y);
            }

            float angle_light = angle(vertex[0].x, vertex[0].y, vertex[1].x, vertex[1].y);
            if (d[1]>d[0])
            {
                height = d[1];
                width = d[0];
            }
            else
            {
                height = d[0];
                width = d[1];
            }
            if (angle_light<min_angle
                    && contours[i].size() > min_contours_size
                    &&height>min_contour_height)
            {
                contours0.push_back(contours[i]);
            }
        }
    }

    if(contours0.size()>=2)
    {
        int k=0;
        int h;
        int y[contours0.size()];
        for (uint i = 0; i < contours0.size(); i++)
        {
            y[i]=1;
            cv::RotatedRect rect_i = cv::minAreaRect(contours0[i]);
            cv::Rect rect_h=cv::boundingRect(contours0[i]);
            h=rect_h.height+1;
            int y_i=rect_i.center.y;
            for (uint j = i+1; j < contours0.size(); j++)
            {
                RotatedRect rect_j = minAreaRect(contours0[j]);
                int y_j=rect_j.center.y;
                if(abs(y_i-y_j)<=h)
                    y[i]+=1;
            }
        }

        int max_y=0;
        for (uint i=0;i<contours0.size();i++)
            if(max_y<y[i])
            {
                max_y=y[i];
                k=i;
            }

        RotatedRect rect_k = minAreaRect(contours0[k]);
        cv::Rect rect_one=cv::boundingRect(contours0[k]);
        int h_one=rect_one.height;
        int y_k=rect_k.center.y;
        for (uint i=0; i < contours0.size(); i++)
        {
            RotatedRect rect_i = minAreaRect(contours0[i]);
            int y_i=rect_i.center.y;
            if(abs(y_k-y_i)<=h_one)
                contours1.push_back(contours0[i]);
        }
    }
    cout<<"contours1.size()="<<contours1.size()<<endl;

//    for (uint i=0; i < contours1.size(); i++)
//    {
//        Rect rect_b=boundingRect(contours1[i]);
//        rectangle(show,rect_b,Scalar(255,0,255),2,8);
//    }

    if(contours1.size()>=2)
    {
        if(contours1.size()==2)
        {
            //int x_center[2], contours_width;
            for (uint i=0; i < contours1.size(); i++)
            {
                contours2.push_back(contours1[i]);
                //RotatedRect rect_0 = minAreaRect(contours1[i]);
                //x_center[i] = rect_0.center.x;

            }
//            contours_width = abs(x_center[0] - x_center[1]);
//            cout<<"center_width:" << contours_width <<endl;   // 80 for yingxiong
//            for (uint i=0; i < contours1.size(); i++)
//            {
//                if(contours_width<80)
//                {
//                    contours2.push_back(contours1[i]);
//                }
//            }
        }
        if(contours1.size()==3)
        {
            int x_i[3],x_mid;
            for (uint i = 0; i < 3; i++)
            {
                RotatedRect rect_i = minAreaRect(contours1[i]);
                x_i[i]=rect_i.center.x;
            }
            int average_x=(x_i[0]+x_i[1]+x_i[2])/3;
            int min_i=min(min(x_i[0],x_i[1]),x_i[2]);
            int max_i=max(max(x_i[0],x_i[1]),x_i[2]);
            for (uint i = 0; i < 3; i++)
            {
                if((x_i[i]!=min_i)&&(x_i[i]!=max_i))
                {
                    x_mid=x_i[i];
                    contours2.push_back(contours1[i]);
                }
            }
            int small_i=0;
            int big_i=0;
            for (uint i = 0; i < 3; i++)
            {
                if(x_i[i]!=x_mid)
                {
                    if(x_i[i]<x_mid)
                        small_i=i;
                    else
                        big_i=i;
                }
            }

            if(abs(x_i[small_i]-x_mid)<abs(x_i[big_i]-average_x))
                contours2.push_back(contours1[small_i]);
            else
                contours2.push_back(contours1[big_i]);
        }

        if(contours1.size()==4)
        {
            int x_i[4];
            for (uint i = 0; i < 4; i++)
            {
                RotatedRect rect_i = minAreaRect(contours1[i]);
                x_i[i]=rect_i.center.x;
            }
            int min_i=min(min(x_i[0],x_i[1]),min(x_i[2],x_i[3]));
            int max_i=max(max(x_i[0],x_i[1]),max(x_i[2],x_i[3]));
//            for(int i=0;i<4;i++)
//                for(int j=i+1 ;j<4;j++)
//            {
//                if(x_i[i]>x_i[j])
//                    swap(x_i[i],x_i[j]);
//            }
//            for(int i=0;i<4;i++)
//                std::cout << "x_i "<<  x_i[i]  << std::endl;

            for (uint i = 0; i < 4; i++)
            {
                RotatedRect rect_ = minAreaRect(contours1[i]);
                int x_=rect_.center.x;
                if((abs(x_-min_i)>2)&&(abs(x_-max_i)>2))
                    contours2.push_back(contours1[i]);
            }
        }

        if(contours1.size()>=5)
        {
            int x_i[contours1.size()];
            int ave=0;
            int height_5=0;
            for (uint i = 0; i < contours1.size(); i++)
            {
                RotatedRect rect_i = minAreaRect(contours1[i]);
                Rect rect_b=boundingRect(contours1[i]);
                height_5+=rect_b.height;
                x_i[i]=rect_i.center.x;
                ave+=x_i[i];
            }
            ave=ave/contours1.size();
            height_5=height_5/contours1.size();

            for (uint i = 0; i < contours1.size(); i++)
            {
                RotatedRect rect_i = minAreaRect(contours1[i]);
                if(abs(rect_i.center.x-ave)<height_5*4)
                    contours2.push_back(contours1[i]);
            }
        }
    }

    std::cout<<"contours2="<<contours2.size()<<std::endl;
//****************************************filter 3*******************************************
    for (uint i=0; i < contours2.size(); i++)
    {
        Rect rect_b=boundingRect(contours1[i]);
        rectangle(show,rect_b,Scalar(0,0,255),3,8);
    }

    //遍历不同灯条，选择符合装甲板特征的灯条轮廓
    if(contours2.size()>=2)
    {
        float box_up_angle = 0, box_down_angle = 0, box_left_angle = 0, box_right_angle = 0;
        float delta_up_down_angle = 0;
        float delta_left_right_angle = 0;

        for (size_t i = 0; i < contours2.size(); i++)
        {
            cv::RotatedRect rect_i = cv::minAreaRect(contours2[i]);
            cv::Point2f vertice_i[4];
            rect_i.points(vertice_i);
            cv::Point2f up_point_i = vertice_i[0], down_point_i = vertice_i[0];

            for (size_t j = i + 1; j < contours2.size(); j++)
            {
                cv::RotatedRect rect_j = cv::minAreaRect(contours2[j]);
                cv::Point2f vertice_j[4];
                rect_j.points(vertice_j);
                cv::Point2f up_point_j = vertice_j[0], down_point_j = vertice_j[0];

                for (size_t m = 0; m < 4; m++)
                {
                    if (up_point_i.y > vertice_i[m].y)
                        up_point_i = vertice_i[m];
                    if (down_point_i.y < vertice_i[m].y)
                        down_point_i = vertice_i[m];
                }
                for (int n = 0; n < 4; n++)
                {
                    if (up_point_j.y > vertice_i[n].y)
                        up_point_j = vertice_j[n];
                    if (down_point_j.y < vertice_i[n].y)
                        down_point_j = vertice_j[n];
                }

                double height_i = std::max(rect_i.size.height, rect_i.size.width);
                double height_j = std::max(rect_j.size.height, rect_j.size.width);
                double distance_center = dist(rect_i.center.x, rect_i.center.y, rect_j.center.x, rect_j.center.y);
                double averageHeight = (height_i + height_j) / 2;

                if ((up_point_i.x - up_point_j.x) == 0)
                    box_up_angle = 0;
                else
                    box_up_angle = angle(up_point_i.x, up_point_i.y, up_point_j.x, up_point_j.y);

                if ((down_point_i.x - down_point_j.x) == 0)
                    box_down_angle = 0;
                else
                    box_up_angle = angle(down_point_i.x, down_point_i.y, down_point_j.x, down_point_j.y);

                if (vertice_i[0].x - vertice_i[1].x == 0)
                    box_left_angle = 0;
                else
                    box_left_angle = angle(vertice_i[0].x, vertice_i[0].y, vertice_i[1].x, vertice_i[1].y);

                if (vertice_j[0].x - vertice_j[1].x == 0)
                    box_right_angle = 0;
                else
                    box_right_angle = angle(vertice_j[0].x, vertice_j[0].y, vertice_j[1].x, vertice_j[1].y);

                delta_up_down_angle = abs(box_up_angle - box_down_angle);
                delta_left_right_angle = abs(box_left_angle - box_right_angle);
#if filter3
        std::cout << "angle(up_point_i.x,up_point_i.y,down_point_i.x,down_point_i.y): " << angle(up_point_i.x,up_point_i.y,down_point_i.x,down_point_i.y)<<" angle(up_point_j.x,up_point_j.y,down_point_j.x,down_point_j.y): "<< angle(up_point_j.x,up_point_j.y,down_point_j.x,down_point_j.y) <<std::endl;
#endif
                bool limit;
                if((angle(up_point_i.x,up_point_i.y,down_point_i.x,down_point_i.y)==0)&&(angle(up_point_j.x,up_point_j.y,down_point_j.x,down_point_j.y)<-14)
                        ||(angle(up_point_i.x,up_point_i.y,down_point_i.x,down_point_i.y)>=14)&&(angle(up_point_j.x,up_point_j.y,down_point_j.x,down_point_j.y)==0))
                {
                    limit=true;
                }
                else
                    limit =false;
#if filter3
        std::cout << "abs(box_up_angle): " << abs(box_up_angle)<<" <max_up_down_angle: "<< max_up_down_angle <<std::endl;
        std::cout << "abs(box_down_angle): " << abs(box_down_angle)<<" <max_up_down_angle: "<< max_up_down_angle  <<std::endl;
        std::cout << "0.4< "<<"height_i / height_j: " << height_i / height_j<<"<2.2"<<std::endl;
        std::cout << "abs(up_point_i.x-up_point_j.x) "<<abs(up_point_i.x-up_point_j.x) << "> averageHeight "<<averageHeight<<std::endl;
        std::cout << "abs(abs(up_point_i.x-up_point_j.x)-abs(down_point_i.x-down_point_j.x))/ height_i "<<abs(abs(up_point_i.x-up_point_j.x)-abs(down_point_i.x-down_point_j.x))/ height_i << "<2 "<<std::endl;
         std::cout <<"2<" <<"distance_center/averageHeight "<< distance_center/averageHeight<< " <10"<<std::endl;
          std::cout << " delta_up_down_angle "<< delta_up_down_angle<< " <min_angle  "<<min_angle<<std::endl;
           std::cout << " delta_left_right_angle "<< delta_left_right_angle<< " <15"<<std::endl;
            std::cout << " rect_i.angle "<<rect_i.angle << " rect_j.angle "<<rect_j.angle<<std::endl;
#endif
                ///第二次用角度，两灯条高度比，中心距，上下左右对称角度进行筛选，参数可调
                if (
                        abs(box_up_angle) <max_up_down_angle//big 15 little 17
                        &&abs(box_down_angle) <max_up_down_angle
                        &&height_i / height_j<1.8
                        &&height_i / height_j>0.6
                        &&abs(up_point_i.x-up_point_j.x)>averageHeight
                        &&abs(abs(up_point_i.x-up_point_j.x)-abs(down_point_i.x-down_point_j.x))<height_i*2
                        &&distance_center>averageHeight*2
                        &&averageHeight*10>distance_center
                        && delta_up_down_angle <min_angle
                        && (delta_left_right_angle <15)//LITTLE15 BIG10
                        &&((rect_i.angle+45)*(rect_j.angle+45)>=0)
                        &&limit==false
                        //&&contours_width < 50

                    )
                    //&&(!(angle(up_point_i.x,up_point_i.y,down_point_i.x,down_point_i.y)=0&&angle(up_point_j.x,up_point_j.y,down_point_j.x,down_point_j.y)<-17))
                {

                    //                    std::cout<<"box_left_angle"<<box_left_angle<<"    "<<box_right_angle<<std::endl;
                    //                    std::cout<<"rect_i"<<rect_i.angle<<"    "<<rect_j.angle<<std::endl;
                                     std::cout<<"*****************************************************angle1: "
                                             <<angle(up_point_i.x,up_point_i.y,down_point_i.x,down_point_i.y)<<" angle2: "
                                            <<angle(up_point_j.x,up_point_j.y,down_point_j.x,down_point_j.y)<<std::endl;
                    std::vector<cv::Point> contours_temp_target;
                    int contours_i_size = contours2[i].size();
                    int contours_j_size = contours2[j].size();
                    contours_temp_target.resize(contours_i_size + contours_j_size);

                    for (size_t m = 0; m < contours_i_size; m++)
                        contours_temp_target[m] = contours2[i][m];
                    for (size_t n = 0; n < contours_j_size; n++)
                        contours_temp_target[n + contours_i_size] = contours2[j][n];
                    contours3.push_back(contours_temp_target);

                }
            }
        }
    }
    std::cout<<"contours3="<<contours3.size()<<std::endl;
    //***************************************filter 4 *************************************************
    //备选装甲板进一步筛选
    if(contours3.size()>=1)
    {
        cv::Rect rect3;
        cv::Mat copy_image,gray_image,threshold_image;
        for (size_t i = 0; i < contours3.size(); i++)
        {
            if(lost_frame<const_lost_frame)
            {
                rect3 = boundingRect(contours3[i])+up_left;
            }
            else
            {
                rect3 = boundingRect(contours3[i]);
            }
            if (makeRectSafe(rect3, src.size()) == false)
            {
                rect3 = cv::Rect(0, 0, src.cols, src.rows);
            }
            else
            {
                src(rect3).copyTo(copy_image);
            }

            cv::cvtColor(copy_image,gray_image,cv::COLOR_RGB2GRAY,0);
            cv::threshold(gray_image,threshold_image, 100, 255,cv::THRESH_BINARY);
            standard_template.create(gray_image.rows,gray_image.cols,gray_image.type());

            makeTemplate(standard_template.cols,standard_template.rows,standard_template);
            //            imshow("standard_template",standard_template);
            //            imshow("threshold",threshold_image);
            double a=brightSums(standard_template);
            double b=brightSums(threshold_image);

            double remind=b/a;

            ///相似度筛选
#if filter4
            std::cout << " remind "<<remind << std::endl;
#endif
            if(remind>0.3&&remind<4)
                if(threshold_image.cols>threshold_image.rows)
                {
                    contours4.push_back(contours3[i]);
                }
        }
    }
    std::cout<<"contours4="<<contours4.size()<<std::endl;
    if(contours4.size()>=1&&contours4.size()<3)
    {

//        //选择面积最大的装甲板
//        vector<double> area;
//        double biggest_area=0,biggest_width=0,biggest_height=0;
//        area.resize(contours4.size());

//        for (uint i = 0; i<contours4.size(); i++)
//        {
//            area[i] = contourArea(contours4[i]);
//            Rect rect_bo = boundingRect(contours4[i]);
//            if(rect_bo.width>biggest_width)
//                biggest_width=rect_bo.width;
//            if(rect_bo.height>biggest_height)
//                biggest_height=rect_bo.height;
//        }

//        for (uint i = 0; i < contours4.size(); i++)
//        {
//            Rect rect_bo = boundingRect(contours4[i]);
//            if(
//                    //area[i]>biggest_area
//                    //&&rect_bo.width==biggest_width
//                    rect_bo.height==biggest_height)
//            {
//                biggest_area=area[i];
//                //            biggest_height=bo_rect[i].height;
//                //            biggest_width=bo_rect[i].width;
//                contours_result=contours4[i];
//                getTarget = 1;
//            }
//        }

        //选择倾斜角最小的装甲板
        std::vector<double> angle_i;
        double smallest_angle=45;
        angle_i.resize(contours4.size());

        for (size_t i = 0; i<angle_i.size()+1; i++)
        {
            cv::RotatedRect rect_ro = cv::minAreaRect(contours2[i]);
            angle_i[i]=rect_ro.angle;
            if(angle_i[i]>45)
                angle_i[i]=abs(angle_i[i]-90);
            if(angle_i[i]<-45)
                angle_i[i]=abs(angle_i[i]+90);
            if(angle_i[i]<=0)
                angle_i[i]=abs(angle_i[i]);
            //std::cout<<"angle_i="<<angle_i[i]<<std::endl;
        }

        for (size_t i = 0; i < angle_i.size(); i++)
        {
            if(angle_i[i]<smallest_angle)
            {
                smallest_angle=angle_i[i];
                contours_result=contours4[i];
                getTarget = 1;
            }
        }

    }

    //get target!
    if (getTarget == 1)
    {
        //get 继续打
        continue_shot = 1;
        //        if((abs(vertex_result[0].x-vertex_result[1].x)>200)||
        //                (abs(vertex_result[0].x-vertex_result[2].x)>200))
        //        {
        //            getTarget = 0;
        //        }

        cv::RotatedRect rect_result = cv::minAreaRect(contours_result);
        cv::Rect bounding_rect =cv::boundingRect(contours_result);
        track_delta_wtidth=bounding_rect.width;
        std::vector <cv::Point2f>_res_last_point;
        rect_result.points(vertex_result);

        float dis[3];
        dis[0]=dist(vertex_result[0].x,vertex_result[0].y,vertex_result[1].x,vertex_result[1].y);
        dis[1]=dist(vertex_result[0].x,vertex_result[0].y,vertex_result[2].x,vertex_result[2].y);
        dis[2]=dist(vertex_result[0].x,vertex_result[0].y,vertex_result[3].x,vertex_result[3].y);

        int min_dist=0;
        if(dis[0]>dis[1])
            min_dist=dis[1];
        else
            min_dist=dis[0];
        if(dis[2]<min_dist)
            min_dist=dis[2];
        //最小距离
        D_height=min_dist;
        //全局搜索
        if (lost_frame >= const_lost_frame)
        {
            _res_last = cv::RotatedRect();

            for (size_t i = 0; i<4; i++)
                _res_last_point.push_back(vertex_result[i]);
            _res_last = minAreaRect(_res_last_point);

            for (size_t i = 0; i<4; i++)
            {
                line(show, vertex_result[i], vertex_result[(i + 1) % 4], cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
            }
        }

        //局部搜索
        else
        {
            for (int i = 0; i<4; i++)
                _res_last_point.push_back(vertex_result[i] + lu);
            _res_last = minAreaRect(_res_last_point);

            for (int line_j = 0; line_j <4; line_j++)
                line(show, vertex_result[line_j] + lu, vertex_result[(line_j + 1) % 4] + lu, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
            cv::Rect rect_test = cv::boundingRect(_res_last_point);
            //cout<<rect_test.width<<endl;
            if (rect_test.width==0)
                lost_frame++;
        }

        if(bounding_rect.width<400)
        {
            target_pixel.x = _res_last.center.x;
            target_pixel.y = _res_last.center.y;
            last_point= target_pixel;
            circle(show, target_pixel, 3, cv::Scalar(0, 255, 0), 2);
            get_one=1;
        }
    }
    else
    {
        get_one=0;
        getTarget = 0;
        continue_shot = 0;
        _res_last = cv::RotatedRect();
    }
    return 0;
}

int armor::ArmorDetector::target_track(cv::Mat &src,cv::Mat &show)
{
    cv::Mat new_srcImage;
    std::cout<<"track"<<std::endl;
    src.copyTo(new_srcImage);
    //Whether Recognize or not
    Is_recognizing = continue_shot;

    std::vector<cv::Point> track_points;

    int result_cols = 0;
    int result_rows = 0;

    unsigned char march_target_flag = 0;

    if (Is_recognizing == 1)
    {
        TargetGet_times++;
        TargetLost_times = 0;

        if (lost_frame >= const_lost_frame)
        {
            for (int i = 0; i<4; i++)
                track_points.push_back(vertex_result[i]);
        }
        else
        {
            for (int i = 0; i<4; i++)
                track_points.push_back(vertex_result[i] + lu);
        }

        track_window = cv::boundingRect(track_points);

        //rectangle(show_src, track_window, Scalar(0,255, 0), 2, 8, 0);

        if (makeRectSafe(track_window, new_srcImage.size()) == false)
        {
            track_window = cv::Rect(0, 0, new_srcImage.cols, new_srcImage.rows);
        }

        new_srcImage(track_window).copyTo(track_Image);

        result_cols = new_srcImage.cols - track_Image.cols + 1;
        result_rows = new_srcImage.rows - track_Image.rows + 1;

        result = cv::Mat(result_cols, result_rows, CV_32FC1);

    }
    else
    {
        track_image_march = track_Image;
        TargetGet_times = 0;
        TargetLost_times++;

        ///模板匹配搜索框大小，参数可调
        int max_half_w = 400;
        int max_half_h = 300;
        double scale = 2.4;


        int exp_half_w = std::min(max_half_w / 2, int(track_window.width * scale));
        int exp_half_h = std::min(max_half_h / 2, int(track_window.height * scale));

        int w = std::min(max_half_w, exp_half_w);
        int h = std::min(max_half_h, exp_half_h);

        cv::Point center;
        center.x = track_window.x + track_window.width / 2;
        center.y = track_window.y + track_window.height / 2;

        int x = std::max(center.x - w, 0);
        int y = std::max(center.y - h, 0);
        track_lu = cv::Point(x, y);

        x = std::min(center.x + w, new_srcImage.cols);
        y = std::min(center.y + h, new_srcImage.rows);
        track_rd = cv::Point(x, y);

        _track_rect = cv::Rect(track_lu, track_rd);
        //cout << track_lu << "\t" << track_rd << endl;
        //rectangle( show_src, _track_rect, Scalar(0, 255, 0), 2, 8, 0 );

        if (makeRectSafe(_track_rect, new_srcImage.size()) == false)
        {
            _track_rect = cv::Rect(0, 0, new_srcImage.cols, new_srcImage.rows);
            image_roix = new_srcImage;
        }

        else
        {
            new_srcImage(_track_rect).copyTo(image_roix);
        }
    }

    if (TargetGet_times >= 1)
        track_object = 1;
    if (TargetLost_times > const_lost_match_frame)//Tracking in next 30 frames when no recognized.
        track_object = 0;


    if (track_object && continue_shot == 0)
    {
        //imshow("image_roix", image_roix);
        //imshow("track_image", track_Image);

        new_srcImage(track_window).copyTo(copy_image);
        cv::cvtColor(copy_image,gray_image,cv::COLOR_RGB2GRAY,0);
        cv::threshold(gray_image,threshold_image, 100, 255,cv::THRESH_BINARY);
        standard_template.create(gray_image.rows,gray_image.cols,gray_image.type());
        makeTemplatebig(standard_template.cols,standard_template.rows,standard_template);

        imshow("standard_template",standard_template);
        imshow("threshold_image",threshold_image);
        double a=brightSums(standard_template);
        double b=brightSums(threshold_image);
        double remind=b/a;
        ///模板匹配条件筛选，防止漂移，参数可调
        if(standard_template.cols<100||((standard_template.cols>100)
                                        &&(remind)>0.2&&(remind<6))
                &&threshold_image.cols>threshold_image.rows
                &&standard_template.cols<400
                &&abs(D_height-standard_template.rows)<20
                )
        {
            double minVal;
            double maxVal;
            cv::Point minLoc;
            cv::Point maxLoc;
            cv::Point2f matchLoc;

            matchTemplate(image_roix, track_Image, result, CV_TM_SQDIFF_NORMED);

            normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

            minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc,cv::Mat());

            matchLoc = minLoc;
            //cout << maxVal<<endl;
            cv::Rect march_target = cv::Rect(matchLoc + track_lu, cv::Size(track_image_march.cols, track_image_march.rows));

            target_pixel.x = march_target.x + march_target.width / 2;
            target_pixel.y = march_target.y + march_target.height / 2;

            ///防漂移，参数可调
            if((abs(last_point.x-target_pixel.x)<march_target.width / 8)
                    &&target_pixel.x>march_target.width
                    &&(new_srcImage.cols-target_pixel.x)>march_target.width
                    &&track_delta_wtidth*1.2>march_target.width
                    &&march_target.width<march_target.height*9
                    &&march_target.width<400)
            {
                cv::rectangle(show, march_target, cv::Scalar(0, 255, 0), 2, 8, 0);

                march_target_flag = 1;
                get_one=1;
                circle(show, target_pixel, 3, cv::Scalar(0, 255, 0), 2);

            }
        }
        else
        {
            lost_frame=const_lost_frame;
        }
    }
    return 0;
}

void armor::ArmorDetector::makeTemplate(int cols,int rows,cv::Mat src)
{
    for(int i=0;i<rows;i++)
    {
        uchar*data=src.ptr<uchar>(i);
        for(int j=0;j<cols;j++)
        {
            if((j<cols/14)||j>cols*13/14)
                data[j]=255;
            else data[j]=0;
        }
    }
}

void armor::ArmorDetector::makeTemplatebig(int cols,int rows,cv::Mat src)
{
    for(int i=0;i<rows;i++)
    {
        uchar*data=src.ptr<uchar>(i);
        for(int j=0;j<cols;j++)
        {
            if((j<cols/14)||j>cols*13/14)
                data[j]=255;
            else data[j]=0;
        }
    }
}

int armor::ArmorDetector::brightSums(cv::Mat src)
{
    int counter = 0;
    //迭代器访问像素点
    cv::Mat_<uchar>::iterator it = src.begin<uchar>();
    cv::Mat_<uchar>::iterator itend = src.end<uchar>();
    for (; it!=itend; ++it)
    {
        if((*it)>0) counter+=1;//二值化后，像素点是0或者255
    }
    return counter;
}


float armor::ArmorDetector::angle(float x1, float y1, float x2, float y2)
{
    float angle_temp;
    float xx, yy;

    xx = x2 - x1;
    yy = y2 - y1;

    if (xx == 0.0)
        angle_temp = 0;
    else
        angle_temp = atan(fabs(yy / xx));
    angle_temp = angle_temp * 180 / PI;
    if (angle_temp>45)
        angle_temp -= 90;
    return (angle_temp);
}


int armor::Shooter::angle_analysis(cv::Point &Target_point,cv::Point2f &convert_angle,double &Distance)
{
    transf_theta_ = Target_point.x - intrinsic_.at<double>(0, 2);
    transf_theta_ = transf_theta_ / intrinsic_.at<double>(0, 0);
    transf_theta_ = atan(transf_theta_);
    transf_theta_ = -180 * transf_theta_ / PI;


    transf_elpha_ = Target_point.y - intrinsic_.at<double>(1, 2);
    transf_elpha_ = transf_elpha_ / intrinsic_.at<double>(1, 1);
    transf_elpha_ = atan(transf_elpha_);
    transf_elpha_ = -180 * transf_elpha_ / PI;

    tmp_direction.x = transf_theta_;
    tmp_direction.y = transf_elpha_;

        cout << ">>>>>>>>>>>>x_angle :" << tmp_direction.x << endl;
        cout << ">>>>>>>>>>>>y_angle :" << tmp_direction.y << endl;

    convert_angle.x=tmp_direction.x;
    convert_angle.y=tmp_direction.y-2;
    //Distance = 0.0931*D_height*D_height-8.5133*D_height+251.65;
    Distance=1000;
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

//运动预测在这里写
//int Shooter::motion_predict(cv::Point &tatget_pixel)
//{
//    cv::Point2f kalmanPoint = kf_.run(tatget_pixel.x, tatget_pixel.y);
//    cv::Point2f currentPoint;
//    cv::Point2f last_Point;

//    float a;
//    //    if(last_Point.x>0 && tatget_pixel.x>0 && 20<abs(kalmanPoint.x - tatget_pixel.x) )
//    //   {
//    //       if (tatget_pixel.x - last_Point.x>0 )
//    //      {
//    if(20<abs(kalmanPoint.x - tatget_pixel.x))
//        kalmanPoint.x = tatget_pixel.x + range_value *(tatget_pixel.x - kalmanPoint.x);
//    //      }
//    //      else
//    //       {
//    //         kalmanPoint.x = tatget_pixel.x - range_value * abs(tatget_pixel.x - kalmanPoint.x);
//    //       }
//    //   }
//    //    else
//    //    {
//    //        kalmanPoint.x=tatget_pixel.x;
//    //    }
//    last_Point = currentPoint;
//    a = kalmanPoint.x - tatget_pixel.x;

//    //std::cout << "target_pixel11111111111" << tatget_pixel.x << "kalmanPoint" << kalmanPoint.x << "a" << a << std::endl;
//    tatget_pixel.x = kalmanPoint.x;
//    return 0;

//}

}
