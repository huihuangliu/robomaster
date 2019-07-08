#include "armor.h"

using namespace std;
using namespace cv;
enum EnemyColor { RED = 0, BLUE = 1 };         // 0 for red, otherwise blue
static int  max_light_delta_h = 450;
float continue_shot[2] = { 0 };
int const_lost_frame = 30;
int lost_frame = 0;
Point2f lu;
Point2f rd;
Point2f vertex_result[4];
#define PI 3.1415926
extern unsigned char enemy_color;
extern Mat src;
Mat maskImage;
Mat show_src;
Mat new_srcImage;
double transf_theta_;
double transf_aefa_;
double x_integrate;
double y_integrate;
float target_height;
#define M_PI 3.14159265358979323846
extern short senddata[2];
Point last_point;

float x_correction(float x_angle)
{
    float newx_angle = 0;
    float distance = 0;
    float pos_neg = abs(x_angle) / x_angle;

    distance = 3.600*0.001*0.06*77.500 / (target_height*0.300*0.001);

    if (abs(x_angle) < 5)
    {
        newx_angle = x_angle;
    }

    else
    {


        if (distance > 2.4)
        {
            newx_angle = 0.9912*x_angle - 0.0007*pos_neg;
        }

        if ((distance > 1) && (distance <= 2.4))
        {
            newx_angle = 0.9825*x_angle - 0.0013*pos_neg;
        }

        if (distance <= 1)
        {
            newx_angle = 0.9584*x_angle - 0.0029*pos_neg;
        }

    }

    if ((distance > 1) && (distance <= 1.5))
    {
        newx_angle = x_angle;
    }

    if ((distance > 1.5) && (distance <= 2))
    {
        newx_angle = x_angle;
    }

    if ((distance > 2) && (distance <= 3))
    {
        newx_angle = x_angle;
    }

    if ((distance > 3) && (distance <= 3.5))
    {
        newx_angle = x_angle;
    }

    if ((distance > 3.5) && (distance <= 3.75))
    {
        newx_angle = x_angle;
    }

    if (distance > 3.75)
    {
        newx_angle = x_angle;
    }

    return newx_angle;
}

float y_correction()
{
    float correct_angle = 0;
    float distance = 0;

    distance = 3.600*0.001*0.06*77.500 / (target_height*0.300*0.001);


    if (distance > 4)
    {
        correct_angle = 2.4;//1.35;
    }

    if ((distance > 3.5) && (distance <= 4))
    {
        correct_angle = 2.4;//1.35;
    }

    if ((distance > 3.25) && (distance <= 3.5))
    {
        correct_angle = 2.4;//1.35;
    }

    if ((distance > 3) && (distance <= 3.25))
    {
        correct_angle = 2.4;//1.4;
    }

    if ((distance > 2.75) && (distance <= 3))
    {
        correct_angle = 2.4;//1.7;
    }

    if ((distance > 2.5) && (distance <= 2.75))
    {
        correct_angle = 2.4;//1.9;
    }

    if ((distance > 2) && (distance <= 2.5))
    {
        correct_angle = 2.395;//1.9;
    }

    if ((distance > 1.5) && (distance <= 2))
    {
        correct_angle = 2.4;//1.8;
    }

    if (distance <= 1.5)
    {
        correct_angle = 2.3;//1.6;
    }
    return correct_angle;
}

void ArmorDetector::Aimtarget()
{
    tmp(convert_dirct);

    cout << ">>>>>>>>>>>>x_now :" << tmp_direction.x << endl;
    cout << ">>>>>>>>>>>>y_now :" << tmp_direction.x << endl;
    if (abs(tmp_direction.x) > 1)
        x_integrate += tmp_direction.x;
    x_integrate = x_integrate < 40 ? x_integrate : 40;
    x_integrate = x_integrate > -40 ? x_integrate : -40;
    //cout <<">>>>>>>>>>>>x_integrate: " << x_integrate <<endl;

    //x_now = tmp_direction.x;
    //cout <<">>>>>>>>>>>>x_now :"<<x_now <<endl;
    //cout <<">>>>>>>>>>>>x_before :"<< x_before <<endl;
    if (abs(tmp_direction.x) > 4)
    {
        tmp_direction.x = (tmp_direction.x / 4 + x_integrate / 40.0);// / 1.6 * 1.50;// + (x_now - x_before) / 4;///10.000 ;//
    }
    else {
        tmp_direction.x = (tmp_direction.x / 4 + x_integrate / 20.0) / 2.5;
    }
    //cout <<">>>>>>>>>>>>x_final :"<<tmp_direction.x <<endl;
    //x_before = x_now;

    //if(abs(x_before) < 7)
    //{


    float distance = 0;
    distance = 3.600*0.001*0.06*77.500 / (target_height*0.300*0.001);

    tmp_direction.y += y_correction();

    //cout <<">>>>>>>>>>>>y_error :"<< (y_correction() * 4) <<endl;

    if (abs(tmp_direction.y) > 1)
        y_integrate += tmp_direction.y;
    y_integrate = y_integrate < 20 ? y_integrate : 20;
    y_integrate = y_integrate > -20 ? y_integrate : -20;
    if (abs(tmp_direction.y) > 2)
    {
        tmp_direction.y = tmp_direction.y / 1.5 + y_integrate / 20.0;
    }
    else {
        tmp_direction.y = 3 * (tmp_direction.y / 1.5 + y_integrate / 60.0);
    }

    cout << ">>>>>>>>>>>>distance : " << distance << endl;
    cout << ">>>>>>>>>>>>x_out : " << tmp_direction.x << endl;
    cout << ">>>>>>>>>>>>y_out : " << tmp_direction.y << endl;


    senddata[0] = tmp_direction.x * 100;
    senddata[1] = tmp_direction.y * 100;
    //tmp_direction.y += y_correction();

    //cout <<">>>>>>>>>>>>y_out :"<< tmp_direction.y <<endl;
    cout << ">>>>>>>>>>>>x_integrate : " << x_integrate << endl;
    cout << ">>>>>>>>>>>>y_integrate : " << y_integrate << endl;
    /*if(abs(tmp_direction.y) > 1)
    {
    tmp_direction.y = tmp_direction.y/1.500 + y_correction();
    }
    if(abs(tmp_direction.y) <= 1)
    {
    if(distance<2.5)
    {
    tmp_direction.y = tmp_direction.y/0.500 + y_correction();
    }
    if(distance>=2.5)
    {
    tmp_direction.y = tmp_direction.y/0.500 + y_correction();
    }
    }*/


}

void ArmorDetector::tmp(const cv::Point2f &Target_point)
{


    transf_theta_ = Target_point.x - intrinsic_.at<double>(0, 2);
    transf_theta_ = transf_theta_ / intrinsic_.at<double>(0, 0);
    transf_theta_ = atan(transf_theta_);
    transf_theta_ = -180 * transf_theta_ / M_PI;


    transf_aefa_ = Target_point.y - intrinsic_.at<double>(1, 2);
    transf_aefa_ = transf_aefa_ / intrinsic_.at<double>(1, 1);
    transf_aefa_ = atan(transf_aefa_);
    transf_aefa_ = -180 * transf_aefa_ / M_PI;

    tmp_direction.x = transf_theta_;
    tmp_direction.y = transf_aefa_;

}

float dist(float ax, float ay, float bx, float by)
{
    float d = sqrt((ax - bx)*(ax - bx) + (ay - by)*(ay - by));
    return d;
}

float angle(float x1, float y1, float x2, float y2)
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

int brightSums(Mat src)    //count all pixel whose intensity >0
{

    int counter = 0;
    //迭代器访问像素点
    Mat_<uchar>::iterator it = src.begin<uchar>();
    Mat_<uchar>::iterator itend = src.end<uchar>();
    for (; it!=itend; ++it)
    {
        if((*it)>0) counter+=1;//二值化后，像素点是0或者255
    }
    return counter;
}

void ArmorDetector::setImage(const cv::Mat & src)    //pretreat image

{
    Mat imagehsv;
    src.copyTo(show_src);
    src.copyTo(new_srcImage);

    _size = src.size();
    const cv::Point last_result = _res_last.center;

    if (last_result.x == 0 || last_result.y == 0)
    {
        if ((_dect_rect.width&&_dect_rect.height) == 0)
        {
            _src = src;
            _dect_rect = Rect(0, 0, src.cols, src.rows);
            src(_dect_rect).copyTo(_src);
        }
        else
        {
            src(_dect_rect).copyTo(_src);
            lost_frame++;
        }
        if (lost_frame >= const_lost_frame)   //full screen search
        {
            _src = src;
            _dect_rect = Rect(0, 0, src.cols, src.rows);
            src(_dect_rect).copyTo(_src);
        }
    }
    else
    {
        lost_frame = 0;
        Rect rect = _res_last.boundingRect();
        int max_half_w = 450 * 1.3;
        int max_half_h = 300;
        double scale = src.rows == 480 ? 1.8 : 2.5;

        int exp_half_w = min(max_half_w / 2, int(rect.width * scale));
        int exp_half_h = min(max_half_h / 2, int(rect.height * scale));

        int w = std::min(max_half_w, exp_half_w);
        int h = std::min(max_half_h, exp_half_h);
        Point center = last_result;
        int x = std::max(center.x - w, 0);
        int y = std::max(center.y - h, 0);
        lu = Point(x, y);
        x = std::min(center.x + w, src.cols);
        y = std::min(center.y + h, src.rows);
        rd = Point(x, y);
        _dect_rect = Rect(lu, rd);
        if (makeRectSafe(_dect_rect, src.size()) == false)
        {
            _res_last = cv::RotatedRect();     //RotatedRect class
            _dect_rect = Rect(0, 0, src.cols, src.rows);
            _src = src;
        }
        else
            src(_dect_rect).copyTo(_src);
    }

    //imshow("_src", _src);


    //    Mat threMat_(cv::Size(_src.cols, _src.rows), CV_8UC1);
    cvtColor(_src, imagehsv, CV_BGR2HSV);
    if (enemy_color == RED)
        inRange(imagehsv, Scalar(150, 15, 65), Scalar(255, 255, 255), imagehsv);
    else
        inRange(imagehsv, Scalar(78, 15, 80), Scalar(145, 255, 255),imagehsv);


    std::cout << "lost_frame: " << lost_frame << std::endl;
    if (enemy_color == RED)
    {
        vector<Mat> mv;
        Mat dst1;
        split(_src, mv);
        threshold(mv[0], dst1, 110, 255, THRESH_BINARY);  //for blue
        bitwise_and(imagehsv, dst1, maskImage);
    }
    else
    {
        vector<Mat> mv;
        Mat dst1;
        split(_src, mv);
        threshold(mv[2], dst1, 110, 255, THRESH_BINARY);   //for red
        bitwise_and(imagehsv, dst1, maskImage);
    }

    Mat element1 = getStructuringElement(MORPH_RECT, Size(2, 1));
    Mat element2 = getStructuringElement(MORPH_RECT, Size(2, 1));
    erode(maskImage, maskImage, element2, cv::Point(-1, -1), 1);
    dilate(maskImage, maskImage, element1, cv::Point(-1, -1), 1);
}

void ArmorDetector::findTarget(const cv::Mat & src, const cv::Mat &mask)

{
    int getTarget = 0;
    vector<vector<Point> >contours, contours2, contours3, contours4;

    findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0));


    for (int i = 0; i < contours.size(); i++)
    {
        float d[4];
        float height, width;
        RotatedRect rect = minAreaRect(contours[i]);
        Rect rect1 = boundingRect(contours[i]);
        Point2f vertex[4];
        rect.points(vertex);
        //Point center=rect.center;

        if (rect1.height>rect1.width)
        {
            for (int i = 0; i<4; i++)
            {
                d[i] = dist(vertex[i].x, vertex[i].y, vertex[(i + 1) % 4].x, vertex[(i + 1) % 4].y);
            }

            float angle1 = angle(vertex[0].x, vertex[0].y, vertex[1].x, vertex[1].y);

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

            if (height>width*1.5
                &&angle1<20
                && contours[i].size() > 10
                )
            {

                contours2.push_back(contours[i]);
            }
        }
    }


    vector<double> area;
    area.resize(contours2.size());

    for (int i = 0; i<contours2.size(); i++)
        area[i] = contourArea(contours2[i]);


    float box_up_angle = 0, box_down_angle = 0, box_left_angle = 0, box_right_angle = 0;
    float delta_up_down_angle = 0;
    float delta_left_right_angle = 0;


    for (int i = 0; i < contours2.size(); i++)
    {
        RotatedRect rect_i = minAreaRect(contours2[i]);
        Point2f vertice_i[4];
        rect_i.points(vertice_i);
        Point2f up_point_i = vertice_i[0], down_point_i = vertice_i[0];

        for (int j = i + 1; j < contours2.size(); j++)
        {
            RotatedRect rect_j = minAreaRect(contours2[j]);
            Point2f vertice_j[4];
            rect_j.points(vertice_j);
            Point2f up_point_j = vertice_j[0], down_point_j = vertice_j[0];


            for (int m = 0; m < 4; m++)
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


            double height_i = max(rect_i.size.height, rect_i.size.width);
            double height_j = max(rect_j.size.height, rect_j.size.width);


            double distance_center = dist(rect_i.center.x, rect_i.center.y, rect_j.center.x, rect_j.center.y);
            double averageHeight = (height_i + height_j) / 2;
            double deltaY = distance_center / averageHeight;


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


            if (box_up_angle <20
                && box_down_angle <20
                && height_i / height_j<1.25
                &&height_i / height_j>0.75
                &&abs(up_point_i.x-up_point_j.x)>averageHeight
                && deltaY <5
                && delta_up_down_angle <7
                && (delta_left_right_angle <5))
            {

                if (distance_center>averageHeight*2.1
                    &&averageHeight*4.5>distance_center)

                {
                    contours3.push_back(contours2[i]);
                    contours3.push_back(contours2[j]);
                    getTarget = 1;
                }
            }
        }
    }



    if (getTarget == 1)
    {
        continue_shot[0] = getTarget;
//        if((abs(vertex_result[0].x-vertex_result[1].x)>200)||
//                (abs(vertex_result[0].x-vertex_result[2].x)>200))
//        {
//            getTarget = 0;
//        }

        vector<Point> contours_result;
        int contours_i_size = contours3[0].size();
        int contours_j_size = contours3[1].size();
        contours_result.resize(contours_i_size + contours_j_size);

        for (int m = 0; m < contours_i_size; m++)
            contours_result[m] = contours3[0][m];
        for (int n = 0; n < contours_j_size; n++)
            contours_result[n + contours_i_size] = contours3[1][n];

        RotatedRect rect_result = minAreaRect(contours_result);
        Rect bounding_rect =boundingRect(contours_result);

        rect_result.points(vertex_result);
        if (lost_frame >= const_lost_frame)
        {
            _res_last = RotatedRect();
            vector <Point2f>_res_last_point;
            for (int i = 0; i<4; i++)
                _res_last_point.push_back(vertex_result[i]);

            _res_last = minAreaRect(_res_last_point);

            for (int i = 0; i<4; i++)
            {

                line(show_src, vertex_result[i], vertex_result[(i + 1) % 4], Scalar(0, 255, 0), 2, LINE_AA);
            }
        }
        else
        {
            vector <Point2f>_res_last_point;
            for (int i = 0; i<4; i++)
                _res_last_point.push_back(vertex_result[i] + lu);
            _res_last = minAreaRect(_res_last_point);

            for (int line_j = 0; line_j <4; line_j++)
                line(show_src, vertex_result[line_j] + lu, vertex_result[(line_j + 1) % 4] + lu, Scalar(255, 0, 0), 2, LINE_AA);
            Rect rect_test = boundingRect(_res_last_point);
            //cout<<rect_test.width<<endl;
            if (rect_test.width==0
                ||(rect_test.width>200))
                lost_frame++;
        }

        if(bounding_rect.width<500)
        {
        convert_dirct.x = _res_last.center.x;
        convert_dirct.y = _res_last.center.y;
        last_point= convert_dirct;
        circle(show_src, convert_dirct, 3, cv::Scalar(0, 255, 0), 2);
        }
    }
    else
    {
        getTarget = 0;
        continue_shot[0] = 0;
        _res_last = cv::RotatedRect();
    }
    //    if(getTarget== 1)
    //            continue_shot[0] = 1;
    //    else
    //    {
    //        continue_shot[0] = 0;
    //        _res_last = cv::RotatedRect();
    //    }
}

void ArmorDetector::target_track()
{
    static unsigned char Is_recognizing = 0;
    static unsigned char track_image_init = 0;
    static int TargetGet_times = 0;
    static int TargetLost_times = 0;
    static Mat track_image;
    Mat track_image_march;
    Rect _track_rect;
    static unsigned char track_object = 0;
    Point2f track_lu, track_rd;
    //Whether Recognize or not
    Is_recognizing = continue_shot[0];
    Mat image_roix;
    Mat result;
    vector<Point> track_points;

    int result_cols = 0;
    int result_rows = 0;
    static Rect track_window;

    unsigned char march_target_flag = 0;

    if (Is_recognizing == 1)
    {
        TargetGet_times++;//Count continuous frames times.
        TargetLost_times = 0;//Fresh lost frames times.

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

        track_window = boundingRect(track_points);
        /*for (int i = 0; i<4; i++)*/
        //track_points.clear;
        //rectangle(show_src, track_window, Scalar(0,255, 0), 2, 8, 0);

        if (makeRectSafe(track_window, new_srcImage.size()) == false)
        {
            track_window = Rect(0, 0, new_srcImage.cols, new_srcImage.rows);
        }

        new_srcImage(track_window).copyTo(track_image);

        result_cols = new_srcImage.cols - track_image.cols + 1;
        result_rows = new_srcImage.rows - track_image.rows + 1;

        result = Mat(result_cols, result_rows, CV_32FC1);
        track_image_init = 1;
    }

    else
    {
        track_image_march = track_image;
        TargetGet_times = 0;
        TargetLost_times++;

        int max_half_w = 450 * 1.3;
        int max_half_h = 300;
        double scale = (new_srcImage.rows == 480 ? 1.8 : 2.5);


        int exp_half_w = min(max_half_w / 2, int(track_window.width * scale));
        int exp_half_h = min(max_half_h / 2, int(track_window.height * scale));

        int w = std::min(max_half_w, exp_half_w);
        int h = std::min(max_half_h, exp_half_h);

        Point center;
        center.x = track_window.x + track_window.width / 2;
        center.y = track_window.y + track_window.height / 2;

        int x = std::max(center.x - w, 0);
        int y = std::max(center.y - h, 0);
        track_lu = Point(x, y);

        x = std::min(center.x + w, new_srcImage.cols);
        y = std::min(center.y + h, new_srcImage.rows);
        track_rd = Point(x, y);

        _track_rect = Rect(track_lu, track_rd);
        //cout << track_lu << "\t" << track_rd << endl;
        //rectangle( show_src, _track_rect, Scalar(0, 255, 0), 2, 8, 0 );

        if (makeRectSafe(_track_rect, new_srcImage.size()) == false)
        {
            _track_rect = Rect(0, 0, new_srcImage.cols, new_srcImage.rows);
            image_roix = new_srcImage;
        }

        else
        {
            new_srcImage(_track_rect).copyTo(image_roix);
        }
    }

    if (TargetGet_times >= 1)
        track_object = 1;
    if (TargetLost_times > const_lost_frame)//Tracking in next 300 frames when no recognized.
        track_object = 0;

    //*****Tracking******
    if (lost_frame < 30)
    {
        if (track_object && track_image_init == 1 && continue_shot[0] == 0)
        {
            //imshow("image_roix", image_roix);
            //imshow("track_image", track_image);


            Mat standard_template;
            Mat gray_image;
            Mat copy_image;
            Mat threshold_image;

            new_srcImage(track_window).copyTo(copy_image);
            cv::cvtColor(copy_image,gray_image,COLOR_RGB2GRAY,0);
            threshold(gray_image,threshold_image, 100, 255,THRESH_BINARY);
            standard_template.create(gray_image.rows,gray_image.cols,gray_image.type());
            for(int i=0;i<standard_template.rows;i++)
            {
                uchar*data=standard_template.ptr<uchar>(i);
                for(int j=0;j<standard_template.cols;j++)
                {
                    if((j<standard_template.cols/14)||j>standard_template.cols*13/14)
                        data[j]=255;
                    else data[j]=0;
                }
            }
            //imshow("standard_template",standard_template);
            //imshow("threshold_image",threshold_image);
            double a=brightSums(standard_template);
            double b=brightSums(threshold_image);
            double remind=b/a;
            cout<<"remind"<<remind<<endl;
//            Mat result1;
//            int result1_cols = 1;
//            int result1_rows = 1;
//            result1.create(result1_cols, result1_rows,standard_template.type());
//            matchTemplate(standard_template, threshold_image, result1, CV_TM_SQDIFF_NORMED);
//            normalize(result1, result1, 0, 1, NORM_MINMAX, -1, Mat());
//            double minVal1 = -1;
//            double maxVal1;
//            Point minLoc1;
//            Point maxLoc1;
//            minMaxLoc(result1, &minVal1, &maxVal1, &minLoc1, &maxLoc1, Mat());
//            cout << "匹配度：" << maxVal1 << endl;
            if(standard_template.cols<100||((standard_template.cols>100)&&(remind)>0.32&&(remind<2)))
            {
                double minVal;
                double maxVal;
                Point minLoc;
                Point maxLoc;
                Point2f matchLoc;

                matchTemplate(image_roix, track_image, result, CV_TM_SQDIFF_NORMED);

                normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());

                minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

                matchLoc = minLoc;
                //cout << maxVal<<endl;
                Rect march_target = Rect(matchLoc + track_lu, Size(track_image_march.cols, track_image_march.rows));

                convert_dirct.x = march_target.x + march_target.width / 2;
                convert_dirct.y = march_target.y + march_target.height / 2;
                cout<<last_point.x<<endl;
                cout << "template_x"<<convert_dirct.x<<endl;
                if((abs(last_point.x-convert_dirct.x)<march_target.width / 2)
                        &&abs(last_point.y-convert_dirct.y)<march_target.height / 2
                        &&convert_dirct.x>march_target.width
                        &&(new_srcImage.cols-convert_dirct.x)>march_target.width
                        &&march_target.width<500)
                {
                    rectangle(show_src, march_target, Scalar(0, 255, 0), 2, 8, 0);

                    march_target_flag = 1;

                    circle(show_src, convert_dirct, 3, cv::Scalar(0, 255, 0), 2);
                }
            }
        }
    }
}


void ArmorDetector::findArmor(const cv::Mat &src)
{

    setImage(src);

    findTarget(_src, maskImage);

    target_track();

    Aimtarget();

    imshow("show_src", show_src);
    imshow("maskImage", maskImage);

}

