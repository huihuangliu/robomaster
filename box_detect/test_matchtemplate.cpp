#include<opencv2/highgui/highgui.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<iostream>
#include<cctype>
#include<cv.hpp>

using namespace cv;
using namespace std;
RNG g_rng(12345);


//Rect templRect = Rect::Rect_(0, 0, 20, 20);                     //模板矩形框
Rect templRect;															//Mat templa = Mat::zeros(20, 20, CV_32F);                        //模板
Mat templa;                        //模板
bool drawing_rect = false;      //是否绘制矩形框
bool add_matchtemplate = true;    //加上模板匹配方法辅助检测
bool stop_contours = true;         //不用轮廓查找的方法


                                   /// 模板匹配追踪///
void matching(Mat frame, Mat &templ, Rect &rect);

void rgb_matching(Mat frame, Mat templ, Rect &rect);

int main()
{
    int countFrame = 0;
    //读入视频
    VideoCapture capture("/home/robomaster/LHH/box_detect/box_42.mp4");
    if (!capture.isOpened())
    {
        return 0;
    }

    Mat srcImage, outImage, maskImage, grayImage;

    while (capture.read(srcImage))
    {
        countFrame++;

        if (countFrame == 1)
        {
            Mat templa = imread("/home/robomaster/LHH/box_detect/template_42.jpg");                        //初始化模板
            /*
            namedWindow("init template", WINDOW_AUTOSIZE);
            imshow("init template", templa);
            waitKey(0);
            */
        }
        double start = (double)getTickCount();
        outImage = srcImage.clone();//复制图片

        cvtColor(outImage, grayImage, CV_RGB2GRAY);//灰度化
        blur(grayImage, maskImage, Size(3, 3)); //均值滤波
        threshold(maskImage, maskImage, 100, 255, CV_THRESH_BINARY);//二值化

                                                                    //形态学闭运算处理
        Mat element1 = getStructuringElement(MORPH_RECT, Size(2, 2));
        Mat element2 = getStructuringElement(MORPH_RECT, Size(2, 2));
        erode(maskImage, maskImage, element2, cv::Point(-1, -1), 1);
        dilate(maskImage, maskImage, element1, cv::Point(-1, -1), 1);

        //遍历寻找外轮廓
        vector<vector<Point>>contours, contours2;
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
            //限制矩形长度，避免框到非对应的灯条
            if (abs(point2.y - point1.y) * 8 > abs(point2.x - point1.x))
                if (abs(point2.x - point1.x) < 122 && abs(point2.y - point1.y)*1.5 < abs(point2.x - point1.x))
                    rectangle(srcImage, point1, point2, Scalar(0, 0, 255), 2, 8, 0);

            //保存模板，并更新
            templRect.x = point1.x;
            templRect.y = point1.y;
            templRect.width = abs(point2.x - point1.x);
            templRect.height = abs(point2.y - point1.y);
            //templa = grayImage(templRect);      //模板图像,使用灰度模板匹配时
            templa = outImage(templRect);         //使用彩色图像模板匹配时

            //cout << "图像尺寸：" << outImage.channels() << endl;
            //cout <<"模板尺寸："<< templa.channels() << endl;

            //stop_contours = false;     //提取出图像初始的ROI区域之后就不再进行轮廓查找
            //cout << "停止轮廓匹配" << endl;

        }
        else if (add_matchtemplate)     //当没有检测到目标时，继续用模板匹配检测
        {
            cout << "出现掉帧" << endl;
            //参数：帧图像，模板图像，模板矩形框
            rgb_matching(outImage, templa, templRect);
            if (drawing_rect)
            {
                rectangle(srcImage, templRect, Scalar(0, 255, 0), 3);
                cout << "模板匹配画出矩形框" << endl;

            }
        }
        imshow("原图", srcImage);
        imshow("掩膜图", maskImage);
        double end = (double)getTickCount();
        cout << "Frame: " << countFrame << "\tTime: ";
        cout << (end - start) * 1000 / (getTickFrequency()) << "ms" << endl;

        waitKey(1);
    }
    //手动释放视频捕获资源
    capture.release();
    //system("pause");
    waitKey(0);
    return 0;
}


void matching(Mat frame, Mat &templ, Rect &rect)     //利用灰度图像模板匹配
{
    Mat grayFrame;
    cvtColor(frame, grayFrame, CV_RGB2GRAY);

    //选取搜索窗口，在目标周围进行搜索
    Rect searchWindow;
    searchWindow.width = rect.width * 3;
    searchWindow.height = rect.height * 2;
    searchWindow.x = rect.x + rect.width * 0.5 - searchWindow.width * 0.5;
    searchWindow.y = rect.y + rect.height * 0.5 - searchWindow.height * 0.5;
    searchWindow &= Rect(0, 0, frame.cols, frame.rows);

    //imshow("searchWindow", frame(searchWindow));

    cout << "模板的宽和高:" << templ.size() << "矩形框宽:" << rect.width << "矩形框高:" << rect.height << endl;

    if (rect.width != 0)
    {
        //模板匹配
        Mat similarity;
        //matchTemplate(grayFrame(searchWindow), templ, similarity, CV_TM_CCOEFF_NORMED);
        matchTemplate(grayFrame(searchWindow), templ, similarity, CV_TM_SQDIFF);       //匹配越好值越小
        //matchTemplate(grayFrame(searchWindow), templ, similarity, CV_TM_CCORR);       //数值越大匹配越好
        //matchTemplate(grayFrame(searchWindow), templ, similarity, CV_TM_CCOEFF);      //1表示完美匹配，-1表示最差
        //matchTemplate(grayFrame(searchWindow), templ, similarity, CV_TM_SQDIFF_NORMED);
        //matchTemplate(grayFrame(searchWindow), templ, similarity, CV_TM_CCORR_NORMED);
        //normalize(similarity, similarity, 0, 1, NORM_MINMAX, -1, Mat());      //归一化结果
        //找出最佳匹配的位置
        /*
        double maxVal;
        Point maxLoc;
        minMaxLoc(similarity, 0, &maxVal, 0, &maxLoc);         //找到最大匹配值和其对应的位置
        */
        //cout << "最大匹配值"<<maxVal << endl;

        double minVal;
        Point minLoc;
        minMaxLoc(similarity, &minVal, 0, &minLoc, 0);

        if (minVal < 0.5)      //只有匹配值大于0.5才进行检测
        {
            cout << "更新模板" << endl;
            //更新模板矩形框
            rect.x = minLoc.x + searchWindow.x;
            rect.y = minLoc.y + searchWindow.y;
            //更新模板
            templ = grayFrame(rect);
            templ.copyTo(templa);
            drawing_rect = true;
        }
    }


}


void rgb_matching(Mat frame, Mat templ, Rect &rect)     //利用彩色图像模板匹配
{
    Mat srcimage_B, srcimage_G, srcimage_R;
    Mat temimage_B, temimage_G, temimage_R;
    vector <Mat> channels_src, channels_tem;

    split(frame, channels_src);       //搜索图像通道分离
    srcimage_B = channels_src.at(0);
    srcimage_G = channels_src.at(1);
    srcimage_R = channels_src.at(2);

    cout << "template channels" << templ.channels() << endl;

    split(templ, channels_tem);       //模板图像通道分离
    temimage_B = channels_tem.at(0);
    temimage_G = channels_tem.at(1);
    temimage_R = channels_tem.at(2);

    //选取搜索窗口，在目标周围进行搜索
    Rect searchWindow;
    searchWindow.width = rect.width * 3;
    searchWindow.height = rect.height * 2;
    searchWindow.x = rect.x + rect.width * 0.5 - searchWindow.width * 0.5;
    searchWindow.y = rect.y + rect.height * 0.5 - searchWindow.height * 0.5;
    searchWindow &= Rect(0, 0, frame.cols, frame.rows);

    //imshow("searchWindow", frame(searchWindow));

    cout << "模板的宽和高:" << templ.size() << "矩形框宽:" << rect.width << "矩形框高:" << rect.height << endl;

    if (rect.width != 0)
    {
        //模板匹配
        Mat similarity_B, similarity_G, similarity_R, tmp, similarity;
        matchTemplate(srcimage_B(searchWindow), temimage_B, similarity_B, CV_TM_CCOEFF_NORMED);
        matchTemplate(srcimage_G(searchWindow), temimage_G, similarity_G, CV_TM_CCOEFF_NORMED);
        matchTemplate(srcimage_R(searchWindow), temimage_R, similarity_R, CV_TM_CCOEFF_NORMED);

        addWeighted(similarity_B, 1 / 3, similarity_G, 1 / 3, 0.0, tmp);
        addWeighted(tmp, 1, temimage_R, 1 / 3, 0.0, similarity);

        //matchTemplate(srcimage_B(searchWindow), temimage_B, similarity, CV_TM_SQDIFF);       //匹配越好值越小
        //matchTemplate(grayFrame(searchWindow), templ, similarity, CV_TM_CCORR);       //数值越大匹配越好
        //matchTemplate(grayFrame(searchWindow), templ, similarity, CV_TM_CCOEFF);      //1表示完美匹配，-1表示最差
        //matchTemplate(grayFrame(searchWindow), templ, similarity, CV_TM_SQDIFF_NORMED);
        //matchTemplate(grayFrame(searchWindow), templ, similarity, CV_TM_CCORR_NORMED);
        //normalize(similarity, similarity, 0, 1, NORM_MINMAX, -1, Mat());      //归一化结果
        //找出最佳匹配的位置

        double maxVal;
        Point maxLoc;
        minMaxLoc(similarity, 0, &maxVal, 0, &maxLoc);         //找到最大匹配值和其对应的位置

        //cout << "最大匹配值"<<maxVal << endl;

        if (maxVal > 0.5)      //只有匹配值大于0.5才进行检测
        {
            cout << "更新模板" << endl;
            //更新模板矩形框
            rect.x = maxLoc.x + searchWindow.x;
            rect.y = maxLoc.y + searchWindow.y;
            //更新模板
            templ = frame(rect);
            templ.copyTo(templa);
            drawing_rect = true;
        }
    }


}
