#include <opencv2/opencv.hpp>
#include <math.h>

using namespace std;
using namespace cv;

Mat templa;
Rect templRect;                     //模板矩形框
bool drawing_rect_1;              //是否绘制矩形框
bool drawing_rect;
int countt;
double match_best;

void matching(Mat frame1, Mat &templ, Rect &rect)     //模板匹配子函数
{
    Mat frame_hsv;
    cvtColor(frame1, frame_hsv, CV_BGR2GRAY);
    //选取搜索窗口，在目标周围进行搜索
    Rect searchWindow;
    searchWindow.width = rect.width * 1.5;
    searchWindow.height = rect.height * 1.5;
    searchWindow.x = rect.x + rect.width * 0.5 - searchWindow.width * 0.5;
    searchWindow.y = rect.y + rect.height * 0.5 - searchWindow.height * 0.5;
    searchWindow &= Rect(0, 0, frame_hsv.cols, frame_hsv.rows);
    //cout << "模板的宽和高:" << templ.size() << "矩形框宽:" << rect.width << "矩形框高:"<< rect.height << endl;
    if (rect.width != 0)
    {
        countt++;
        //模板匹配
        Mat similarity;
        matchTemplate(frame_hsv(searchWindow), templ, similarity, CV_TM_CCOEFF_NORMED);
        //找出最佳匹配的位置
        double maxVal;
        Point maxLoc;
        minMaxLoc(similarity, 0, &maxVal, 0, &maxLoc);         //找到最大匹配值和其对应的位置
        cout << "最大匹配值"<<maxVal << endl;
        if (maxVal > 0.9)      //只有匹配值大于0.5才进行检测
        {
            //cout << "更新模板" << endl;
            //更新模板矩形框
            rect.x = maxLoc.x + searchWindow.x;
            rect.y = maxLoc.y + searchWindow.y;
            //更新模板
            templ = frame_hsv(rect);
            templ.copyTo(templa);
            templRect = rect;
            //cout<<templ.size()<<"\t"<<countt<<endl;
            drawing_rect = true;
        }
    }
}

void init_matching(Mat frame1, Mat &templ, Rect &rect)     //模板匹配子函数
{
    Mat frame_gray;
    //cvtColor(frame1, frame_hsv, CV_BGR2HSV);
    cvtColor(frame1, frame_gray, CV_BGR2GRAY);

    if (rect.width != 0)
    {
        countt++;
        //模板匹配
        Mat similarity;
        matchTemplate(frame_gray, templ, similarity, CV_TM_CCOEFF_NORMED);
        //找出最佳匹配的位置
        double maxVal;
        Point maxLoc;
        minMaxLoc(similarity, 0, &maxVal, 0, &maxLoc);         //找到最大匹配值和其对应的位置
        cout << "最大匹配值"<<maxVal << endl;

        //pick the best matched template for first frame
        //match_best = 0;
        /*
        if (match_best <= maxVal)
        {
            match_best = maxVal;
        }
*/
        if (maxVal > 0.7)   //只有匹配值大于0.5才进行检测
        {
            cout << "更新模板" << endl;
            //更新模板矩形框
            rect.x = maxLoc.x;
            rect.y = maxLoc.y;
            cout << "rect.x:" << rect.x << "\t" << "rect.y:" <<rect.y << endl;
            //更新模板
            templ = frame_gray(rect);
            templ.copyTo(templa);
            templRect = rect;
            cout<<templ.size()<<"\t"<<countt<<endl;
            drawing_rect_1 = true;
        }
    }
}


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
        resize(srcImage, srcImage, Size(floor(srcImage.cols/2), floor(srcImage.rows/2)));

        if (countFrame == 1)   //find the first template
        {  
            //get the length array for template
            short scale_arr[500];
            int i;
            for (i=0; i<500; i++)
            {
                scale_arr[i] = srcImage.rows - 5 *i;
                cout<<scale_arr[i]<<endl;
            }

            Mat templa_;
            templa = imread("/home/robomaster/LHH/box_detect/template_42.jpg");
            imshow("template", templa);
            waitKey(20);
            resize(templa, templa, Size(srcImage.cols, srcImage.rows));
            cvtColor(templa, templa, CV_BGR2GRAY);

            int j;
            //match_best = 0;
            for (j=0; j<500; j++)
            {
                resize(templa, templa_, Size(scale_arr[j], scale_arr[j]));
                Rect templRect(0, 0, templa_.cols, templa_.rows);
                cout << templa_.cols <<"\t" <<templa_.rows<<endl;
                imshow("templa_", templa_);
                waitKey(20);

                init_matching(srcImage, templa_, templRect);
                if (drawing_rect_1)
                {
                    templa = templa_;
                    cout << "templRect" << templRect << endl;
                    rectangle(srcImage, templRect, Scalar(0, 0, 255), 2);
                    imshow("matched", srcImage);
                    //waitKey(0);
                    break;
                }
            }
        }

        outImage = srcImage.clone();//复制图片
        //加载源图像和模板图像
        matching(outImage, templa, templRect);

        if (drawing_rect)
        {
            rectangle(srcImage, templRect, Scalar(0, 255, 0), 2);
        }

        imshow("result", srcImage);
        //waitKey(30);

        if (waitKey(20) == 27)
            break;

    }
    return 0;
}
