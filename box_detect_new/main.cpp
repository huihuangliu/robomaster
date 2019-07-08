#include <opencv2/opencv.hpp>
#include <math.h>


using namespace std;
using namespace cv;

#define zoomRatio 0.5
#define cvtColor_method CV_RGB2GRAY
#define zoomSpeed 15
#define zoomMax 400

int match_method=TM_CCOEFF_NORMED;//TM_CCORR_NORMED,TM_SQDIFF_NORMED,TM_CCOEFF_NORMED
bool drawRect;
int countt = 0;
double match_Value[500];
vector<Point2f>  tempRect_point;
Mat temp2,finalImage;
int scale_arr[400];
int j;
Rect tempRect;
bool drawing_rect;
//resize method

void tracking(Mat frame1,Mat &templ,Rect &rect)
{
    Mat frame2;
    cvtColor(frame1,frame2,cvtColor_method);
    Rect searchWindow;
    searchWindow.width = rect.width * 1.5;
    searchWindow.height = rect.height * 1.5;
    searchWindow.x = rect.x + rect.width * 0.5 - searchWindow.width * 0.5;
    searchWindow.y = rect.y + rect.height * 0.5 - searchWindow.height * 0.5;
    searchWindow &= Rect(0, 0, frame1.cols, frame1.rows);// 防止矩形框出界
    if (rect.width != 0)
    {
        //模板匹配
        Mat resultImage2;
        matchTemplate(frame2(searchWindow), templ, resultImage2, match_method);
        //找出最佳匹配的位置
        double maxVal;
        Point maxLoc;
        minMaxLoc(resultImage2, 0, &maxVal, 0, &maxLoc);         //找到最大匹配值和其对应的位置
        cout << "最大匹配值"<<maxVal << endl;

        if(maxVal > 0.4)
        {
            //cout << "更新模板" << endl;
            //更新模板矩形框
            rect.x = maxLoc.x + searchWindow.x;
            rect.y = maxLoc.y + searchWindow.y;
            //更新模板
            templ = frame2(rect);
            //cout<<templ.size()<<"\t"<<countt<<endl;
            drawing_rect = true;
        }

    }

}

int main()
{
//restart :
    int countFrame = 0;
    int freshcount = 0;
    VideoCapture capture ("/home/robomaster/视频/box/box42_1080_2.avi");
    if (!capture.isOpened())
        cout << "fail to open video" << endl;
    Mat srcImage,srcImage2;

    while (capture.read(srcImage))
    {
         resize(srcImage,srcImage,Size(srcImage.cols*zoomRatio,srcImage.rows*zoomRatio));
        countFrame++;
        //在第一帧识别出方块并更新最符合的模板
        cout << "********** "<< tempRect.br().x << " " << 2*srcImage.rows << endl;
        if (countFrame == 1)
        {

            freshcount++;
            int i;
            //创建度量数组,作为模板列长
            for (i=0;i<zoomMax/zoomSpeed;i++)//srcImage.rows/zoomSpeed
            {
                scale_arr[i] = srcImage.rows - zoomSpeed*i;
            }
            //将模板放到最大
            Mat temp;
            temp = imread("/home/robomaster/LHH/box_detect/template_42.jpg");
            imshow("template",temp);
            waitKey(20);
            resize(temp,temp,Size(srcImage.cols*zoomRatio,srcImage.rows*zoomRatio));
            //将模板变为灰度图减小计算量
            cvtColor(temp,temp2,cvtColor_method);
            //将各个大小的模板进行匹配并记录匹配值
            for (j=0;j<zoomMax/zoomSpeed;j++)//srcImage.rows/zoomSpeed
            {
                cout <<"1: "<< j<<" "<<scale_arr[j] << endl;
                resize(temp2,temp2,Size(scale_arr[j],scale_arr[j]));
                imshow ("temp2",temp2);
                waitKey(20);
                //Mat srcImage2;
                cvtColor(srcImage,srcImage2,cvtColor_method);
                Rect tempRect(0,0,temp2.cols,temp2.rows);
                cout << "*" << tempRect.size()<< endl;
                if (tempRect.width != 0)
                {
                    Mat resultImage;
                    int resultImage_cols = srcImage2.cols - temp2.cols + 1;
                    int resultImage_rows = srcImage2.rows - temp2.rows + 1;
                    resultImage.create(resultImage_cols,resultImage_rows,CV_32FC1);
                    matchTemplate(srcImage2,temp2,resultImage,match_method);
                    //normalize(resultImage,resultImage,0,1,NORM_MINMAX,-1,Mat());
                    double minValue,maxValue,matchValue;
                    Point minLocation,maxLocation,matchLocation;
                    minMaxLoc(resultImage,&minValue,&maxValue,&minLocation,&maxLocation,Mat());
                    if (match_method == TM_SQDIFF_NORMED)
                    {
                        matchLocation = minLocation;
                        matchValue = 1 - minValue;
                    }
                    else
                    {
                        matchLocation = maxLocation;
                        matchValue = maxValue;

                    }
                    cout << "matchValue " << matchValue << endl;
                    match_Value[j] = matchValue;

                    tempRect_point.push_back(matchLocation);
                }

            }
            //得到最大匹配值的最大值的模板的列长
            int max_j;
            double max_matchValue = match_Value[0];
            for (int k =0; k < zoomMax/zoomSpeed;k++)
            {
                if(match_Value[k] > max_matchValue)
                  {
                    max_matchValue = match_Value[k];
                    max_j = k;
                  }
            }
            cout<< "max_matchValue "<< max_matchValue << " max_j " << max_j << endl;
            //得到合适的模板
            resize(temp2,temp2,Size(scale_arr[max_j],scale_arr[max_j]));
            imshow("max_temp",temp2);
            waitKey(20);
            //确定矩形框位置
            tempRect.x = tempRect_point[max_j+(zoomMax/zoomSpeed)*(freshcount-1)].x;
            tempRect.y = tempRect_point[max_j+(zoomMax/zoomSpeed)*(freshcount-1)].y;
            cout << "tempRect.x " << tempRect.x << " tempRect.y "<< tempRect.y << endl;
            tempRect = tempRect + Size(scale_arr[max_j]-tempRect.height,scale_arr[max_j]-tempRect.width);
            Mat matchImage = srcImage.clone();
            rectangle(matchImage,tempRect,Scalar(0,255,0),2);
            imshow("matched",matchImage);
            waitKey(0);
        }
        tracking(srcImage,temp2,tempRect);
        if (drawing_rect)
        {
            finalImage = srcImage.clone();
            rectangle(finalImage, tempRect, Scalar(0, 255, 0), 2);
        }
        imshow("result", finalImage);

        if(waitKey(1) == 27)
            countFrame=0;



        cout << "countFrame " << countFrame << " freshcount " << freshcount <<endl;
   }
     return 0;
}

