#include "matchplate.h"



void MatchTemplate::matching(Mat frame, Mat &templ, Rect &rect)     //模板匹配子函数
{
    Mat hsvFrame;
    cvtColor(frame, hsvFrame, CV_RGB2HSV);

    //选取搜索窗口，在目标周围进行搜索
    Rect searchWindow;
    searchWindow.width = rect.width * 1.2;
    searchWindow.height = rect.height * 1.2;
    searchWindow.x = rect.x + rect.width * 0.5 - searchWindow.width * 0.5;
    searchWindow.y = rect.y + rect.height * 0.5 - searchWindow.height * 0.5;
    searchWindow &= Rect(0, 0, frame.cols, frame.rows);

    //cout << "模板的宽和高:" << templ.size() << "矩形框宽:" << rect.width << "矩形框高:"<< rect.height << endl;

    if (rect.width != 0)

    {
        //模板匹配
        Mat similarity;
        matchTemplate(hsvFrame(searchWindow), templ, similarity, CV_TM_CCOEFF_NORMED);
        //找出最佳匹配的位置
        double maxVal;
        Point maxLoc;
        minMaxLoc(similarity, 0, &maxVal, 0, &maxLoc);         //找到最大匹配值和其对应的位置
        cout << "最大匹配值"<<maxVal << endl;
        imshow("searchWindow", hsvFrame(searchWindow));

        if (maxVal > 0.5)      //只有匹配值大于0.5才进行检测
        {
            //cout << "更新模板" << endl;
            //更新模板矩形框
            rect.x = maxLoc.x + searchWindow.x;
            rect.y = maxLoc.y + searchWindow.y;
            //更新模板
            templ = hsvFrame(rect);
            templ.copyTo(templa);
            drawing_rect = true;
        }
    }


}
