#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>


using namespace cv;
using namespace std;

int orb_match(Mat &image1, Mat &image2)
{
    Mat img_1 = image1;
    Mat img_2 = image2;
    // -- Step 1: Detect the keypoints using STAR Detector
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    int nkeypoint = 50;//特征点个数
    //Ptr<ORB> orb = ORB::create(nkeypoint);
    Ptr<ORB> orb = ORB::create(nkeypoint);

    orb->detect(img_1, keypoints_1);
    orb->detect(img_2, keypoints_2);

    // -- Stpe 2: Calculate descriptors (feature vectors)
    Mat descriptors_1, descriptors_2;
    orb->compute(img_1, keypoints_1, descriptors_1);
    orb->compute(img_2, keypoints_2, descriptors_2);

    //-- Step 3: Matching descriptor vectors with a brute force matcher
    BFMatcher matcher(NORM_HAMMING);
    std::vector<DMatch> mathces;
    matcher.match(descriptors_1, descriptors_2, mathces);
    // -- dwaw matches
    Mat img_mathes;
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, mathces, img_mathes);
    // -- show
    imshow("OPENCV_ORB", img_mathes);
    return 0;
}

int main()
{

    Mat img_2 = imread("/home/robomaster/LHH/box_detect/template_42.jpg");
    VideoCapture capture("/home/robomaster/LHH/box_detect/box_42.mp4");

    if(!capture.isOpened())
    {
        cout<<"video not open."<<endl;
        return 1;
    }

    Mat frame;
    bool stop(false);
    while(!stop)
    {
        if(!capture.read(frame))
        {
            cout<<"no video frame"<< endl;
            break;
         }
        resize(frame, frame, Size(frame.cols/2,frame.rows/2), 0, 0,INTER_LINEAR);
//        imshow("video",frame);

        Mat img_1 = frame;

        //resize(img_2, img_2, Size(img_2.cols/4,img_2.rows/4), 0, 0,INTER_LINEAR);

        orb_match(img_1, img_2);

        if(cv::waitKey(30)>0)
        stop = true;
    }

    return 0;
}
