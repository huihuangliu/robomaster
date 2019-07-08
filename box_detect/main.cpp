
#include "armor.h"
#include "serial_commu.h"    //serial communition
#include <iostream>
#include <unistd.h>


using namespace cv;
using namespace std;

Mat src;
int countFrame = 0;


unsigned char enemy_color = 0;
ArmorDetector arm;
Point2f final_direct;

const int fd = serial_Init();
short senddata[2];

int armor()
{
    VideoCapture capture(1);
    //"/home/imagegroup1/桌面/out.avi"

    capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);

    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
    while (1)
    {
        countFrame++;
        double start = (double)getTickCount();
        capture >> src;
        if (!capture.isOpened())
        {
            cout << "capture read error!" << endl;
            return 0;
        }
        if (src.size == 0)
        {
            cout << "video is over!" << endl;
            return 0;
        }

        arm.findArmor(src);

        serial_send(fd, senddata);    //send data
        cout << "sending data:" << *senddata <<"\t"<< *(senddata + 1) << "\n" << endl;

        double end = (double)getTickCount();
        cout << "Frame: " << countFrame << "\tTime: ";
        cout << (end - start) * 1000 / (getTickFrequency()) << "ms" << endl;
        //waitKey(0);
        if (waitKey(1) == 27)
            break;
    }
}



int main()
{
    armor();
    return 0;
}



