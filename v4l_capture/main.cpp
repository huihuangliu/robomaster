#include "errno.h"
#include "fcntl.h"
#include "linux/videodev2.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "sys/ioctl.h"
#include "sys/mman.h"
#include "unistd.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include "cv.hpp"

using namespace std;
using namespace cv;

static uchar *buffer;

//set imagewidth and imageheight
#define IMAGEWIDTH 1080
#define IMAGEHEIGHT 720

#define TRUE 1
#define FALSE 0

//set the device
#define FILE_VIDEO1 "/dev/video0"

static int fd;
static struct v4l2_streamparm setfps;
static struct v4l2_capability cap;
static struct v4l2_fmtdesc fmtdesc;
static struct v4l2_format fmt,fmtack;
static struct v4l2_requestbuffers req;
static struct v4l2_buffer buf;
static enum   v4l2_buf_type type;

static int init_v4l2(void);
static int v4l2_grab(void);

int init_v4l2(void){
        if ((fd = open(FILE_VIDEO1, O_RDWR)) == -1){
            printf("Opening video device error\n");
            return FALSE;
        }
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1){
                printf("unable Querying Capabilities\n");
                return FALSE;
        }
        else

        {
        printf( "Driver Caps:\n"
                "  Driver: \"%s\"\n"
                "  Card: \"%s\"\n"
                "  Bus: \"%s\"\n"
                "  Version: %d\n"
                "  Capabilities: %x\n",
                cap.driver,
                cap.card,
                cap.bus_info,
                cap.version,
                cap.capabilities);

        }

        if((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == V4L2_CAP_VIDEO_CAPTURE){
            printf("Camera device %s: support capture\n",FILE_VIDEO1);
        }
        if((cap.capabilities & V4L2_CAP_STREAMING) == V4L2_CAP_STREAMING){
            printf("Camera device %s: support streaming.\n",FILE_VIDEO1);
        }

        fmtdesc.index = 0;
        fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        printf("Support format: \n");
        while(ioctl(fd,VIDIOC_ENUM_FMT,&fmtdesc) != -1){
            printf("\t%d. %s\n",fmtdesc.index+1,fmtdesc.description);
            fmtdesc.index++;
        }
        //set fmt
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = IMAGEWIDTH;
        fmt.fmt.pix.height = IMAGEHEIGHT;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG; //V4L2_PIX_FMT_YUYV***V4L2_PIX_FMT_MJPEG
        fmt.fmt.pix.field = V4L2_FIELD_NONE;

        if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1){
            printf("Setting Pixel Format error\n");
            return FALSE;
        }
        if(ioctl(fd,VIDIOC_G_FMT,&fmt) == -1){
            printf("Unable to get format\n");
            return FALSE;
        }
//        else

/*        {
            printf("fmt.type:\t%d\n",fmt.type);
            printf("pix.pixelformat:\t%c%c%c%c\n",fmt.fmt.pix.pixelformat & 0xFF,(fmt.fmt.pix.pixelformat >> 8) & 0xFF,\
                   (fmt.fmt.pix.pixelformat >> 16) & 0xFF, (fmt.fmt.pix.pixelformat >> 24) & 0xFF);
            printf("pix.height:\t%d\n",fmt.fmt.pix.height);
            printf("pix.field:\t%d\n",fmt.fmt.pix.field);
        }
*/
/*
        setfps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        setfps.parm.capture.timeperframe.numerator = 100;
        setfps.parm.capture.timeperframe.denominator = 100;
        printf("init %s is OK\n",FILE_VIDEO1);
*/
        return TRUE;
}

int v4l2_grab(void){
    //struct v4l2_requestbuffers req = {0};
    //4  request for 4 buffers
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;   //use memory mmap

    if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1)
    {
        printf("Requesting Buffer error\n");
        return FALSE;
    }
    //5 mmap for buffers
    buffer = (uchar*)malloc(req.count * sizeof(*buffer));
    if(!buffer){
        printf("Out of memory\n");
        return FALSE;
    }
    unsigned int n_buffers;
    for(n_buffers = 0;n_buffers < req.count; n_buffers++){
    //struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers;
    if(ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1){
        printf("Querying Buffer error\n");
        return FALSE;
        }

    buffer = (uchar*)mmap (NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);

    if(buffer == MAP_FAILED){
        printf("buffer map error\n");
        return FALSE;
        }
    printf("Length: %d\nAddress: %p\n", buf.length, buffer);
    printf("Image Length: %d\n", buf.bytesused);
    }
    //6 queue
    for(n_buffers = 0;n_buffers <req.count;n_buffers++){
        buf.index = n_buffers;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if(ioctl(fd,VIDIOC_QBUF,&buf)){
            printf("query buffer error\n");
            return FALSE;
        }
    }
    //7 starting
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(ioctl(fd,VIDIOC_STREAMON,&type) == -1){
        printf("stream on error\n");
        return FALSE;
    }
    return TRUE;
}



int main()
{
        printf("first~~\n");
        if(init_v4l2() == FALSE){
            printf("Init fail~~\n");
            exit(1);
        }
        printf("second~~\n");
        if(v4l2_grab() == FALSE){
            printf("grab fail~~\n");
            exit(2);
        }
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        printf("third~~\n");

        cvNamedWindow("one",CV_WINDOW_AUTOSIZE);
        IplImage* img;
        CvMat cvmat;
        CvMat ImgGray;


        int i = 100;
        double t;
        while(1){
                t = (double)cvGetTickCount();
                ioctl(fd,VIDIOC_DQBUF,&buf);
                buf.index = 0;
                cvmat = cvMat(IMAGEHEIGHT,IMAGEWIDTH,CV_8UC3,(void*)buffer);//CV_8UC3
                //t = (double)cvGetTickCount();
                img = cvDecodeImage(&cvmat,1);     //decode image
                //t=(double)cvGetTickCount()-t;
                //printf("used time is %gms\n",(t/(cvGetTickFrequency()*1000)));
                if(!img)    printf("No img\n");
                //cvCvtColor(img, ImgGray, CV_BGR2GRAY);
                cvShowImage("src",img);
                cvShowImage("ImgGray",img);
                cvReleaseImage(&img);
                ioctl(fd,VIDIOC_QBUF,&buf);
                if((cvWaitKey(1)&255) == 27)    exit(0);
                t=(double)cvGetTickCount()-t;
                printf("used time is %gms\n",(t/(cvGetTickFrequency()*1000)));
        }

        ioctl(fd,VIDIOC_STREAMOFF,&type);
        return 0;
}


