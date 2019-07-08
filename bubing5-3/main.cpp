#include "armor.h"
#include "Structure.h"
#include "buff.h"
#include "serial_commu.hpp"

//(buf[0] == 0x32) model_flag = 4;         // big buff
//(buf[0] == 0x31) model_flag = 3;    //small buff
//(buf[0] == 0x30) model_flag = -1;   //exit
//(buf[0] == 0x41) model_flag = 0;   //armor_red
//(buf[0] == 0x42) model_flag = 1;   // armor_blue

int model_flag = 3;
double sendData_armor;
double sendData_buff;
int usart_0, data_length;
char buf[255];

int debug_flag = 1;

int title_h_L = 0, title_h_H = 180;//znj
int title_s_L = 0, title_s_H = 160;//znj
int title_v_L = 189, title_v_H = 255;//znj
int title_hight_L = 10, title_hight_H = 70;
int title_width_L = 5, title_width_H = 75;
int SMG_height_difference = 70;
int SMG_width_difference = 255;
int SMG_x_difference = 255;
int SMG_y_difference = 100;

int context_h_L = 0, context_h_H = 180;
int context_s_L = 0, context_s_H = 130;
int context_v_L = 74, context_v_H = 233;
int context_hight_L = 20, context_hight_H = 120;
int context_width_L = 20, context_width_H = 140;
int HW_height_difference = 209;
int HW_width_difference = 209;
int HW_x_difference = 40;
int HW_y_difference = 40;

int fire_context_h_L = 0, fire_context_h_H = 180;
int fire_context_s_L = 0, fire_context_s_H = 255;
int fire_context_v_L = 48, fire_context_v_H = 162;
int fire_context_hight_L = 32, fire_context_hight_H = 120;
int fire_context_width_L = 20, fire_context_width_H = 140;
int fire_y_difference = 150;
int fire_x_difference = 150;

int adjust_context_flag = 0;
int adjust_title_flag = 0;

std::vector<cv::Rect> rect_context_backup;
Mat image_RGB_backup;
int draw_target_context=0;
int rectangle_flag=0;

PyObject *pModule;
PyObject *pFunc_SMG, *pFunc_HW, *pFunc_fire;
int count;
#if record2
struct tm *tblock;
string name="";
string doc=".avi";
void Inittime()
{
    time_t timer;

    timer=time(NULL);
    tblock=localtime(&timer);
    cout<<asctime(tblock);
    name=asctime(tblock)+doc;

}
#endif

void sigIntHandler(int signal)
{
    std::cout<<"\nHonestly, you are out!"<<std::endl;
    exit(0); // It can free the data.
}
void signal_handler_IO (int status)     /* definition of signal handler */
{
    data_length = read(usart_0,buf,255);
    if( data_length == 1)
        if (buf[0] == 0x32) model_flag = 4;         // big buff
        else if (buf[0] == 0x31) model_flag = 3;    //small buff
        else if (buf[0] == 0x30) model_flag = -1;   //exit
        else if (buf[0] == 0x41) model_flag = 0;   //armor_red
        else if (buf[0] == 0x42) model_flag = 1;   // armor_blue
}
void python_init(){
    Py_Initialize();

    // 检查初始化是否成功
    if (!Py_IsInitialized())
       exit(1);

    PyRun_SimpleString("import sys");
    PyRun_SimpleString("import os");
    PyRun_SimpleString("import ctypes");
    PyRun_SimpleString("sys.path.append('./')");
    PyRun_SimpleString("sys.path.append('/home/allspark003/bubing5-3/build-bubing5-3-unknown-Debug')");
    //PyRun_SimpleString("print(sys.path)");

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
int main()
{
    python_init();
    armor::ArmorDetector detector;
    armor::Shooter shooter;
    VideoWriter out;
    Buff::BuffDetector Detector;
    Buff::Angle_analysis angle;
    Buff::Handle handle;

    //Ctrl+C后执行sigIntHandler
    signal(SIGINT, sigIntHandler);
    usart_init();

#if record2

  while(1)
  {

      Inittime();
      VideoWriter out(name,CV_FOURCC('M','J','P','G'),30,Size(640,480));
#endif
    while(1)
    {
  //     if((detector.loopStart(shooter,out))==1)
  //         break;
        switch (model_flag)
        {
        case 0:
            detector.loopStart(shooter,out);//armor_red
            break;
        case 1:
            detector.loopStart(shooter,out);   //armor_blue
            break;
        case 3:
            Detector.loopStart(handle,angle,out);  //small buff
            break;
        case 4:
            Detector.loopStart(handle,angle,out);   //big buff
        case -1:
        {
            cv::waitKey(1);
        }

            break;
        default:
        {}
            break;
        }
    }
#if record2
  }
#endif
    Py_Finalize();
    return 0;
}
