#include "armor.h"
#include "Structure.h"
#include "buff.h"
#include "serial_commu.hpp"

int model_flag = 0;
double sendData_armor;
double sendData_buff;
int usart_0, data_length;
char buf[255];

int title_h_L = 0, title_h_H = 180;//znj
int title_s_L = 0, title_s_H = 255;//znj
int title_v_L = 70, title_v_H = 255;//znj
int context_h_L = 0, context_h_H = 180;
int context_s_L = 0, context_s_H = 255;
int context_v_L = 70, context_v_H = 255;

void sigIntHandler(int signal)
{
    std::cout<<"\nHonestly, you are out!"<<std::endl;
    exit(0); // It can free the data.
}

void signal_handler_IO (int status)     /* definition of signal handler */
{
    data_length = read(usart_0,buf,255);
    if( data_length == 1)
        if (buf[0] == 0x32) model_flag = 1;         //armor_blue 32
        else if (buf[0] == 0x31) model_flag = 0;    //armor_red  31
        else if (buf[0] == 0x30) model_flag = -1;   //exit
        else if (buf[0] == 0x41) model_flag = 3;   //small buff
        else if (buf[0] == 0x42) model_flag = 4;   //big buff
}

int main()
{
    armor::ArmorDetector detector;
    armor::Shooter shooter;

    Buff::BuffDetector Detector;
    Buff::Angle_analysis angle;
    Buff::Handle handle;

    //Ctrl+C后执行sigIntHandler
    signal(SIGINT, sigIntHandler);
    usart_init();

    while(1)
    {
        switch (model_flag)
        {
        case 0:
            detector.loopStart(shooter);//armor_red
            break;
        case 1:
            detector.loopStart(shooter);   //armor_blue
            break;
        case 3:
            Detector.loopStart(handle,angle);  //small buff
            break;
        case 4:
            Detector.loopStart(handle,angle);   //big buff
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

    return 0;
}
