#include "armor.h"
#include "Structure.h"

int model_flag = 0;
double sendData;
int usart_0, data_length;
char buf[255];

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

    std::cout<<"the size of received SIGIO signal:"<<data_length << "\n"<<std::endl;
    std::cout<<"the data of received SIGIO signal:"<< buf;
}

int main()
{
    armor::ArmorDetector detector;
    armor::Shooter shooter;
    armor::V4L_capture v4l_capture;
    armor::Serial_commu serial_commu;

    //Ctrl+C后执行sigIntHandler
    signal(SIGINT, sigIntHandler);
    serial_commu.usart_init();

    detector.loopStart(v4l_capture, shooter, serial_commu);

    return 0;
}
