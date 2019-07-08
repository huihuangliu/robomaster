#include "armor.h"

#define MODEMDEVICE "/dev/ttyUSB0"

extern int model_flag;
extern int usart_0, data_length;
extern char buf[255];

void signal_handler_IO (int status);

int send_data(int fd_, const int16_t *buffer)
{
    //printf("\tsend:%d\n", *buffer);
    return write(fd_, (char*)buffer, sizeof(uint16_t));
}
int send_dataarmor(int fd_, const int *buffer)
{
    //printf("\tsend:%d\n", *buffer);
    return write(fd_, (char*)buffer, sizeof(uint16_t));
}

int serial_send(int fd_, double *float_data,int len) // If recognized, add 1 to data_check; Give out 4 data in total: ff, x, y, (x+y+255+recognize).
{
    std::vector<int16_t> data;
    int16_t data_check;


    for (int i=0;i<len;i++)
    {
        data.push_back((int16_t)(float_data[i] * 100));
        data_check += data[i];
    }
    data_check+=255;
    //std::cout << data[0] <<" data "<< data[1] << std::endl;

    int16_t ff = 255;
    //int16_t recog_check = recog;

    send_data(fd_, &ff);
    //write(fd, (char*)buffer, sizeof(char));
    for (int i = 0; i < len; i++)
    {
        //printf("data%d:", i + 1);
        send_data(fd_, &data[i]);
    }

    send_data(fd_, &data_check);

    return 0;
}
int serial_sendarmor(int fd_, double *float_data,int len) // If recognized, add 1 to data_check; Give out 4 data in total: ff, x, y, (x+y+255+recognize).
{
    std::vector<int> data;
    int data_check=0;


    for (int i=0;i<len;i++)
    {
        data.push_back((int)(float_data[i]));
        //data.push_back((int)(2));
        data_check += data[i];
    }
    data_check+=255;
    //std::cout << data[0] <<" data "<< data[1] << std::endl;

    int ff = 255;
    //int16_t recog_check = recog;

    send_dataarmor(fd_, &ff);
    //write(fd, (char*)buffer, sizeof(char));
    for (int i = 0; i < len; i++)
    {
        //printf("data%d:", i + 1);
        send_dataarmor(fd_, &data[i]);
    }

    send_dataarmor(fd_, &data_check);

    return 0;
}

int set_opt(struct termios newtio,int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
    /*步骤一，设置字符大小*/
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;
    /*设置数据位*/
    switch( nBits )
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }
    /*设置奇偶校验位*/
    switch( nEvent )
    {
    case 'O': //奇数
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E': //偶数
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N': //无奇偶校验位
        newtio.c_cflag &= ~PARENB;
        break;
    }
    /*设置波特率*/
    switch( nSpeed )
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    /*设置停止位*/
    if( nStop == 1 )
        newtio.c_cflag &= ~CSTOPB;
    else if ( nStop == 2 )
        newtio.c_cflag |= CSTOPB;
    /*设置等待时间和最小接收字符*/
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    /*处理未接收字符*/
    tcflush(fd,TCIFLUSH);
    /*激活新配置*/
    if((tcsetattr(fd,TCSANOW,&newtio))!=0)
    {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");
    return 0;
}

void usart_init()
{
    //__sighandler_t signal_handler_IO;
    struct termios signal_usart;
    struct sigaction saio;           /* definition of signal action */

    /* open the device to be non-blocking (read will return immediatly) */
    usart_0 = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (usart_0 <0)
    {
        std::cout<<"usart_0<0";
        perror(MODEMDEVICE); return ;
    }

    saio.sa_handler = signal_handler_IO;
    sigemptyset(&saio.sa_mask);
    saio.sa_flags = 0;
    saio.sa_restorer = NULL;
    sigaction(SIGIO,&saio,NULL);

    fcntl(usart_0, F_SETOWN, getpid());//allow the process to receive SIGIO
    fcntl(usart_0, F_SETFL, FASYNC);//恢复串口的状态为阻塞状态，用于等待串口数据的读入
    set_opt(signal_usart,usart_0,115200,8,'N',1);
}



