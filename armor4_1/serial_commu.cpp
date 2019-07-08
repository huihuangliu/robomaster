#include "armor.h"

#define SEND_NUM 3
#define READ_NUM 1
#define DEFAULT_DEV "/dev/ttyUSB0"
#define DEFAULT_BAUND B115200

//#define DEFAULT_BAUND B9600
#define DEFAULT_DATABIT CS8
#define DEFAULT_PAR ~PARENB
#define DEFAULT_INP ~INPCK
#define DEFAULT_STOPBIT ~CSTOPB
#define DEFAULT_INTERVAL 11


#include <stdint.h>



//串口参考框架
//// 回调（callback）函数，data 的生命周期在 serial_recv_nowait 内完成
//void handle_serial_received(uint8_t *data, size_t len) {
//    if (data[0] == 自喵) {
//        global.xxx_flag = xxx;
//    } else if (...)
//}


//class Serial {
//public:
//    Serial();
//    ~Serial();
//    int serial_init();
//    int serial_send(const uint8_t *data, size_t len);
//    void serial_send_nowait(const uint8_t *data, size_t len);
//private:
//    // 阻塞函数，data 的生命周期在外部
//    int serial_recv(uint8_t *data, size_t len, int timeout=2) {
//        // 超时机制通过 select() 实现
//    }
//    void serial_recv_nowait() {
//        while (退出条件) {
//        // 1. 读取串口数据
//        flag = serial_recv(...);
//        // 2. 判断读取结果：成功读取->3， 超时 continue
//        // 3. 只要读到数据，立刻将数据交给回调函数
//        handle_serial_received(...); 或 global.flag = flag;
//        }
//    }
//    void serial_close();
//private:
//    int fd_;
//}

#include<stdio.h>      /*标准输入输出定义*/
#include<stdlib.h>     /*标准函数库定义*/
#include<unistd.h>     /*Unix 标准函数定义*/
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>      /*文件控制定义*/
#include<termios.h>    /*PPSIX 终端控制定义*/
#include<errno.h>      /*错误号定义*/
#include<string.h>

namespace zst{
/**
 * open port
 * @param  fd
 * @param  comport 想要打开的串口号
 * @return  返回-1为打开失败
 */
//宏定义
#define FALSE  -1
#define TRUE   0


/*******************************************************************
* 名称：                  UART0_Open
* 功能：                打开串口并返回串口设备文件描述
* 入口参数：        fd    :文件描述符     port :串口号(ttyS0,ttyS1,ttyS2)
* 出口参数：        正确返回为1，错误返回为0
*******************************************************************/
int UART0_Open(int fd,char* port)
{

    fd = open( port, O_RDWR|O_NOCTTY|O_NDELAY);
    if (FALSE == fd)
    {
        perror("Can't Open Serial Port");
        return(FALSE);
    }

    //    fcntl(fd, F_SETFL, FNDELAY); //非阻塞
    //    fcntl(fd, F_SETFL, 0); // 阻塞,没收到会等待读取数据
    //恢复串口为阻塞状态
    if(fcntl(fd, F_SETFL, 0) < 0)
    {
        printf("fcntl failed!\n");
        return(FALSE);
    }
    else
    {
        printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));
    }
    //测试是否为终端设备
    if(0 == isatty(STDIN_FILENO))
    {
        printf("standard input is not a terminal device\n");
        return(FALSE);
    }
    else
    {
        printf("isatty success!\n");
    }
    printf("fd->open=%d\n",fd);
    return fd;
}
/*******************************************************************
* 名称：                UART0_Close
* 功能：                关闭串口并返回串口设备文件描述
* 入口参数：        fd    :文件描述符     port :串口号(ttyS0,ttyS1,ttyS2)
* 出口参数：        void
*******************************************************************/

void UART0_Close(int fd)
{
    close(fd);
}

/*******************************************************************
* 名称：                UART0_Set
* 功能：                设置串口数据位，停止位和效验位
* 入口参数：        fd        串口文件描述符
*                              speed     串口速度
*                              flow_ctrl   数据流控制
*                           databits   数据位   取值为 7 或者8
*                           stopbits   停止位   取值为 1 或者2
*                           parity     效验类型 取值为N,E,O,,S
*出口参数：          正确返回为1，错误返回为0
*******************************************************************/
int UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)
{

    int   i;
    int   status;
    int   speed_arr[] = { B115200, B19200, B9600, B4800, B2400, B1200, B300};
    int   name_arr[] = {115200,  19200,  9600,  4800,  2400,  1200,  300};

    struct termios options;

    /*tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,
     该函数还可以测试配置是否正确，该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.*/
    if( tcgetattr( fd,&options)  !=  0)
    {
        perror("SetupSerial 1");
        return(FALSE);
    }

    //设置串口输入波特率和输出波特率
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);i++)
    {
        if  (speed == name_arr[i])
        {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }

    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;

    //设置数据流控制
    switch(flow_ctrl)
    {

    case 0 ://不使用流控制
        options.c_cflag &= ~CRTSCTS;
        break;

    case 1 ://使用硬件流控制
        options.c_cflag |= CRTSCTS;
        break;
    case 2 ://使用软件流控制
        options.c_cflag |= IXON | IXOFF | IXANY;
        break;
    }
    //设置数据位
    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
    case 5    :
        options.c_cflag |= CS5;
        break;
    case 6    :
        options.c_cflag |= CS6;
        break;
    case 7    :
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr,"Unsupported data size\n");
        return (FALSE);
    }
    //设置校验位
    switch (parity)
    {
    case 'n':
    case 'N': //无奇偶校验位。
        options.c_cflag &= ~PARENB;
        options.c_iflag &= ~INPCK;
        break;
    case 'o':
    case 'O'://设置为奇校验
        options.c_cflag |= (PARODD | PARENB);
        options.c_iflag |= INPCK;
        break;
    case 'e':
    case 'E'://设置为偶校验
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        options.c_iflag |= INPCK;
        break;
    case 's':
    case 'S': //设置为空格
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
    default:
        fprintf(stderr,"Unsupported parity\n");
        return (FALSE);
    }
    // 设置停止位
    switch (stopbits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB; break;
    case 2:
        options.c_cflag |= CSTOPB; break;
    default:
        fprintf(stderr,"Unsupported stop bits\n");
        return (FALSE);
    }

    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    //options.c_lflag &= ~(ISIG | ICANON);

    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */

    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(fd,TCIFLUSH);

    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("com set error!\n");
        return (FALSE);
    }
    return (TRUE);
}

/*******************************************************************
* 名称：                UART0_Init()
* 功能：                串口初始化
* 入口参数：        fd       :  文件描述符
*               speed  :  串口速度
*                              flow_ctrl  数据流控制
*               databits   数据位   取值为 7 或者8
*                           stopbits   停止位   取值为 1 或者2
*                           parity     效验类型 取值为N,E,O,,S
*
* 出口参数：        正确返回为1，错误返回为0
*******************************************************************/
int UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
    int err;
    //设置串口数据帧格式
    if (UART0_Set(fd,19200,0,8,1,'N') == FALSE)
    {
        return FALSE;
    }
    else
    {
        return  TRUE;
    }
}
/*******************************************************************
* 名称：                  UART0_Recv
* 功能：                接收串口数据
* 入口参数：        fd                  :文件描述符
*                              rcv_buf     :接收串口中数据存入rcv_buf缓冲区中
*                              data_len    :一帧数据的长度
* 出口参数：        正确返回为1，错误返回为0
*******************************************************************/
int UART0_Recv(int fd, char *rcv_buf,int data_len)
{
    int len,fs_sel;
    fd_set fs_read;

    struct timeval time;

    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);

    time.tv_sec = 10;
    time.tv_usec = 0;

    //使用select实现串口的多路通信
    fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);
    printf("fs_sel = %d\n",fs_sel);

    if(fs_sel)
    {
        len = read(fd,rcv_buf,data_len);
        printf("I am right! len = %d fs_sel = %d\n",len,fs_sel);
        return len;
    }
    else
    {
        printf("Sorry,I am wrong!");
        return FALSE;
    }
}
/********************************************************************
* 名称：                  UART0_Send
* 功能：                发送数据
* 入口参数：        fd                  :文件描述符
*                              send_buf    :存放串口发送数据
*                              data_len    :一帧数据的个数
* 出口参数：        正确返回为1，错误返回为0
*******************************************************************/
int UART0_Send(int fd, char *send_buf,int data_len)
{
    int len = 0;

    len = write(fd,send_buf,data_len);
    if (len == data_len )
    {
        printf("send data is %s\n",send_buf);
        return len;
    }
    else
    {

        tcflush(fd,TCOFLUSH);
        return FALSE;
    }
}
}


int armor::Serial_commu::serial_Init()    //initialize serial port
{
    int fd;
    char *dev_name = DEFAULT_DEV;
    if((fd = open_dev(DEFAULT_DEV)) == -1)   // /dev/ttyUSB0
    {
        perror("open error!");
        return -1;
    }
    if(set_port(fd) == -1)
    {
        perror("set error!");
        return -1;
    }
    return fd;
}


int armor::Serial_commu::serial_Close(int fd)
{
    close(fd);
    return 0;
}

int armor::Serial_commu::serial_send(int fd, short *float_data)     //send data
{
    short ff = 255;      //first data as sign
    send_data(fd,&ff);
    for(int i = 0; i < SEND_NUM; i++)
    {
        send_data(fd, &float_data[i]);
    }
    short sumdata = float_data[0] + float_data[1] + float_data[2] + 255;
    send_data(fd, &sumdata);
    //std::cout<<"serial is sending......"<<std::endl;
    return 0;
}

int armor::Serial_commu::serial_read(int fd, int data)     //data read
{
//    //    int flagdata = 0;
//    const int fd = serial_commu.serial_Init();


//    //    read (fd, &flagdata, 1);
//    //    if(flagdata ==255)
//    //    {
//    //        for(int i = 0; i < READ_NUM; i++)
//    //        {
//    //            read(fd, &data[i], 1);
//    //        }
//    //    }
//    //    std::cout<<"rd_data :"<<data[0]<<"\t"<<data[1]<<std::endl;

//    //    return 0;
    while(1){
        fd_set rd;
        int nread;
        struct timeval timeout;    //超时设置参数结构体

        FD_ZERO(&rd);
        FD_SET(fd,&rd);
        while(FD_ISSET(fd,&rd))
        {
            if(select(fd+1,&rd,NULL,NULL,&timeout) < 0)   //在此阻塞
                perror("select error!\n");
            else
            {
                while((nread = read(fd,&data,2))>0)
                {
                    printf("nread = %d, %x\n",nread,data);
                }
            }
        }
    }
}






// 等待时间  串口描述符  读取的buff
int armor::Serial_commu::Select_Tart_Read(struct timeval tv,int  uart_fd,int* buff)
{

    memset(buff,0,8);
    fd_set rfds;
    int retval=0;
    //int i;

    FD_ZERO(&rfds);     //数据集清零
    FD_SET(uart_fd, &rfds);     //绑定描述符与数据集

    retval=select(uart_fd + 1, &rfds, NULL, NULL, &tv);     //查询改描述符关联设备是否存在数据可读
    if(retval<0)
    {
        perror("select error\n");
    }
    else
    {
        if(retval && FD_ISSET(uart_fd, &rfds))
        {
            //testi++;
            //printf("FD_ISSET!\n");
            int rc=read(uart_fd, buff, 8);
            if(rc>0)
            {
                //printf("recv serial----rc is %d   testi is %d \n  ",rc,testi);
                //for(i = 0; i < 1; i++)
                //printf(" %x ", buff[i]);
                //printf("\n");
                return rc;
            }
        }
    }
    return 0;
}

int armor::Serial_commu::monitor_routine( int  uart_fd, int* new_data)
{
    int i;
    printf("in  monitor_routine\n");

    //int rebuff[2];//设置最大的数据长度为8个
    //rebuff = new_data;
    memset(new_data,0,8);
    struct timeval tv;
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    //int retval;
    //unsigned int commbuff[1];//设置最大的数据长度为8个
    //memset(commbuff,0,8);

    while(1)
    {
        //printf("retval is %d\n",retval);
        if(Select_Tart_Read(tv,uart_fd,new_data)>0)   //存在可读数据，准备读取
        {
            printf("in while Read!\n");
            printf(" %x ", new_data);

        }
    }
    return 0;
}

int armor::Serial_commu::open_dev(const char *dev_name)     //open a device
{
    //return open(dev_name, O_NONBLOCK | O_RDWR);
    return open(dev_name, O_RDWR);
}

int armor::Serial_commu::set_port(const int fd)     //port configuration
{
    struct termios opt;
    if(tcgetattr(fd, &opt) != 0)
    {
        printf("set error 1\n");
        return -1;
    }
    cfsetispeed(&opt, DEFAULT_BAUND);
    cfsetospeed(&opt,DEFAULT_BAUND);
    tcsetattr(fd,TCSANOW,&opt);

    opt.c_cflag &= ~CSIZE;
    opt.c_cflag |= DEFAULT_DATABIT;
    opt.c_cflag &= DEFAULT_PAR;
    opt.c_iflag &= DEFAULT_INP;
    opt.c_cflag &= DEFAULT_STOPBIT;

    tcflush(fd, TCIFLUSH);
    opt.c_cc[VTIME] = DEFAULT_INTERVAL;
    opt.c_cc[VMIN] = 0;
    if(tcsetattr(fd, TCSANOW, &opt) != 0)
    {
        printf("set error 1\n");
        return -1;
    }
    return 0;
}

int armor::Serial_commu::send_data(const int fd, const short *buffer)
{
    //printf("\tsend:%d\n", *buffer);
    return write(fd, (char*)buffer, 2);
}
/******************************************
 信号处理函数，设备wait_flag=FASLE
 ******************************************************/
