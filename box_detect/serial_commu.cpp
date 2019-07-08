#include "serial_commu.h"


int serial_Init()    //initialize serial port
{
    int fd;
    char *dev_name = DEFAULT_DEV;
    if((fd = open_dev(DEFAULT_DEV)) == FALSE)   // /dev/ttyUSB0
    {
        perror("open error!");
        return -1;
    }
    if(set_port(fd) == FALSE)
    {
        perror("set error!");
        return -1;
    }
    return fd;
}


int serial_Close(int fd)
{
    close(fd);
    return 0;
}

int serial_send(int fd, short *float_data)     //send data
{
    short ff = 255;      //first data as sign
    send_data(fd,&ff);
    for(int i = 0; i < SEND_NUM; i++)
    {
        send_data(fd, &float_data[i]);
    }
    short sumdata = float_data[0] + float_data[1] + 255;
    send_data(fd, &sumdata);
    return 0;
}

int serial_read(int fd, short *data)     //data read
{
    int flagdata = 0;
    read (fd, &flagdata, 1);
    if(flagdata ==255)
    {
        for(int i = 0; i < READ_NUM; i++)
        {
            read(fd, &data[i], 1);
        }
    }
    return 0;
}

int open_dev(const char *dev_name)     //open a device
{
    //return open(dev_name, O_NONBLOCK | O_RDWR);
    return open(dev_name, O_RDWR);
}

int set_port(const int fd)     //port configuration
{
    struct termios opt;
    if(tcgetattr(fd, &opt) != 0)
    {
        printf("set error 1\n");
        return FALSE;
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
        return FALSE;
    }
    return TRUE;
}

int send_data(const int fd, const short *buffer)
{
    //printf("\tsend:%d\n", *buffer);
    return write(fd, (char*)buffer, sizeof(short));
}

