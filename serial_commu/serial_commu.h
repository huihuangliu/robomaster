#ifndef _SERIAL_COMMU_
#define _SERIAL_COMMU_
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>

#define TRUE 0
#define FALSE -1
#define SEND_NUM 2
#define READ_NUM 3
#define DEFAULT_DEV "/dev/ttyUSB0"
#define DEFAULT_BAUND B115200

//#define DEFAULT_BAUND B9600
#define DEFAULT_DATABIT CS8
#define DEFAULT_PAR ~PARENB
#define DEFAULT_INP ~INPCK
#define DEFAULT_STOPBIT ~CSTOPB
#define DEFAULT_INTERVAL 11


int open_dev(const char *dev_name);
int set_port(const int fd);
int send_data(const int fd, const short *buffer);
short read_data(const int fd);

//******************************************************
int serial_Init();  //Init the serial
int serial_Close(int fd);  //Close the serial
int serial_send(int fd, short *float_data);
int serial_read(int fd, int *data);
//********************************************************
# endif
