#include <iostream>
#include <sstream>
#include <math.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <fstream>
#include <signal.h>
#include <stdlib.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <cstdlib>
#include <stdint.h>

void signal_handler_IO (int status);   /* definition of signal handler */
void usart_init();
int set_opt(struct termios newtio,int fd,int nSpeed, int nBits, char nEvent, int nStop);
int send_data(int fd_, const int16_t *buffer);
int send_dataarmor(int fd_, const int *buffer);
int serial_send(int fd_, double *float_data,int len);
int serial_sendarmor(int fd_, double *float_data,int len);
void sigIntHandler(int signal);

extern int model_flag;
extern int usart_0;

