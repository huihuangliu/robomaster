//
// Created by parallels on 4/27/17.
//
#include <stdio.h>
#include <sys/shm.h>
#include <sys/ipc.h>
#include <stdarg.h>
#include <stdio.h>
#include <pthread.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#ifndef VIDEOSERVER_STRUCTURE_H
#define VIDEOSERVER_STRUCTURE_H

using namespace cv;
extern long long frame_count_last;
extern FILE *flog;
extern uint8_t image_buffer[921600];        //640*480图像数据
extern int image_buffer_len;
void printlog(const char *format,...);
uint8_t getImageFromMemory();
uint8_t getImageFromMemory(Mat &image);

struct shared_package{
    pthread_rwlock_t image_lock=PTHREAD_RWLOCK_INITIALIZER;
    int image_size;
    char image_data[921600];        //640*480图像数据
    long long count;

};
struct shared_package * get_shared_package();


#endif //VIDEOSERVER_STRUCTURE_H
