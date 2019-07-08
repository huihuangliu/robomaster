#include"serial_commu.h"
#include <iostream>
#include <unistd.h>
//!static params
const int fd = serial_Init();

//!define serial and data 640
short senddata[] = {5,9};
int readdata[3] = {0};

using namespace std;

int main()
{
    //int flagdata = 0;

    while(1)
    {
        serial_send(fd, senddata);    //send data
        cout << "sending data:" << *senddata <<"\t"<< *(senddata + 1) << "\n" << endl;
        //read(fd, &flagdata, 1);     //read data
        //cout<<"receiving data:"<<flagdata<<endl;
    }
    return 0;
}
