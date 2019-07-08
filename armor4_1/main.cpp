#include "armor.h"
//敌方颜色判定，红色为0,蓝色为1
int enemyColor;
int model_flag=0;
int data;
short sendData[3];


int main() {

    if(model_flag==0)
        enemyColor=0;
    if(model_flag==1)
        enemyColor=1;
    armor::ArmorDetector detector;
    armor::Shooter shooter;
    armor::Serial_commu serial_commu;
    armor::V4L_capture v4l_capture;
    const int fd = serial_commu.serial_Init();
    //std::cout<<sendData[0]<<std::endl;
    //pthread_t id;
    //pthread_create(&id, NULL, serial_commu.serial_read(),&data);
    serial_commu.serial_read(fd, data);
    //pthread_join(id,NULL);

    //t.join();
    detector.loopStart(v4l_capture,shooter,serial_commu);


    return 0;
}
