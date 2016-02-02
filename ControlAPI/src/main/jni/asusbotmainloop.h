//
// Created by gaujei on 2015/9/11.
//

#ifndef ASUSBOT_ASUSBOTMAINLOOP_H
#define ASUSBOT_ASUSBOTMAINLOOP_H

#include  <pthread.h>
#include  "include/Threads.h"

int asusbotMain(int *param, int argc);
int pushCommand(uint8_t cmdID, uint8_t cmdSize, uint8_t data[]);

struct iparameter {
    int data[29];   // 15 + 2 (4Byte TimeStamp) + 4*3 BaseTask
};

struct iRawData {
    int data[8];
};

struct iparameter GetSensorStatusFromJNI(int para);

struct iRawData GetSensorRAWFromJNI(int type);


class ASUSBOTSPIN {
public:
    pthread_t asusbot_driver_id;

    ASUSBOTSPIN(void);

    ~ASUSBOTSPIN();

    void InitialThread();
};

#endif //ASUSBOT_ASUSBOTMAINLOOP_H
