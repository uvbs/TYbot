//
// Created by gaujei on 2015/9/8.
//
#include <string.h>
#include <jni.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "DriverInterface.h"
#include <com_asus_control_ControlAPI.h>
#include "asusbotmainloop.h"
#include "include/DebugMessage.h"

#define LOG_TAG  "ASUS_DriverInterface"

#include <vector>
#include "include/Initialization.h"
#include "include/PathFinding.h"
#include "include/EnumDefine.h"
#include "include/asusbot.h"

std::vector <std::vector<int> > Table;
// uint8_t StatusArr[12] = {0};  // refer to 84 (int size = 12;)
using namespace asusbot;
ASUSBOTSPIN mspin;
//using namespace asusbot;
//Asusbot Driver;

//JNIEXPORT jstring JNICALL Java_tw_blogspot_gaujei_asusbot_MainActivity_getStringFromNative
//(JNIEnv * evn, jobject obj)
//{
//    return evn->NewStringUTF("Hello from NDK~~mint~^^");
//}
//jstring Java_com_asus_control_ControlAPI_getStringFromNative(
//        JNIEnv *evn, jobject obj) {
//
//    return evn->NewStringUTF("Hi~");
//}

jstring Java_com_asus_control_ControlAPI_NativeSetControlParameter(
        JNIEnv *env, jobject thiz, jintArray strIn) {
    //jbytearray strIn

    jint *olddata = (jint *) env->GetIntArrayElements(strIn, 0);
    jsize oldsize = env->GetArrayLength(strIn);
    int *intarr = (int *) olddata;

#ifdef DRIVER_INTERFACE
    LOGD("data length =%d", oldsize);

//    for(int j=0;j<9;j++)
//    	LOGD("data =%d",intarr[j]);
#endif

    asusbotMain(intarr, oldsize);
    mspin.InitialThread();

    env->ReleaseIntArrayElements(strIn, olddata, 0);

    if ((intarr[0] == 0) && (intarr[1] == 0) && (intarr[2] == 0)) {
        return env->NewStringUTF("Initial...");
    }
    else if (intarr[0] == 1) {
        return env->NewStringUTF("MobileBase move...");
    }
    else if (intarr[0] == 0x04) {
        return env->NewStringUTF("Neck Control Complete ...");
    }
    else if (intarr[0] == 0x08) {
        return env->NewStringUTF("Request Sensor Raw Date...");
    }
    else {
        return env->NewStringUTF("CommandSender");
    }

}

#ifdef DEBUG_FAKETIMESTAMP
int fakeTimeStamp = 0;
#endif

jintArray Java_com_asus_control_ControlAPI_getStatus
        (JNIEnv *env, jobject thiz, jint para) {

    struct iparameter data = GetSensorStatusFromJNI(para);

    int size = 17;
    jintArray newIntArray = (env)->NewIntArray(size);
    int *arr = new int[size];

    for (int i = 0; i < size; i++) {
        *(arr + i) = data.data[i];
        // StatusArr[i] = data.data[i];
    }

    for (int i = 0; i < size; i++) {
#ifdef DRIVER_INTERFACE
        LOGD("ASUS_JNI -> arr[%d] = %d",i,*(arr+i));
#endif
    }

#ifdef DEBUG_FAKETIMESTAMP
    if (fakeTimeStamp < 60000)
        fakeTimeStamp += 20;
    else
        fakeTimeStamp = 0;
#endif

    (env)->SetIntArrayRegion(newIntArray, 0, size, arr);
    delete[] arr;

    return newIntArray;
}

extern struct SensorRAW sensorraw;

jintArray Java_com_asus_control_ControlAPI_getSensorRAWData
        (JNIEnv *env, jobject thiz, jint type) {


    struct iRawData data = GetSensorRAWFromJNI(type);

    int size;

    if (type == 0x00) {
        size = sizeof(sensorraw.IRDrop);
    }
    else if (type == 0x01) {
        size = sizeof(sensorraw.Sonar);
    }
    else if (type == 0x02) {
        size = sizeof(sensorraw.DockingIR);
    }
    else if (type == 0x03) {
        size = sizeof(sensorraw.Accelerometer);
    }
    else if (type == 0x04) {
        size = sizeof(sensorraw.Gyroscope);
    }
    else if (type == 0x05) {
        size = sizeof(sensorraw.Ecompass);
    }
    else if (type == 0x06) {
        size = sizeof(sensorraw.MotorTemperature);
    }


    jintArray newIntArray = (env)->NewIntArray(size);
    int *arr = new int[size];

    for (int i = 0; i < size; i++) {
        *(arr + i) = data.data[i];
    }

    for (int i = 0; i < size; i++) {
#ifdef DRIVER_INTERFACE
        LOGD("ASUS_JNI -> arr[%d] = %d",i,*(arr+i));
#endif
    }

    (env)->SetIntArrayRegion(newIntArray, 0, size, arr);
    delete[] arr;


    return newIntArray;
}


// Added by Ting-Ying
jint Java_com_asus_control_ControlAPI_NativeSendCommand
        (JNIEnv *env, jobject thiz, jint mCmdID, jint mCmdSize, jintArray mCmdData) {

    // Get the target pose (xB, yB, thetaB)
    jint *CmdData = (jint *) env->GetIntArrayElements(mCmdData, 0);

    unsigned char data[mCmdSize];
    for (int i = 0; i < mCmdSize; i++) {   // vl and vr
        data[i] = LOBYTE(CmdData[i]);
    }
    pushCommand(mCmdID, mCmdSize, data);
    mspin.InitialThread();
    env->ReleaseIntArrayElements(mCmdData, CmdData, 0);

    return e_rtnSuccess;   //
}


jint Java_com_asus_control_ControlAPI_NativeWheelSpdCtrl
        (JNIEnv *env, jobject thiz, jintArray Spd) {

    // Get the target pose (xB, yB, thetaB)
    jint *WheelSpd = (jint *) env->GetIntArrayElements(Spd, 0);

    unsigned char size = 2 * 2; // 2*integer (vl, vr)
    unsigned char data[size];
    for (int i = 0; i < 2; i++) {   // vl and vr
        data[2 * i    ] = LOBYTE(WheelSpd[i]);
        data[2 * i + 1] = HIBYTE(WheelSpd[i]);
    }
    pushCommand(e_WheelVelCtrl, size, data);
    mspin.InitialThread();
    env->ReleaseIntArrayElements(Spd, WheelSpd, 0);

    return e_rtnSuccess;   //
}


jint Java_com_asus_control_ControlAPI_NativeReloadMapFile
        (JNIEnv *, jobject) {
/*
    // Load the file, SpecificAction.txt, in which the motions for pitch and yaw directions are pre-defined
    int result = loadSpecificAction( _Specific );
    LOGD("SpecificActionTable[0][2] = %d", SpecificActionTable[0][2]);
 */

/*  In case, the map is not generated, and it can be created or push to /sdcard/map.txt manually
    int m = 100;	// vertical size of the map
    int n = 100;	// horizontal size of the map
    writeTextToFile( _FilePath, m, n );
*/

    // Load the map, map.txt, in which the map for the environment is declare
    Table = readIn2dData(_FilePath);

    return e_rtnSuccess;
}


jint Java_com_asus_control_ControlAPI_NativeControlNeckJoint
        (JNIEnv *env, jobject thiz, jintArray target) {
    // Get the target pose ()
    jint *targetPose = (jint *) env->GetIntArrayElements(target, 0);

    unsigned char size = 4 * 2; // targetPose (4*2)
    unsigned char data[size];
    for (int i = 0; i < 4; i++) {
        data[2 * i    ] = LOBYTE(targetPose[i]);
        data[2 * i + 1] = HIBYTE(targetPose[i]);
    }
    pushCommand(e_NeckJointCtrl, size, data);
    mspin.InitialThread();
    env->ReleaseIntArrayElements(target, targetPose, 0);

    return e_rtnSuccess;
}

jint Java_com_asus_control_ControlAPI_NativeCtrlTrkNeckJoint
        (JNIEnv *env, jobject thiz, jintArray target) {
    // Get the target pose ()
    jint *targetPose = (jint *) env->GetIntArrayElements(target, 0);

    unsigned char size = 9; // targetPose (4*2)+1switch
    unsigned char data[size];  // add data size
    for (int i = 0; i < 4; i++) {
        data[2 * i    ] = LOBYTE(targetPose[i]);
        data[2 * i + 1] = HIBYTE(targetPose[i]);
    }
    data[8] = 0x01;//switch constAccProfile(0x00) or linearProfile(0x01)
    pushCommand(e_NeckJointCtrl, size, data);// (cmdID=0x44, cmdSize, cmdData)
    mspin.InitialThread();
    env->ReleaseIntArrayElements(target, targetPose, 0);

    return e_rtnSuccess;
}

jint Java_com_asus_control_ControlAPI_NativePointSmooth
        (JNIEnv *env, jobject thiz, jintArray target) {
    // Get the target pose ()
    jint *targetPose = (jint *) env->GetIntArrayElements(target, 0);

    unsigned char size = 8; // targetPose (4*2)+1switch
    unsigned char data[size];  // add data size
    for (int i = 0; i < 4; i++) {
        data[2 * i    ] = LOBYTE(targetPose[i]);
        data[2 * i + 1] = HIBYTE(targetPose[i]);
    }
    pushCommand(e_SpecificAction, size, data);// (cmdID=0x40, cmdSize, cmdData)
    mspin.InitialThread();
    env->ReleaseIntArrayElements(target, targetPose, 0);

    return e_rtnSuccess;
}

jint Java_com_asus_control_ControlAPI_NativeControlNeckSpecificAction
        (JNIEnv *env, jobject thiz, jint ItemNo) {

    int result = loadSpecificAction(_Specific);
    // yaw, pitch, yawTime, pitchTime to MCU
    int targetPose[4] = {SpecificActionTable[ItemNo][0], SpecificActionTable[ItemNo][1],
                         SpecificActionTable[ItemNo][2], SpecificActionTable[ItemNo][2]};
    LOGD("Specific Action No. %d = (%d, %d, %d)", ItemNo, targetPose[0], targetPose[1], targetPose[2]);

    unsigned char size = 4 * 2; // targetPose (4*2)
    unsigned char data[size];
    for (int i = 0; i < 4; i++) {
        data[2 * i    ] = LOBYTE(targetPose[i]);
        data[2 * i + 1] = HIBYTE(targetPose[i]);
    }
    pushCommand(e_NeckJointCtrl, size, data);
    mspin.InitialThread();

    return e_rtnSuccess;
}


jint Java_com_asus_control_ControlAPI_NativeRelMoveTo
        (JNIEnv *env, jobject thiz, jint mVolume, jint mIndex, jintArray mTarget) {

    unsigned char size = 1 + 3 * 2; // index(1) + targetPose (3*2)
    // Get the target pose (xB, yB, thetaB)
    jint *targetPose = (jint *) env->GetIntArrayElements(mTarget, 0);

    if (mIndex == 0) {
        unsigned char Volume[1] = {mVolume};

        pushCommand(e_BaseTaskSize, 1, Volume);     // int pushCommand(uint8_t cmdID, uint8_t cmdSize, uint8_t data[]);
        mspin.InitialThread();
    }

    unsigned char data[size];
    data[0] = mIndex;   // Index : from 0 to 255
    for (int i = 0; i < 3; i++) {
        data[2 * i + 1] = LOBYTE(targetPose[i]);
        data[2 * i + 2] = HIBYTE(targetPose[i]);
    }
    pushCommand(e_BaseTaskData, size, data);
    mspin.InitialThread();
    env->ReleaseIntArrayElements(mTarget, targetPose, 0);

    return e_rtnSuccess;   //
}


jintArray Java_com_asus_control_ControlAPI_NativeGoFromAToB
        (JNIEnv *env, jobject thiz, jintArray init, jintArray target) {

// Check if the map (map.txt) is loaded or not
//    if (sizeof(Table) == 0) {
    Table = readIn2dData(_FilePath);
//    }

    // Initial pose (xA, yA, thetaA) and target pose (xB, yB, thetaB)
    jint *initPose = (jint *) env->GetIntArrayElements(init, 0);
    jint *targetPose = (jint *) env->GetIntArrayElements(target, 0);


    _ROUTE Route = pathFind(initPose, targetPose);

    /*
    int TempPose[][3] = { {20, 20, 0}, {40, 0, 0}, {20, -20, 0}, {0, 0, 0},     // The grid width is 50 mm, so { {1.0, 1.0, 0}, {2.0, 0.0, 0} ...
                          {40, 40, 0}, {80, 0, 0}, {40, -40, 0}, {0, 0, 0} };
    Route.sizeSeg = 8;
    for (int i = 0 ; i < Route.sizeSeg ; i++) {
        Route.relPathPt[0][i] = TempPose[i][0];
        Route.relPathPt[1][i] = TempPose[i][1];
    }
    */

    jintArray gridPath = (env)->NewIntArray( Route.sizeSeg*2 );
    int *Array = new int[ (Route.sizeSeg*2) ];
    if (Route.sizeSeg != 0) {
        for (int i = 0; i < Route.sizeSeg; i++ ) {
            Array[2*i]   = Route.relPathPt[0][i] ;    // x-axis
            Array[2*i+1] = Route.relPathPt[1][i] ;    // y-axis
        }
        (env)->SetIntArrayRegion(gridPath, 0, Route.sizeSeg*2, Array);
        delete[] Array;

        return gridPath;
    } else
        return NULL;

}

