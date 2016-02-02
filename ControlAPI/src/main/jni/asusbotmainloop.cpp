#include <iostream>
#include <string.h>
#include <unistd.h>
#include <stdio.h> //printf
#include <stdlib.h>


#include "include/asusbot.h"
#include "include/Trajectory.h"
#include "include/command.h"
#include "include/Threads.h"
#include "include/PathFinding.h"
#include "include/Initialization.h"
#include "include/VCP.h"
#include "include/DebugMessage.h"
#include "include/EnumDefine.h"

#include "DriverInterface.h"
#include "asusbotmainloop.h"


#define LOG_TAG "ASUS_asusbotmainloop"

int JNICommandParse[20];

extern bool PortIsOpen;

extern struct SensorStatus  sensorstatus;
extern struct _BASETASK     absBaseTask;
extern struct _AVOIDTARGET  AvoidTarget;
extern struct Encoder encoder;
extern struct SensorRAW sensorraw;
extern struct Version version;
extern struct ControllerInfo controllerinfo;
extern struct SensorInfo sensorinfo;

class AsusbotManager {
public:
    double vx, wz;

    AsusbotManager();

    ~AsusbotManager();

    bool init();

    bool init2(uint8_t cmdID, uint8_t size, uint8_t data[]);

    void spin();

    void setspeed(double vx, double wz);

    void setview(int16_t yaw, int16_t pitch, int16_t yaw_t, int16_t pitch_t);

    void setPWM(int8_t MotorSelect, int16_t PWMValue);

//    void BaseTaskSize(uint8_t size);
    void moveBaseTask(uint8_t index, int16_t x, int16_t y, int16_t theta);

    void setFeedbackSensor(int8_t sensor_flag);

    void GetVersionInformation();

    void SendExtraCommand(int8_t command_item);

    void SetControllerOutputLimit(int8_t control_type, int8_t controller_sn, int16_t upper_value, int16_t lower_value);

    void SetControllerOffset(int8_t control_type, int8_t controller_sn, int16_t Offset_value);

    void SetControllerPIDGain(int8_t control_type, int8_t controller_sn, int16_t PGain, int16_t IGain, int16_t DGain);

    void GetControllerInfomation(int8_t control_type, int8_t controller_sn, int8_t Type);

    void SetSensorOffset(uint8_t sensor_type, uint8_t sensor_sn, int16_t Offset_value);

    void SetSensorEMGValue(uint8_t sensor_type, uint8_t sensor_sn, int16_t EMG_value);

private:

    asusbot::Asusbot ausubot;
};

AsusbotManager::AsusbotManager() {
}

AsusbotManager::~AsusbotManager() {
    LOGD("~AsusbotManager()");
}

bool AsusbotManager::init() {
    vx = 0.0;
    wz = 0.0;

    asusbot::Parameters parameters;

    parameters.device_port = "/dev/ttyACM0";

    LOGI("AsusbotManager::init -> try to open serial port..............");

    PortIsOpen = false;
    ausubot.init(parameters);

    return true;
}


bool AsusbotManager::init2(uint8_t cmdID, uint8_t cmdSize, uint8_t data[]) {

    asusbot::Parameters parameters;

    PortIsOpen = false;
    ausubot.init(parameters);
    ausubot.packageCommand(cmdID, cmdSize, data);

    // ausubot.RCMThread_sleep_ms(parameters.checkTimer);  // Sleep to wait for checking the return status

/*  // if the status of checksum is false, resend the command again.
    if (){
        ausubot.packageCommand(cmdID, cmdSize, data);
    }
*/
    //
    return true;
}

void AsusbotManager::spin() {
    asusbot::Threads threadx;
    threadx.StartReceiverThread();

    ausubot.spin();
    //ausubot.RCMThread_sleep_ms(10);

}

void AsusbotManager::setspeed(double vx, double wz) {

    asusbot::Parameters parameters;
    parameters.device_port = "/dev/ttyACM0";
    LOGI("AsusbotManager::setspeed -> linear velocity = %lf , angular velocity = %lf", vx, wz);

    PortIsOpen = false;
    ausubot.init(parameters);
    ausubot.setBaseControl(vx, wz);

}

void AsusbotManager::setview(int16_t yaw, int16_t pitch, int16_t yaw_t, int16_t pitch_t) {

    asusbot::Parameters parameters;
    parameters.device_port = "/dev/ttyACM0";
    LOGI("AsusbotManager::setview -> yaw = %d , pitch = %d, yaw_t = %d, pitch_t = %d.", yaw, pitch, yaw_t, pitch_t);

    PortIsOpen = false;
    ausubot.init(parameters);
    ausubot.setNeckControl(yaw, pitch, yaw_t, pitch_t);

}

void AsusbotManager::setPWM(int8_t MotorSelect, int16_t PWMValue) {

    asusbot::Parameters parameters;
    parameters.device_port = "/dev/ttyACM0";
    LOGI("AsusbotManager::setPWM -> MotorSel = %d , PWM = %d.", MotorSelect, PWMValue);

    PortIsOpen = false;
    ausubot.init(parameters);
    ausubot.SetPWMControl(MotorSelect, PWMValue);

}

/*
void AsusbotManager::BaseTaskSize(uint8_t size) {

    asusbot::Parameters parameters;
    parameters.device_port = "/dev/ttyACM0";
    LOGI("AsusbotManager::moveBase -> x = %d , y = %d, theta = %d.", x, y, theta);

    PortIsOpen = false;
    ausubot.init(parameters);
    ausubot.BaseTaskSize(index);

}
*/

void AsusbotManager::moveBaseTask(uint8_t index, int16_t x, int16_t y, int16_t theta) {

    asusbot::Parameters parameters;
    parameters.device_port = "/dev/ttyACM0";
    LOGI("AsusbotManager::moveBase -> x = %d , y = %d, theta = %d.", x, y, theta);

    PortIsOpen = false;
    ausubot.init(parameters);
    ausubot.moveBaseTaskControl(index, x, y, theta);

}

void AsusbotManager::setFeedbackSensor(int8_t sensor_flag){
    asusbot::Parameters parameters;
    parameters.device_port = "/dev/ttyACM0";
    LOGI("AsusbotManager::setFeedbackSensor -> Mux = %d.", sensor_flag);

    PortIsOpen = false;
    ausubot.init(parameters);
    ausubot.SetFeedbackInfo(sensor_flag);

}

void AsusbotManager::GetVersionInformation(){
    asusbot::Parameters parameters;
    parameters.device_port = "/dev/ttyACM0";
    LOGI("AsusbotManager::GetVersionInformation.");

    PortIsOpen = false;
    ausubot.init(parameters);
    ausubot.GetVersionInfo();

}

void AsusbotManager::SendExtraCommand(int8_t command_item){
    asusbot::Parameters parameters;
    parameters.device_port = "/dev/ttyACM0";
    LOGI("AsusbotManager::SendExtraCommand -> command_item = %d.", command_item);

    PortIsOpen = false;
    ausubot.init(parameters);
    ausubot.SetExtraCommand(command_item);

}

void AsusbotManager::SetControllerOutputLimit(int8_t control_type, int8_t controller_sn,
                                              int16_t upper_value, int16_t lower_value) {
    asusbot::Parameters parameters;
    parameters.device_port = "/dev/ttyACM0";
    LOGI("AsusbotManager::SetControllerOutputLimit -> type = %d, sn = %d, upper = %d, lower = %d.",
         control_type, controller_sn, upper_value, lower_value);

    PortIsOpen = false;
    ausubot.init(parameters);
    ausubot.SetControllerOutputLimit(control_type, controller_sn, upper_value, lower_value);

}

void AsusbotManager::SetControllerOffset(int8_t control_type, int8_t controller_sn,
                                              int16_t Offset_value) {
    asusbot::Parameters parameters;
    parameters.device_port = "/dev/ttyACM0";
    LOGI("AsusbotManager::SetControllerOffset -> type = %d, sn = %d, Offset = %d.",
         control_type, controller_sn, Offset_value);

    PortIsOpen = false;
    ausubot.init(parameters);
    ausubot.SetControllerOutputOffset(control_type, controller_sn, Offset_value);

}

void AsusbotManager::SetControllerPIDGain(int8_t control_type, int8_t controller_sn,
                                         int16_t PGain, int16_t IGain, int16_t DGain) {
    asusbot::Parameters parameters;
    parameters.device_port = "/dev/ttyACM0";
    LOGI("AsusbotManager::SetControllerGain -> type = %d, sn = %d, P = %d, I = %d, D = %d.",
         control_type, controller_sn, PGain, IGain, DGain);

    PortIsOpen = false;
    ausubot.init(parameters);
    ausubot.SetControllerGain(control_type, controller_sn, PGain, IGain, DGain);

}

void AsusbotManager::GetControllerInfomation(int8_t control_type, int8_t controller_sn,
                                             int8_t Type){
    asusbot::Parameters parameters;
    parameters.device_port = "/dev/ttyACM0";
    LOGI("AsusbotManager::GetControllerGain -> Controller type = %d, Controller = %d, info type = %d.",
         control_type, controller_sn, Type);

    PortIsOpen = false;
    ausubot.init(parameters);
    ausubot.GetControllerInfo(control_type, controller_sn, Type);

}

void AsusbotManager::SetSensorOffset(uint8_t sensor_type, uint8_t sensor_sn,
                                         int16_t Offset_value){
    asusbot::Parameters parameters;
    parameters.device_port = "/dev/ttyACM0";
    LOGI("AsusbotManager::SetSensorOffset -> type = %d, sn = %d, Offset = %d.",
         sensor_type, sensor_sn, Offset_value);

    PortIsOpen = false;
    ausubot.init(parameters);
    ausubot.SetSensorOffset(sensor_type, sensor_sn, Offset_value);

}

void AsusbotManager::SetSensorEMGValue(uint8_t sensor_type, uint8_t sensor_sn,
                                     int16_t EMG_value){
    asusbot::Parameters parameters;
    parameters.device_port = "/dev/ttyACM0";
    LOGI("AsusbotManager::SetSensorEMGValue -> type = %d, sn = %d, EMGValue = %d.",
         sensor_type, sensor_sn, EMG_value);

    PortIsOpen = false;
    ausubot.init(parameters);
    ausubot.SetSensorOffset(sensor_type, sensor_sn, EMG_value);

}



void *asusbotloop(void *argv) {
    AsusbotManager asusbot_manager;
    asusbot_manager.spin();

    LOGD("....new a threads");

    return 0;
}

#ifdef DEBUG_FAKETIMESTAMP
extern int fakeTimeStamp;
#endif

struct iparameter GetSensorStatusFromJNI(int para) {

    struct iparameter tempparameter;

    tempparameter.data[0] = sensorstatus.timestamp;
#ifdef DEBUG_FAKETIMESTAMP
    tempparameter.data[0] = fakeTimeStamp;
#endif
    tempparameter.data[1] = sensorstatus.IRDrop;
    tempparameter.data[2] = sensorstatus.Sonar;
    tempparameter.data[3] = sensorstatus.OverCurrent;
    tempparameter.data[4] = sensorstatus.OverTemperature;
    tempparameter.data[5] = sensorstatus.Neck;
    tempparameter.data[6] = sensorstatus.Mobile;
    tempparameter.data[7] = sensorstatus.ChargerandBattery;

    tempparameter.data[8] = encoder.wheel_encoder[0];
    tempparameter.data[9] = encoder.wheel_encoder[1];

    tempparameter.data[10] = encoder.neck[0];
    tempparameter.data[11] = encoder.neck[1];

    tempparameter.data[12] = absBaseTask.x;
    tempparameter.data[13] = absBaseTask.y;
    tempparameter.data[14] = absBaseTask.theta;

    tempparameter.data[15] = AvoidTarget.x;
    tempparameter.data[16] = AvoidTarget.y;

    return tempparameter;
}

struct iRawData GetSensorRAWFromJNI(int type) {

    struct iRawData rawdata;

    if(type == 0x00){
        rawdata.data[0] = sensorraw.IRDrop[0];
        rawdata.data[1] = sensorraw.IRDrop[1];
        rawdata.data[2] = sensorraw.IRDrop[2];
    }
    else if(type == 0x01){
        rawdata.data[0] = sensorraw.Sonar[0];
        rawdata.data[1] = sensorraw.Sonar[1];
        rawdata.data[2] = sensorraw.Sonar[2];
    }
    else if(type == 0x02){
        rawdata.data[0] = sensorraw.DockingIR[0];
        rawdata.data[1] = sensorraw.DockingIR[1];
    }
    else if(type == 0x03){
        rawdata.data[0] = sensorraw.Accelerometer[0];
        rawdata.data[1] = sensorraw.Accelerometer[1];
        rawdata.data[2] = sensorraw.Accelerometer[2];
    }
    else if(type == 0x04){
        rawdata.data[0] = sensorraw.Gyroscope[0];
        rawdata.data[1] = sensorraw.Gyroscope[1];
        rawdata.data[2] = sensorraw.Gyroscope[2];
    }
    else if(type == 0x05){
        rawdata.data[0] = sensorraw.Ecompass[0];
        rawdata.data[1] = sensorraw.Ecompass[1];
        rawdata.data[2] = sensorraw.Ecompass[2];
    }
    else if(type == 0x06){

        rawdata.data[0] = sensorraw.MotorTemperature[0];
        rawdata.data[1] = sensorraw.MotorTemperature[1];
        rawdata.data[2] = sensorraw.MotorTemperature[2];
        rawdata.data[3] = sensorraw.MotorTemperature[3];
    }

    return rawdata;
}

int asusbotMain(int *param, int argc) {
    LOGD("Finish init and enter asusbotMain ");
    AsusbotManager asusbot_manager;

    if (argc > 0) {
        LOGD("Commmand from android is .... ");
        int i;
        for (i = 0; i < argc; i++) {
            JNICommandParse[i] = param[i];
            LOGD("JNICommandParse[%d] = %d", i, JNICommandParse[i]);

        }
    }

    if ((JNICommandParse[0] == 0) && (JNICommandParse[1] == 0) && (JNICommandParse[2] == 0)) {
        if (asusbot_manager.init()) {
            LOGD("asusbotmainloop -> initial");
        }
    }

    if ((JNICommandParse[0] == 1)) {
        LOGD("Command to set base speed control....");
        asusbot_manager.setspeed((double) JNICommandParse[1] / 100.0,
                                 (double) JNICommandParse[2] / 100.0); //mm/s rad/s
    }

    // 0x04 : Neck Position Control
    if ((JNICommandParse[0] == 0x04)) {
        // Check the upper and lower bound (90 deg needed to be changed!)
        if (JNICommandParse[1] > 900)   // yaw
            JNICommandParse[1] = 900;
        else if (JNICommandParse[1] < -900)
            JNICommandParse[1] = -900;

        if (JNICommandParse[2] > 450)   // pitch
            JNICommandParse[2] = 450;
        else if (JNICommandParse[2] < -300)
            JNICommandParse[2] = -300;

        LOGD("Command to set neck...."); //test
        asusbot_manager.setview(JNICommandParse[1], JNICommandParse[2],  JNICommandParse[3],  JNICommandParse[4]); //mm/s rad/s
    }

    if ((JNICommandParse[0] == 0x05)) {
        LOGD("Command to set PWM....");
        asusbot_manager.setPWM(JNICommandParse[1], JNICommandParse[2]);
    }

/*
    // 0x06 : e_BaseTaskSize
    if ((JNICommandParse[0] == e_BaseTaskSize)) {

        LOGD("Command to set Base Task Control....");
        asusbot_manager.BaseTaskSize(JNICommandParse[1]); //
    }
*/
    // 0x07 : e_BaseTaskData
    if ((JNICommandParse[0] == 0x07)) {
        // (x, y, theta) = (JNICommandParse[1], JNICommandParse[2],  JNICommandParse[3])

        LOGD("Command to set Base Task Control....");
        asusbot_manager.moveBaseTask(JNICommandParse[1], JNICommandParse[2], JNICommandParse[3], JNICommandParse[4]); //mm/s rad/s
    }

    if ((JNICommandParse[0] == 0x08)) {
        LOGD("Command to Get feedback Info....");
        asusbot_manager.setFeedbackSensor(JNICommandParse[1]);
    }

    if ((JNICommandParse[0] == 0x09)) {
        LOGD("Command to Get Version Info....");
        asusbot_manager.GetVersionInformation();
    }

    if ((JNICommandParse[0] == 0x0A)) {
        LOGD("Command to extra command....");
        asusbot_manager.SendExtraCommand(JNICommandParse[1]);
    }

    if ((JNICommandParse[0] == 0x11)) {
        LOGD("Set controller limit");
        asusbot_manager.SetControllerOutputLimit(JNICommandParse[1], JNICommandParse[2], JNICommandParse[3], JNICommandParse[4]);
    }

    if ((JNICommandParse[0] == 0x12)) {
        LOGD("Set controller Offset");
        asusbot_manager.SetControllerOffset(JNICommandParse[1], JNICommandParse[2], JNICommandParse[3]);
    }


    if ((JNICommandParse[0] == 0x13)) {
        LOGD("Set controller PID Gain");
        asusbot_manager.SetControllerPIDGain(JNICommandParse[1], JNICommandParse[2], JNICommandParse[3], JNICommandParse[4], JNICommandParse[5]);
    }

    if ((JNICommandParse[0] == 0x14)) {
        LOGD("Get controller info");
        asusbot_manager.GetControllerInfomation(JNICommandParse[1], JNICommandParse[2], JNICommandParse[3]);
    }

    if ((JNICommandParse[0] == 0x30)) {
        LOGD("Set Sensor offset");
        asusbot_manager.SetSensorOffset(JNICommandParse[1], JNICommandParse[2], JNICommandParse[3]);
    }

    if ((JNICommandParse[0] == 0x31)) {
        LOGD("Set Sensor EMG Value");
        asusbot_manager.SetSensorEMGValue(JNICommandParse[1], JNICommandParse[2], JNICommandParse[3]);
    }


    return 0;
}

int pushCommand(uint8_t cmdID, uint8_t cmdSize, uint8_t data[]) {

    AsusbotManager asusbot_manager;
    asusbot_manager.init2(cmdID, cmdSize, data); //mm/s rad/s

    return 0;
}

ASUSBOTSPIN::ASUSBOTSPIN(void) {
    LOGI("ASUSBOTSPIN create");
}

ASUSBOTSPIN::~ASUSBOTSPIN() {

    LOGI("ASUSBOTSPIN kill thread");
    pthread_kill(asusbot_driver_id, SIGKILL);
}

void ASUSBOTSPIN::InitialThread() {
    LOGI("ASUSBOTSPIN InitialThread");
    pthread_create(&asusbot_driver_id, NULL, asusbotloop, (void *) this);
}

