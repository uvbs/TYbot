#include <stdio.h> /* printf */
#include <pthread.h>
#include <signal.h>
#include <iostream>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

#include "../include/Trajectory.h"
#include "../include/asusbot.h"
#include "../include/DebugMessage.h"
#include "../include/EnumDefine.h"

#define LOG_TAG "asusbot"


extern int JNICommandParse[20];
extern bool PortIsOpen;


extern struct SensorStatus sensorstatus;
extern struct Encoder encoder;
extern struct _BASETASK     absBaseTask;
extern struct _AVOIDTARGET  AvoidTarget;
extern struct SensorRAW sensorraw;
extern struct Version version;
extern struct ControllerInfo controllerinfo;
extern struct SensorInfo sensorinfo;

extern double DiffDrive_radius, DiffDrive_speed; // in [mm] ,[mm/s]

bool is_enabled;

namespace asusbot {
    unsigned int last_timestamp = 0, last_tick_right = 0, last_tick_left = 0;
    double last_diff_time = 0.0, last_diff_tick_left = 0.0, last_diff_tick_right = 0.0;
    bool initial_timestamp = false;

    Asusbot::Asusbot() {
    }

    Asusbot::~Asusbot() {
//		asusbot::Thread thread; //0917 test
//		LOGI("kill Asusbot");
//		thread.join(); //0917 test
//
//		pthread_kill(Tx_thread, SIGKILL); //(void *) &PkgCmd);
//		pthread_kill(Rx_thread, SIGKILL); //(void *) &PkgCmd);
//		disable();
    }

    void Asusbot::RCMThread_sleep_us(int ut) {
        timespec ts;
        ts.tv_sec = 0;
        ts.tv_nsec = 1000 * ut;
        nanosleep(&ts, NULL);
    }

    void Asusbot::RCMThread_sleep_ms(int mt) {
        timespec mts;
        mts.tv_sec = 0;
        mts.tv_nsec = 1000000 * mt;
        nanosleep(&mts, NULL);
    }

    void Asusbot::init(Parameters &paremeters) {

        if (PortIsOpen == false) {

            LOGD("port is not open, init process......");

            this->vcp.Initial(paremeters.device_port);
        }
#ifdef ASUSBOT_COMMAND
        LOGI("sendCommand : GetVersionInfo");
#endif
        sendCommand(Command::GetVersionInfo());
    }

    unsigned char Asusbot::CheckSum(unsigned char checksum, unsigned char value) {
        checksum ^= value;
        return checksum;
    }

    void Asusbot::spin() {
        Mutex update_mutex;
        Parameters parameters;
        Asusbot asusbot;
        unsigned char Buffer[256];

        txrxQueue rxPackageQueue;
        rxPackageQueue.Queue_Init();

#ifdef ASUSBOT_COMMAND
        LOGI("Entry ReceiverThread");
#endif

        vcp.Initial(parameters.device_port);

        while (true) {

            update_mutex.lock();
            int bytesReceive = vcp.ReadPort(Buffer, 256); //Read data from kernel buffer
            if (bytesReceive <= 0) {

            }
            else {
                //It will enque data to C-queue, so must be sure the Queue is not full.
                if (!rxPackageQueue.Queue_Is_Full()) {
#ifdef ASUSBOT_COMMAND
                    //LOGD("receive %2d data to rear of queue = ", bytesReceive);
#endif

                    for (int i = 0; i < bytesReceive; i++) {
                        if (!rxPackageQueue.Queue_Is_Full()) {
                            rxPackageQueue.push(Buffer[i]); // Receive rs232 data from Queue
#ifdef ASUSBOT_COMMAND
                            //LOGD(" %2x ", Buffer[i]);
#endif
                        }
                        else {

                            LOGE("Error !! rxPackageQueue.full()!!!!!!!! ");

                            rxPackageQueue.Queue_Clear(); //GauJei for temp because have no parse yet.
                        }
                    }
                }
                else {

                    LOGE("Error !! rxPackageQueue.full()!!!!!!!! \n ");

                    rxPackageQueue.Queue_Clear(); //GauJei for temp because have no parse yet.
                }
            }

            unsigned char CS = 0x00, Length = 0x00, CheckTY = 0x00;
            unsigned char payload[256], sub_payload[256][256];
            unsigned char Sub_Payload_Header = 0x00, sub_payload_id = 0x00, sub_payload_length = 0x00;

            while (!rxPackageQueue.Queue_Is_Empty()) {
                if ((rxPackageQueue.pop() == 0xaa) && (!rxPackageQueue.Queue_Is_Empty())) {
                    if ((rxPackageQueue.pop() == 0x55) && (!rxPackageQueue.Queue_Is_Empty())) {
                        Length = rxPackageQueue.pop();
                        CS = CheckSum(CS, Length);

                        for (int i = 0; i < Length; i++) {
                            if (!rxPackageQueue.Queue_Is_Empty()) {
                                payload[i] = rxPackageQueue.pop();
                                CS = CheckSum(CS, payload[i]);
                            }
                        }

                        if (!rxPackageQueue.Queue_Is_Empty()) {
                            unsigned char rxChecksum = rxPackageQueue.pop();
                            CheckTY ^= Length;
                            for (int i = 0; i < Length; i++)
                                CheckTY ^= payload[i];


                            if (CheckTY == rxChecksum) {
//								LOGD("CheckSum is correct =  %2x.",CS);
                            }
                            else {
//                                LOGE("Error : Checksum is incorrcet !! \n");
//                                LOGD("Head---------------------------");
//                                LOGD("length =  %2x (Hex)", Length);
//                                for (int i = 0; i < Length; i++) {
//                                    LOGD("payload[%2d] =  %2x.", i, payload[i]);
//                                }
//                                LOGD("CS: %2x ", CS);
//                                LOGD("rxPackageQueue : %2x", rxChecksum);
//                                LOGD("CheckTY : %2x ", CheckTY);
//                                LOGD("End---------------------------");
//                                continue;
                            }
                        }
                    }
                }


                txrxQueue SubPayloadQ;
                SubPayloadQ.Queue_Init();

                for (int j = 0; j < Length; j++) {
                    SubPayloadQ.push(payload[j]);
                }

                while (!SubPayloadQ.Queue_Is_Empty()) {
                    sub_payload_id = SubPayloadQ.pop();

                    if (!SubPayloadQ.Queue_Is_Empty()) {
                        sub_payload_length = SubPayloadQ.pop();
                        //sub_payload[Sub_Payload_Header] = new int[sub_payload_length+2];
                    }

                    sub_payload[Sub_Payload_Header][0] = sub_payload_id;
                    sub_payload[Sub_Payload_Header][1] = sub_payload_length;

                    for (int k = 2; k < sub_payload_length + 2; k++) {
                        if (!SubPayloadQ.Queue_Is_Empty()) {
                            sub_payload[Sub_Payload_Header][k] = SubPayloadQ.pop();
                        }
                    }

                    switch (sub_payload[Sub_Payload_Header][0]) {
                        case 0x01: //core sensor
                            sensorstatus.timestamp = sub_payload[Sub_Payload_Header][5] << 24 | sub_payload[Sub_Payload_Header][4] << 16 |
                                                     sub_payload[Sub_Payload_Header][3] << 8 | sub_payload[Sub_Payload_Header][2];

                            sensorstatus.IRDrop = sub_payload[Sub_Payload_Header][6];
                            sensorstatus.Sonar = sub_payload[Sub_Payload_Header][7];
                            sensorstatus.OverCurrent = sub_payload[Sub_Payload_Header][8];
                            sensorstatus.OverTemperature = sub_payload[Sub_Payload_Header][9];
                            sensorstatus.Neck = sub_payload[Sub_Payload_Header][10];
                            sensorstatus.Mobile = sub_payload[Sub_Payload_Header][11];
                            sensorstatus.ChargerandBattery = sub_payload[Sub_Payload_Header][12];

                            encoder.wheel_encoder[0] = sub_payload[Sub_Payload_Header][14] * 256 +
                                                sub_payload[Sub_Payload_Header][13];
                            encoder.wheel_encoder[1] = sub_payload[Sub_Payload_Header][16] * 256 +
                                                sub_payload[Sub_Payload_Header][15];
                            encoder.neck[0] = sub_payload[Sub_Payload_Header][18] * 256 +
                                                       sub_payload[Sub_Payload_Header][17];
                            encoder.neck[1] = sub_payload[Sub_Payload_Header][20] * 256 +
                                                       sub_payload[Sub_Payload_Header][19];

                            absBaseTask.x = sub_payload[Sub_Payload_Header][24] << 24 | sub_payload[Sub_Payload_Header][23] << 16 |
                                            sub_payload[Sub_Payload_Header][22] << 8 | sub_payload[Sub_Payload_Header][21];
                            absBaseTask.y = sub_payload[Sub_Payload_Header][28] << 24 | sub_payload[Sub_Payload_Header][27] << 16 |
                                            sub_payload[Sub_Payload_Header][26] << 8 | sub_payload[Sub_Payload_Header][25];
                            absBaseTask.theta = (sub_payload[Sub_Payload_Header][32] << 24 | sub_payload[Sub_Payload_Header][31] << 16 |
                                                 sub_payload[Sub_Payload_Header][30] << 8 | sub_payload[Sub_Payload_Header][29]);

                            // Locomotion.AvoidTarget
                            AvoidTarget.x = sub_payload[Sub_Payload_Header][36] << 24 | sub_payload[Sub_Payload_Header][35] << 16 |
                                            sub_payload[Sub_Payload_Header][34] << 8 | sub_payload[Sub_Payload_Header][33];
                            AvoidTarget.y = sub_payload[Sub_Payload_Header][40] << 24 | sub_payload[Sub_Payload_Header][39] << 16 |
                                            sub_payload[Sub_Payload_Header][38] << 8 | sub_payload[Sub_Payload_Header][37];


                            if (sensorstatus.timestamp != last_timestamp) {
                                last_diff_time = (double) ((sensorstatus.timestamp - last_timestamp) & 0xffff) / 1000.0;

                                last_diff_tick_left = (double) ((encoder.wheel_encoder[0] - last_tick_left));

                                last_diff_tick_right = (double) ((encoder.wheel_encoder[1] - last_tick_right));

                                if (initial_timestamp) {
                                    encoder.velocity[0] = (int) (parameters.wheel_radius *
                                                                (parameters.tick_to_rad *
                                                                 last_diff_tick_left) /
                                                                last_diff_time);
                                    encoder.velocity[1] = (int) (parameters.wheel_radius *
                                                                (parameters.tick_to_rad *
                                                                 last_diff_tick_right) /
                                                                last_diff_time);
                                }

                                last_timestamp = sensorstatus.timestamp;
                                last_tick_left = encoder.wheel_encoder[0];
                                last_tick_right = encoder.wheel_encoder[1];
                                initial_timestamp = true;
                            }

                            break;

                        case 0x02: //Drop IR
                            sensorraw.IRDrop[0] = sub_payload[Sub_Payload_Header][2];
                            sensorraw.IRDrop[1] = sub_payload[Sub_Payload_Header][3];
                            sensorraw.IRDrop[2] = sub_payload[Sub_Payload_Header][4];
                            break;

                        case 0x03: //Sonar
                            sensorraw.Sonar[0] = sub_payload[Sub_Payload_Header][3] * 256 +
                                                 sub_payload[Sub_Payload_Header][2];
                            sensorraw.Sonar[1] = sub_payload[Sub_Payload_Header][5] * 256 +
                                                 sub_payload[Sub_Payload_Header][4];
                            sensorraw.Sonar[2] = sub_payload[Sub_Payload_Header][7] * 256 +
                                                 sub_payload[Sub_Payload_Header][6];
                            break;

                        case 0x04: //Docking IR
                            sensorraw.DockingIR[0] = sub_payload[Sub_Payload_Header][3] * 256 +
                                                 sub_payload[Sub_Payload_Header][2];
                            sensorraw.DockingIR[1] = sub_payload[Sub_Payload_Header][5] * 256 +
                                                 sub_payload[Sub_Payload_Header][4];
                            sensorraw.DockingIR[2] = sub_payload[Sub_Payload_Header][7] * 256 +
                                                 sub_payload[Sub_Payload_Header][6];
                            break;

                        case 0x05: //Accelerometer
                            sensorraw.Accelerometer[0] = sub_payload[Sub_Payload_Header][3] * 256 +
                                                     sub_payload[Sub_Payload_Header][2];
                            sensorraw.Accelerometer[1] = sub_payload[Sub_Payload_Header][5] * 256 +
                                                     sub_payload[Sub_Payload_Header][4];
                            sensorraw.Accelerometer[2] = sub_payload[Sub_Payload_Header][7] * 256 +
                                                     sub_payload[Sub_Payload_Header][6];
                            break;

                        case 0x06: //Gyroscope
                            sensorraw.Gyroscope[0] = sub_payload[Sub_Payload_Header][3] * 256 +
                                                     sub_payload[Sub_Payload_Header][2];
                            sensorraw.Gyroscope[1] = sub_payload[Sub_Payload_Header][5] * 256 +
                                                     sub_payload[Sub_Payload_Header][4];
                            sensorraw.Gyroscope[2] = sub_payload[Sub_Payload_Header][7] * 256 +
                                                     sub_payload[Sub_Payload_Header][6];
                            break;

                        case 0x07: //E-compass
                            sensorraw.Ecompass[0] = sub_payload[Sub_Payload_Header][3] * 256 +
                                                     sub_payload[Sub_Payload_Header][2];
                            sensorraw.Ecompass[1] = sub_payload[Sub_Payload_Header][5] * 256 +
                                                     sub_payload[Sub_Payload_Header][4];
                            sensorraw.Ecompass[2] = sub_payload[Sub_Payload_Header][7] * 256 +
                                                     sub_payload[Sub_Payload_Header][6];
                            break;

                        case 0x09: //Motor temperature
                            sensorraw.MotorTemperature[0] = sub_payload[Sub_Payload_Header][2];
                            sensorraw.MotorTemperature[1] = sub_payload[Sub_Payload_Header][3];
                            sensorraw.MotorTemperature[2] = sub_payload[Sub_Payload_Header][4];
                            sensorraw.MotorTemperature[3] = sub_payload[Sub_Payload_Header][5];
                            break;

                        case 0x0A: //Motor Life
                            sensorraw.MotorLife[0] = sub_payload[Sub_Payload_Header][3] << 8 |
                                                     sub_payload[Sub_Payload_Header][2];
                            sensorraw.MotorLife[1] = sub_payload[Sub_Payload_Header][5] << 8 |
                                                     sub_payload[Sub_Payload_Header][4];
                            sensorraw.MotorLife[2] = sub_payload[Sub_Payload_Header][7] << 8 |
                                                     sub_payload[Sub_Payload_Header][6];
                            sensorraw.MotorLife[3] = sub_payload[Sub_Payload_Header][9] << 8 |
                                                     sub_payload[Sub_Payload_Header][8];
                            sensorraw.MotorLife[4] = sub_payload[Sub_Payload_Header][11] << 8 |
                                                     sub_payload[Sub_Payload_Header][10];
                            sensorraw.MotorLife[5] = sub_payload[Sub_Payload_Header][13] << 8 |
                                                     sub_payload[Sub_Payload_Header][12];
                            sensorraw.MotorLife[6] = sub_payload[Sub_Payload_Header][15] << 8 |
                                                     sub_payload[Sub_Payload_Header][14];
                            sensorraw.MotorLife[7] = sub_payload[Sub_Payload_Header][17] << 8 |
                                                     sub_payload[Sub_Payload_Header][16];
                            break;

                        case 0x10: //Hardware version
                            version.Hardware[0] = sub_payload[Sub_Payload_Header][2];
                            version.Hardware[1] = sub_payload[Sub_Payload_Header][3];
                            version.Hardware[2] = sub_payload[Sub_Payload_Header][4];
                            version.Hardware[3] = sub_payload[Sub_Payload_Header][5];
                            break;

                        case 0x11: //Firmware version
                            version.Firmware[0] = sub_payload[Sub_Payload_Header][2];
                            version.Firmware[1] = sub_payload[Sub_Payload_Header][3];
                            version.Firmware[2] = sub_payload[Sub_Payload_Header][4];
                            version.Firmware[3] = sub_payload[Sub_Payload_Header][5];
                            break;

                        case 0x13: //Controller limit
                            controllerinfo.OutputLimit[0] = sub_payload[Sub_Payload_Header][3] << 8 |
                                                            sub_payload[Sub_Payload_Header][2];
                            controllerinfo.OutputLimit[1] = sub_payload[Sub_Payload_Header][5] << 8 |
                                                            sub_payload[Sub_Payload_Header][4];

                            if ((controllerinfo.OutputLimit[0] & 0x8000) == 0x8000)
                            {
                                controllerinfo.OutputLimit[0] = -32768 + (controllerinfo.OutputLimit[0] & 0x7FFF);
                            }
                            if ((controllerinfo.OutputLimit[1] & 0x8000) == 0x8000)
                            {
                                controllerinfo.OutputLimit[1] = -32768 + (controllerinfo.OutputLimit[1] & 0x7FFF);
                            }
                            break;

                        case 0x14: //Controller offset
                            controllerinfo.Offset = sub_payload[Sub_Payload_Header][3] << 8 |
                                                        sub_payload[Sub_Payload_Header][2];

                            if ((controllerinfo.Offset & 0x8000) == 0x8000)
                            {
                                controllerinfo.Offset = -32768 + (controllerinfo.Offset & 0x7FFF);
                            }
                            break;

                        case 0x15: //Controller gain
                            controllerinfo.Gain[0] = sub_payload[Sub_Payload_Header][3] << 8 |
                                                     sub_payload[Sub_Payload_Header][2];
                            controllerinfo.Gain[1] = sub_payload[Sub_Payload_Header][5] << 8 |
                                                     sub_payload[Sub_Payload_Header][4];
                            controllerinfo.Gain[2] = sub_payload[Sub_Payload_Header][7] << 8 |
                                                     sub_payload[Sub_Payload_Header][6];
                            break;

                        case 0x16: //SensorInfo
                            sensorinfo.Offset = sub_payload[Sub_Payload_Header][3] << 8 |
                                                sub_payload[Sub_Payload_Header][2];

                            if ((sensorinfo.Offset & 0x8000) == 0x8000)
                            {
                                sensorinfo.Offset = -32768 + (sensorinfo.Offset & 0x7FFF);
                            }
                            break;

                        case 0x17: //Sensor EMG value
                            sensorinfo.EMGValue = sub_payload[Sub_Payload_Header][3] << 8 |
                                                sub_payload[Sub_Payload_Header][2];

                            if ((sensorinfo.EMGValue & 0x8000) == 0x8000)
                            {
                                sensorinfo.EMGValue = -32768 + (sensorinfo.EMGValue & 0x7FFF);
                            }
                            break;


                        default:
                            break;
                    }
                }
            }

            update_mutex.unlock();

            Asusbot ausubot;
            ausubot.RCMThread_sleep_ms(1);


//			LOGD("Time stamp = %d. ", sensor.timestamp);
//			LOGD("Sensor status : bumper : %d, WheelDrop = %d, Cliff = %d. ", sensor.status_Bumper, sensor.status_WheelDrop, sensor.status_Cliff);
//			LOGD("L_Encoder = %d , R_Encoder = %d ,L_Velocity = %d , R_Velocity = %d ", sensor.encoder[0], sensor.encoder[1], sensor.velocity[0], sensor.velocity[1]);
//			LOGD("Sensor RAW Data -> IR01 : %d , IR02 = %d , IR03 = %d ,IR04 : %d, IR05 = %d, IR06 = %d. ", sensor.SensorData_IR[0], sensor.SensorData_IR[1], sensor.SensorData_IR[2], sensor.SensorData_IR[3], sensor.SensorData_IR[4], sensor.SensorData_IR[5]);
//			LOGD("Sensor RAW Data -> ULsonic01 : %d , ULsonic02 = %d , ULsonic03 = %d ,ULsonic04 : %d, ULsonic05 = %d. ", sensor.SensorData_ULsonic[0], sensor.SensorData_ULsonic[1], sensor.SensorData_ULsonic[2], sensor.SensorData_ULsonic[3], sensor.SensorData_ULsonic[4]);
//			LOGD("Controller -> No : %u , PGain = %lu , IGain = %lu ,DGain : %lu. ", controller.ControllerNo, controller.Controller_PGain, controller.Controller_IGain, controller.Controller_DGain);
//			return 0;
        }
    }

/*****************************************************************************
** Commands
*****************************************************************************/

    //ID:0x01 BaseVelocityControl
    void Asusbot::setBaseControl(const double &linear_velocity, const double &angular_velocity){

        diff_drive.velocityCommands(linear_velocity, angular_velocity); // V ,  ω   --> V = R x  ω

        #ifdef ASUSBOT_COMMAND
            LOGD("sendCommand : SetVelocityControl speed = %lf, radius = %lf", DiffDrive_speed, DiffDrive_radius);
        #endif

        LOGD("asusbot.cpp : DiffDrive_speed = %f, DiffDrive_radius = %f", DiffDrive_speed, DiffDrive_radius);
        sendCommand(Command::SetVelocityControl(DiffDrive_speed, DiffDrive_radius));
    }

    //ID:0x04 NeckPositionControl
    void Asusbot::setNeckControl(const int16_t &yaw_position, const int16_t &pitch_position, const int16_t &yaw_time, const int16_t &pitch_time){

        #ifdef ASUSBOT_COMMAND
            LOGD("sendCommand : setNeckPositionControl yaw = %d, pitch = %d, yaw_t = %d, pitch_t = %d", yaw_position, pitch_position, yaw_time, pitch_time);
        #endif

        sendCommand(Command::SetNeckPositionControl(yaw_position, pitch_position, yaw_time, pitch_time));
    }

    //ID:0x05 MotorDrivePWMOutput
    void Asusbot::SetPWMControl(const int8_t &PWMControllerSelect, const int16_t &PWMOutputValue){

        #ifdef ASUSBOT_COMMAND
            LOGD("sendCommand : SetPWMControl ControllerSel = %d, PWM Value = %d.", PWMControllerSelect, PWMOutputValue);
        #endif

        sendCommand(Command::SetMotorDrivePWMOutput(PWMControllerSelect, PWMOutputValue));
    }

/*
    //ID:0x06 BaseTaskControl
    void Asusbot::BaseTaskSize(const uint8_t &size){

        #ifdef ASUSBOT_COMMAND
            LOGD("sendCommand : BaseTaskSize = %d", size);
        #endif

        sendCommand(Command::BaseTaskSize(size));
    }
*/
    //ID:0x07 BaseTaskControl
    void Asusbot::moveBaseTaskControl(const uint8_t &index, const int16_t &x, const int16_t &y, const int16_t &theta){

        #ifdef ASUSBOT_COMMAND
            LOGD("sendCommand : moveBaseTaskControl x = %d, y = %d, theta = %d", x, y, theta);
        #endif
        LOGD("vcpdebug asusbot.cpp : %d, %d, %d", index, LOBYTE(x), HIBYTE(x));
        sendCommand(Command::MoveBaseTaskControl(index, x, y, theta));
    }

    //ID:0x08 RequestFeedbackInfo
    void Asusbot::SetFeedbackInfo(const int8_t &RequestFeedback){

    #ifdef ASUSBOT_COMMAND
        LOGD("sendCommand : SetFeedbackInfo = %d.", RequestFeedback);
    #endif

        sendCommand(Command::SetFeedbackInfo(RequestFeedback));
    }

    //ID:0x09 RequestExtra
    void Asusbot::GetVersionInfo(){

        #ifdef ASUSBOT_COMMAND
            LOGD("sendCommand : GetVersionInfo.");
        #endif

        sendCommand(Command::GetVersionInfo());
    }

    //ID:0x0A SetCommandExtra
    void Asusbot::SetExtraCommand(const int8_t &ExtraCommand){

        #ifdef ASUSBOT_COMMAND
            LOGD("ExtraCommand = %d.", ExtraCommand);
        #endif
        sendCommand(Command::ExtraCommand(ExtraCommand));
    }

    //ID:0x10 MotorControlSwitch
    void Asusbot::SetControllerSwitch(const int8_t &CtrlSwitch){

        #ifdef ASUSBOT_COMMAND
            LOGD("Controller Switch = %d.", CtrlSwitch);
        #endif
        sendCommand(Command::ControllerSwitch(CtrlSwitch));
    }

    //ID:0x11 ControllerOutputLimit
    void Asusbot::SetControllerOutputLimit(const int8_t &CtrlType, const int8_t &CtrlSN,
                                           const int16_t &UpperLimit, const int16_t &LowerLimit){

        #ifdef ASUSBOT_COMMAND
            LOGD("SetController Limit, type = %d, SN= %d, Upper = %d, Lower = %d.", CtrlType, CtrlSN, UpperLimit, LowerLimit);
        #endif
        sendCommand(Command::ControllerOutputLimitValue(CtrlType, CtrlSN, UpperLimit, LowerLimit));
    }

    //ID:0x12 ControllerOffset
    void Asusbot::SetControllerOutputOffset(const int8_t &CtrlType, const int8_t &CtrlSN,
                                            const int16_t &Offset){

        #ifdef ASUSBOT_COMMAND
            LOGD("SetController Offset, type = %d, SN= %d, offset = %d.", CtrlType, CtrlSN, Offset);
        #endif
        sendCommand(Command::ControllerOffsetValue(CtrlType, CtrlSN, Offset));
    }

    //ID:0x13 ControllerGain
    void Asusbot::SetControllerGain(const int8_t &CtrlType, const int8_t &CtrlSN,
                                    const int16_t &p_gain,
                                    const int16_t &i_gain,
                                    const int16_t &d_gain){

        #ifdef ASUSBOT_COMMAND
            LOGD("SetController Gain, type = %d, SN= %d, P = %d, I = %d, D = %d.", CtrlType, CtrlSN, p_gain, i_gain, d_gain);
        #endif
        sendCommand(Command::ControllerGainValue(CtrlType, CtrlSN, p_gain, i_gain, d_gain));
    }

    //ID:0x14 ControllerInfo
    void Asusbot::GetControllerInfo(const int8_t &CtrlType, const int8_t &CtrlSN,
                                    const int8_t &info_type){

        #ifdef ASUSBOT_COMMAND
            LOGD("SetController Gain, type = %d, SN= %d, info tpye = %d.", CtrlType, CtrlSN, info_type);
        #endif
        sendCommand(Command::ControllerInformation(CtrlType, CtrlSN, info_type));
    }

    //ID:0x30 SensorCalOffset
    void Asusbot::SetSensorOffset(const uint8_t &SensorType, const uint8_t &SensorSN,
                                  const int16_t &Offset){

        #ifdef ASUSBOT_COMMAND
            LOGD("Set Sensor Offset, type = %d, SN= %d, Offset = %d.", SensorType, SensorSN, Offset);
        #endif
        sendCommand(Command::Sensoroffset(SensorType, SensorSN, Offset));
    }

    //ID:0x31 SensorEMG
    void Asusbot::SetSensorEMG(const uint8_t &SensorType, const uint8_t &SensorSN,
                                  const int16_t &EMGValue){

//#ifdef ASUSBOT_COMMAND
        LOGD("Set Sensor EMG value, type = %d, SN= %d, EMGValue = %d.", SensorType, SensorSN, EMGValue);
//#endif
        sendCommand(Command::SensorEMGValue(SensorType, SensorSN, EMGValue));
    }



//void Asusbot::setGJVendorCommand(const unsigned char &para01, const unsigned char &para02, const unsigned char &para03) //GauJei : Add a vendor command 0707
//{
//  sendCommand(Command::GJVendorCommand(para01, para02, para03));
//}



    void Asusbot::sendCommand(Command command) {
        unsigned char cammand_Length = 0x00, TotalCammandLength = 0x00;
        unsigned char checksum = 0x00;
        unsigned char CommandPackage[255];

        command_mutex.lock();
        CommandPackage[0] = command.header0;
        CommandPackage[1] = command.header1;

        switch (command.data.command) //Command ID
        {
            case 0x01: //ID:0x01 BaseVelocityControl

                CommandPackage[2] = cammand_Length
                                  = 2 + sizeof(command.data.speed) + sizeof(command.data.radius); //Command total length
                checksum ^= (CommandPackage[2]);

                CommandPackage[3] = command.data.command; //Command ID
                checksum ^= (CommandPackage[3]);

                CommandPackage[4] = sizeof(command.data.speed) + sizeof(command.data.radius); //sub-payolad size
                checksum ^= (CommandPackage[4]);

                CommandPackage[5] = LOBYTE(command.data.speed);
                checksum ^= (CommandPackage[5]);

                CommandPackage[6] = HIBYTE(command.data.speed);
                checksum ^= (CommandPackage[6]);

                CommandPackage[7] = LOBYTE(command.data.radius);
                checksum ^= (CommandPackage[7]);

                CommandPackage[8] = HIBYTE(command.data.radius);
                checksum ^= (CommandPackage[8]);

                break;

            case 0x04: //ID:0x04 NeckPositionControl

                CommandPackage[2] = cammand_Length
                                  = 2 + sizeof(command.data.yaw_position) + sizeof(command.data.pitch_position)
                                      + sizeof(command.data.yaw_time) + sizeof(command.data.pitch_time); //Command total length
                checksum ^= (CommandPackage[2]);

                CommandPackage[3] = command.data.command; //Command ID
                checksum ^= (CommandPackage[3]);

                CommandPackage[4] = sizeof(command.data.yaw_position) + sizeof(command.data.pitch_position)
                                    + sizeof(command.data.yaw_time) + sizeof(command.data.pitch_time); //sub-payolad size
                checksum ^= (CommandPackage[4]);

                CommandPackage[5] = LOBYTE(command.data.yaw_position);
                checksum ^= (CommandPackage[5]);

                CommandPackage[6] = HIBYTE(command.data.yaw_position);
                checksum ^= (CommandPackage[6]);

                CommandPackage[7] = LOBYTE(command.data.pitch_position);
                checksum ^= (CommandPackage[7]);

                CommandPackage[8] = HIBYTE(command.data.pitch_position);
                checksum ^= (CommandPackage[8]);

                CommandPackage[9] = LOBYTE(command.data.yaw_time);
                checksum ^= (CommandPackage[9]);

                CommandPackage[10] = HIBYTE(command.data.yaw_time);
                checksum ^= (CommandPackage[10]);

                CommandPackage[11] = LOBYTE(command.data.pitch_time);
                checksum ^= (CommandPackage[11]);

                CommandPackage[12] = HIBYTE(command.data.pitch_time);
                checksum ^= (CommandPackage[12]);

                break;

            case 0x05: //ID:0x05 MotorDrivePWMOutput

                CommandPackage[2] = cammand_Length
                                  = 2 + sizeof(command.data.PWMControllerSelect) +
                                        sizeof(command.data.PWMOutputValue); //Command total length
                checksum ^= (CommandPackage[2]);

                CommandPackage[3] = command.data.command; //Command ID
                checksum ^= (CommandPackage[3]);

                CommandPackage[4] = sizeof(command.data.PWMControllerSelect) +
                                    sizeof(command.data.PWMOutputValue); //sub-payolad size
                checksum ^= (CommandPackage[4]);

                CommandPackage[5] = command.data.PWMControllerSelect;
                checksum ^= (CommandPackage[5]);

                CommandPackage[6] = LOBYTE(command.data.PWMOutputValue);
                checksum ^= (CommandPackage[6]);

                CommandPackage[7] = HIBYTE(command.data.PWMOutputValue);
                checksum ^= (CommandPackage[7]);

                break;
/*
            case e_BaseTaskSize:

                CommandPackage[2] = cammand_Length
                        = 2 + sizeof(command.data.size); //Command total length
                checksum ^= (CommandPackage[2]);

                CommandPackage[3] = command.data.command; //Command ID
                checksum ^= (CommandPackage[3]);

                CommandPackage[4] = sizeof(command.data.size); //sub-payolad size
                checksum ^= (CommandPackage[4]);

                CommandPackage[5] = command.data.size;
                checksum ^= (CommandPackage[5]);

                break;
*/
            case e_BaseTaskData: //ID:0x07 BaseTaskControl

                CommandPackage[2] = cammand_Length
                        = 2 + sizeof(command.data.index) + sizeof(command.data.x) + sizeof(command.data.y) + sizeof(command.data.theta); //Command total length
                checksum ^= (CommandPackage[2]);

                CommandPackage[3] = command.data.command; //Command ID
                checksum ^= (CommandPackage[3]);

                CommandPackage[4] = sizeof(command.data.index) + sizeof(command.data.x) + sizeof(command.data.y) + sizeof(command.data.theta); //sub-payolad size
                checksum ^= (CommandPackage[4]);

                CommandPackage[5] = command.data.index; //sub-payolad size
                checksum ^= (CommandPackage[5]);

                CommandPackage[6] = LOBYTE(command.data.x);
                checksum ^= (CommandPackage[6]);

                CommandPackage[7] = HIBYTE(command.data.x);
                checksum ^= (CommandPackage[7]);

                CommandPackage[8] = LOBYTE(command.data.y);
                checksum ^= (CommandPackage[8]);

                CommandPackage[9] = HIBYTE(command.data.y);
                checksum ^= (CommandPackage[9]);

                CommandPackage[10] = LOBYTE(command.data.theta);
                checksum ^= (CommandPackage[10]);

                CommandPackage[11] = HIBYTE(command.data.theta);
                checksum ^= (CommandPackage[11]);

                break;

            case 0x08: //ID:0x08 RequestFeedbackInfo

                CommandPackage[2] = cammand_Length
                                  = 2 + sizeof(command.data.request_feedback_flags); //Command total length
                checksum ^= (CommandPackage[2]);

                CommandPackage[3] = command.data.command; //Command ID
                checksum ^= (CommandPackage[3]);

                CommandPackage[4] = sizeof(command.data.request_feedback_flags); //sub-payolad size
                checksum ^= (CommandPackage[4]);

                CommandPackage[5] = command.data.request_feedback_flags;
                checksum ^= (CommandPackage[5]);


                break;

            case 0x09: //ID:0x09 RequestExtra

                CommandPackage[2] = cammand_Length
                        = 2 + sizeof(command.data.request_flags); //Command total length
                checksum ^= (CommandPackage[2]);

                CommandPackage[3] = command.data.command; //Command ID
                checksum ^= (CommandPackage[3]);

                CommandPackage[4] = sizeof(command.data.request_flags); //sub-payolad size
                checksum ^= (CommandPackage[4]);

                CommandPackage[5] = command.data.request_flags;
                checksum ^= (CommandPackage[5]);

                break;


            case 0x0A: //ID:0x0A SetCommandExtra

                CommandPackage[2] = cammand_Length
                        = 2 + sizeof(command.data.ExtraList); //Command total length
                checksum ^= (CommandPackage[2]);

                CommandPackage[3] = command.data.command; //Command ID
                checksum ^= (CommandPackage[3]);

                CommandPackage[4] = sizeof(command.data.ExtraList); //sub-payolad size
                checksum ^= (CommandPackage[4]);

                CommandPackage[5] = command.data.ExtraList;
                checksum ^= (CommandPackage[5]);

                break;

            case 0x10: //ID:0x10 MotorControlSwitch

                CommandPackage[2] = cammand_Length
                        = 2 + sizeof(command.data.ControllerSwitch); //Command total length
                checksum ^= (CommandPackage[2]);

                CommandPackage[3] = command.data.command; //Command ID
                checksum ^= (CommandPackage[3]);

                CommandPackage[4] = sizeof(command.data.ControllerSwitch); //sub-payolad size
                checksum ^= (CommandPackage[4]);

                CommandPackage[5] = command.data.ControllerSwitch;
                checksum ^= (CommandPackage[5]);

                break;

            case 0x11: //ID:0x11 ControllerOutputLimit

                CommandPackage[2] = cammand_Length
                        = 2 + sizeof(command.data.ControlType) + sizeof(command.data.ControllerSN)
                          + sizeof(command.data.OutputUpperLimit) + sizeof(command.data.OutputLowerLimit); //Command total length
                checksum ^= (CommandPackage[2]);

                CommandPackage[3] = command.data.command; //Command ID
                checksum ^= (CommandPackage[3]);

                CommandPackage[4] = sizeof(command.data.ControlType) + sizeof(command.data.ControllerSN)
                                    + sizeof(command.data.OutputUpperLimit) + sizeof(command.data.OutputLowerLimit); //sub-payolad size
                checksum ^= (CommandPackage[4]);

                CommandPackage[5] = command.data.ControlType;
                checksum ^= (CommandPackage[5]);

                CommandPackage[6] = command.data.ControllerSN;
                checksum ^= (CommandPackage[6]);

                CommandPackage[7] = LOBYTE(command.data.OutputUpperLimit);
                checksum ^= (CommandPackage[7]);

                CommandPackage[8] = HIBYTE(command.data.OutputUpperLimit);
                checksum ^= (CommandPackage[8]);

                CommandPackage[9] = LOBYTE(command.data.OutputLowerLimit);
                checksum ^= (CommandPackage[9]);

                CommandPackage[10] = HIBYTE(command.data.OutputLowerLimit);
                checksum ^= (CommandPackage[10]);

                break;

            case 0x12: //ID:0x12 ControllerOffset

                CommandPackage[2] = cammand_Length
                        = 2 + sizeof(command.data.ControlType) + sizeof(command.data.ControllerSN)
                          + sizeof(command.data.ControllerOffset); //Command total length
                checksum ^= (CommandPackage[2]);

                CommandPackage[3] = command.data.command; //Command ID
                checksum ^= (CommandPackage[3]);

                CommandPackage[4] = sizeof(command.data.ControlType) + sizeof(command.data.ControllerSN)
                                    + sizeof(command.data.ControllerOffset); //sub-payolad size
                checksum ^= (CommandPackage[4]);

                CommandPackage[5] = command.data.ControlType;
                checksum ^= (CommandPackage[5]);

                CommandPackage[6] = command.data.ControllerSN;
                checksum ^= (CommandPackage[6]);

                CommandPackage[7] = LOBYTE(command.data.ControllerOffset);
                checksum ^= (CommandPackage[7]);

                CommandPackage[8] = HIBYTE(command.data.ControllerOffset);
                checksum ^= (CommandPackage[8]);

                break;

            case 0x14: //ID:0x14 ControllerInfo

                CommandPackage[2] = cammand_Length
                        = 2 + sizeof(command.data.ControlType) + sizeof(command.data.ControllerSN)
                          + sizeof(command.data.InfoType); //Command total length
                checksum ^= (CommandPackage[2]);

                CommandPackage[3] = command.data.command; //Command ID
                checksum ^= (CommandPackage[3]);

                CommandPackage[4] = sizeof(command.data.ControlType) + sizeof(command.data.ControllerSN)
                                    + sizeof(command.data.InfoType); //sub-payolad size
                checksum ^= (CommandPackage[4]);

                CommandPackage[5] = command.data.ControlType;
                checksum ^= (CommandPackage[5]);

                CommandPackage[6] = command.data.ControllerSN;
                checksum ^= (CommandPackage[6]);

                CommandPackage[7] = command.data.InfoType;
                checksum ^= (CommandPackage[7]);

                break;



            case 0x30: //ID:0x30 SensorCalOffset

                CommandPackage[2] = cammand_Length
                        = 2 + sizeof(command.data.SensorType) + sizeof(command.data.SensorSN)
                          + sizeof(command.data.SensorCalOffset); //Command total length
                checksum ^= (CommandPackage[2]);

                CommandPackage[3] = command.data.command; //Command ID
                checksum ^= (CommandPackage[3]);

                CommandPackage[4] = sizeof(command.data.SensorType) + sizeof(command.data.SensorSN)
                                    + sizeof(command.data.SensorCalOffset); //sub-payolad size
                checksum ^= (CommandPackage[4]);

                CommandPackage[5] = command.data.SensorType;
                checksum ^= (CommandPackage[5]);

                CommandPackage[6] = command.data.SensorSN;
                checksum ^= (CommandPackage[6]);

                CommandPackage[7] = LOBYTE(command.data.SensorCalOffset);
                checksum ^= (CommandPackage[7]);

                CommandPackage[8] = HIBYTE(command.data.SensorCalOffset);
                checksum ^= (CommandPackage[8]);

                break;

            case 0x31: //ID:0x31 SensorEMG

                CommandPackage[2] = cammand_Length
                        = 2 + sizeof(command.data.SensorType) + sizeof(command.data.SensorSN)
                          + sizeof(command.data.SensorEMGValue); //Command total length
                checksum ^= (CommandPackage[2]);

                CommandPackage[3] = command.data.command; //Command ID
                checksum ^= (CommandPackage[3]);

                CommandPackage[4] = sizeof(command.data.SensorType) + sizeof(command.data.SensorSN)
                                    + sizeof(command.data.SensorEMGValue); //sub-payolad size
                checksum ^= (CommandPackage[4]);

                CommandPackage[5] = command.data.SensorType;
                checksum ^= (CommandPackage[5]);

                CommandPackage[6] = command.data.SensorSN;
                checksum ^= (CommandPackage[6]);

                CommandPackage[7] = LOBYTE(command.data.SensorEMGValue);
                checksum ^= (CommandPackage[7]);

                CommandPackage[8] = HIBYTE(command.data.SensorEMGValue);
                checksum ^= (CommandPackage[8]);

                break;


            default:
                #ifdef ASUSBOT_COMMAND
                    LOGE("No serialization ! Add it please...");
                #endif
                break;
        }

        //headerx2 + payolad size + sub-payolad id + sub-payolad size + data + checksum
        TotalCammandLength = 2 + 1 + cammand_Length + 1;
        CommandPackage[TotalCammandLength - 1] = checksum;

        //#ifdef ASUSBOT_COMMAND
        LOGD("Cammand =");
        for (int j = 0; j < TotalCammandLength; j++) {
            LOGD(" %x", CommandPackage[j]);
        }
        //#endif

        vcp.WritePort(CommandPackage, TotalCammandLength);

        command_mutex.unlock();
    }

    void Asusbot::packageCommand(uint8_t cmdID, uint8_t cmdSize, uint8_t data[]) {
        unsigned char cammand_Length = 0x00, TotalCammandLength = 0x00;
        unsigned char checksum = 0x00;
        unsigned char CommandPackage[255];

        command_mutex.lock();

        CommandPackage[0] = 0xaa;   // mCommand.header0;    // 0xaa
        CommandPackage[1] = 0x55;   // mCommand.header1;    // 0x55

        // sub-package w/o considering the header, but including cmdID and cmdSize
        CommandPackage[2] = cammand_Length
                = 2 + cmdSize;         // sizeof(data)/ sizeof(data[0]); //cmdID + cmdSize + data
        checksum ^= (CommandPackage[2]);

        CommandPackage[3] = cmdID; //Command ID
        checksum ^= (CommandPackage[3]);

        CommandPackage[4] = cmdSize; //sub-payolad size
        checksum ^= (CommandPackage[4]);

        for (int i = 0 ; i < cmdSize ; i++) {
            CommandPackage[i+5] = data[i];
            checksum ^= (CommandPackage[i+5]);
        }
        CommandPackage[cmdSize+5] = checksum;               // 5 = headerx2 + payolad size + sub-payolad id + sub-payolad size
        TotalCammandLength = 2 + 1 + cammand_Length + 1;    //headerx2 + payolad size + sub-payolad id + sub-payolad size + data + checksum

        //LOGD("VCPdebug : %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x, %x", CommandPackage[0], CommandPackage[1], CommandPackage[2], CommandPackage[3], CommandPackage[4],
        //     CommandPackage[5], CommandPackage[6], CommandPackage[7], CommandPackage[8], CommandPackage[9], CommandPackage[10], CommandPackage[11], CommandPackage[12]);

        //#ifdef ASUSBOT_COMMAND
        LOGD("Cammand =");
        for (int j = 0; j < TotalCammandLength; j++) {
            LOGD(" %x", CommandPackage[j]);
        }
        //#endif

        vcp.WritePort(CommandPackage, TotalCammandLength);

        command_mutex.unlock();
    }

    bool Asusbot::enable() {
        is_enabled = true;
        return true;
    }

    bool Asusbot::disable() {
        setBaseControl(0.0f, 0.0f);
        //setGJVendorCommand(0x01, 0x01, 0x09); //GauJei : Add a vendor command 0707
        is_enabled = false;

        return true;
    }

    void Asusbot::lockDataAccess() {
        data_mutex.lock();
    }

    void Asusbot::unlockDataAccess() {
        data_mutex.unlock();
    }
}