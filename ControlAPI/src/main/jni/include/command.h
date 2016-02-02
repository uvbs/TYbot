/*
 * command.h
 *
 *  Created on: 2015年8月19日
 *      Author: mint
 */

#ifndef INCLUDE_COMMAND_H_
#define INCLUDE_COMMAND_H_
#include <stdint.h>
#include <stdio.h> // printf

namespace asusbot
{
    class Command
    {

      public:

        enum Name
        {
          //Command Identifier
          GauJei = 0x00,

          //Asusbot standard command
          BaseVelocityControl = 0x01, //(speed, radius)
          BasePositionControl = 0x02,
          NeckVelocityControl = 0x03,
          NeckPositionControl = 0x04, //(yaw, pitch, yaw_time, pitch_time)
          MotorDrivePWMOutput = 0x05, //(MotorSelect, PWMValue)
          BaseTaskSize = 0x06,
          BaseTaskControl = 0x07, //(x, y, theta)

          RequestFeedbackInfo = 0x08,
          RequestExtra = 0x09,
          SetCommandExtra = 0x0A,
          TaskMotion = 0x0B,

          //Control setting
          MotorControlSwitch = 0x10,
          ControllerOutputLimit = 0x11,
          ControllerOffset = 0x12,
          ControllerGain = 0x13,
          ControllerInfo = 0x14,

          //Vendor command
          VelocityControlTest = 0x23,
          PositionControlTest = 0x24,
          VendorDriveIO = 0x25,

          //Sensor calibration/parameter
          SensorCalOffset = 0x30,
          SensorEMG = 0x31,


          //Periphery
          FanControl = 0x40,
          FanSpeed = 0x41,
          FanProfile = 0x42,
          LEDControl = 0x45,
          ChargeControl = 0x4A,

        };

//        enum FeedbackInfo
//        {
//          DropIR = 0x01, MotorTemp = 0x02, MotorLife = 0x04, LEDInfo = 0x08,
//          BatteryInfo = 0x10, ChargeInfo = 0x20, FanInfo = 0x40, DockInfo = 0x80
//        };

        enum VersionFlag
        {
          HardwareVersion = 0x01, FirmwareVersion = 0x02/*, Time = 0x04*/, UniqueDeviceID = 0x08
        };


        struct Data
        {
          Data()
            : command(BaseVelocityControl),
              speed(0), radius(0), //ID:0x01 BaseVelocityControl
              yaw_position(0), pitch_position(0), yaw_time(0), pitch_time(0), //ID:0x04 NeckPositionControl
              PWMControllerSelect(0), PWMOutputValue(0), //ID:0x05 MotorDrivePWMOutput
              x(0), y(0), theta(0), // ID:0x06 BaseTaskControl
              request_feedback_flags(0),    //ID:0x08 RequestFeedbackInfo
              request_flags(0), //ID:0x09 RequestExtra
              p_gain(1000), i_gain(1000), d_gain(1000),
              para01(0), para02(0), para03(0) //GauJei : Add a vendor command 0707
          {
          }

          Name command;

          //ID:0x01 BaseVelocityControl
          int16_t speed;
          int16_t radius;

          //ID:0x04 NeckPositionControl
          int16_t yaw_position;
          int16_t pitch_position;
          int16_t yaw_time;
          int16_t pitch_time;

          //ID:0x05 MotorDrivePWMOutput
          uint8_t PWMControllerSelect;
          int16_t PWMOutputValue;

          // ID: e_BaseTaskSize
          uint8_t size;

          //ID:0x07 BaseTaskControl
          uint8_t index;
          int16_t x;
          int16_t y;
          int16_t theta;

          //ID:0x08 RequestFeedbackInfo
          uint8_t request_feedback_flags;

          //ID:0x09 RequestExtra (version flags)
          uint8_t request_flags;


          //ID:0x0A SetCommandExtra
          uint8_t ExtraList; //(ClearAlarm)(HardReset)

          //ID:0x10 MotorControlSwitch
          uint8_t ControllerSwitch;

          //ID:0x11 ControllerOutputLimit
          uint8_t ControlType;
          uint8_t ControllerSN;
          int16_t OutputUpperLimit;
          int16_t OutputLowerLimit;

          //ID:0x12 ControllerOffset
          int16_t ControllerOffset;

          //ID:0x13 ControllerGain
          uint16_t p_gain;
          uint16_t i_gain;
          uint16_t d_gain;

          //ID:0x14 ControllerInfo
          uint8_t InfoType;

          //ID:0x30 SensorCalOffset
          uint8_t SensorType;
          uint8_t SensorSN;
          int16_t SensorCalOffset;

          //ID:0x31 SensorEMG
          int16_t SensorEMGValue;



          int8_t para01;
          int8_t para02;
          int8_t para03;



        };

        virtual ~Command(){}

        Data data;

        //ID:0x01 BaseVelocityControl
        static Command SetVelocityControl(const int16_t &speed, const int16_t &radius);

        //ID:0x04 NeckPositionControl
        static Command SetNeckPositionControl(const int16_t &yaw_position, const int16_t &pitch_position,
                                              const int16_t &yaw_time, const int16_t &pitch_time);

        //ID:0x05 MotorDrivePWMOutput
        static Command SetMotorDrivePWMOutput(const uint8_t &PWMControllerSelect, const int16_t &PWMOutputValue);
/*
        //e_BaseTaskSize
        static Command BaseTaskSize(const uint8_t &size);
*/
        //e_BaseTaskData
        static Command MoveBaseTaskControl(const uint8_t &index, const int16_t &x, const int16_t &y, const int16_t &theta);

        //ID:0x08 RequestFeedbackInfo
        static Command SetFeedbackInfo(const uint8_t &RequestFeedback);

        //ID:0x09 RequestExtra
        static Command GetVersionInfo();

        //ID:0x0A SetCommandExtra
        static Command ExtraCommand(const int8_t &ExCommand);

        //ID:0x10 MotorControlSwitch
        static Command ControllerSwitch(const uint8_t &CtrlSwitch);

        //ID:0x11 ControllerOutputLimit
        static Command ControllerOutputLimitValue(const uint8_t &type, const uint8_t &SN,
                                             const int16_t &UpperLimit, const int16_t &LowerLimit);

        //ID:0x12 ControllerOffset
        static Command ControllerOffsetValue(const uint8_t &type, const uint8_t &SN,
                                             const int16_t &Offset);

        //ID:0x13 ControllerGain
        static Command ControllerGainValue(const uint8_t &type, const uint8_t &SN,
                                      const uint16_t &p_gain,
                                      const uint16_t &i_gain,
                                      const uint16_t &d_gain);

        //ID:0x14 ControllerInfo
        static Command ControllerInformation(const uint8_t &type, const uint8_t &SN,
                                         const uint8_t &InfoType);



        //ID:0x30 SensorCalOffset
        static Command Sensoroffset(const uint8_t &type, const uint8_t &SN,
                                             const int16_t &Offset);
        //ID:0x31 SensorEMG
        static Command SensorEMGValue(const uint8_t &type, const uint8_t &SN,
                                    const int16_t &emg_value);





        static Command GJVendorCommand(const unsigned char &para01, const unsigned char &para02, const unsigned char &para03);



        static const unsigned char header0;
        static const unsigned char header1;

      private:


    };

}



#endif /* INCLUDE_COMMAND_H_ */
