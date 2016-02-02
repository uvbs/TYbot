/*
 * command.cpp
 *
 *  Created on: 2015年8月19日
 *      Author: mint
 */


#include "../include/command.h"
#include "../include/EnumDefine.h"

namespace asusbot {

    const unsigned char Command::header0 = 0xaa;
    const unsigned char Command::header1 = 0x55;

    //ID:0x01 BaseVelocityControl
    Command Command::SetVelocityControl(const int16_t &speed, const int16_t &radius) {
        Command outgoing;
        outgoing.data.speed = speed;
        outgoing.data.radius = radius;
        outgoing.data.command = Command::BaseVelocityControl;
        return outgoing;
    }

    //ID:0x04 NeckPositionControl
    Command Command::SetNeckPositionControl(const int16_t &yaw_position, const int16_t &pitch_position,
                                            const int16_t &yaw_time, const int16_t &pitch_time) {
        Command outgoing;
        outgoing.data.yaw_position = yaw_position;
        outgoing.data.pitch_position = pitch_position;
        outgoing.data.yaw_time = yaw_time;
        outgoing.data.pitch_time = pitch_time;
        outgoing.data.command = Command::NeckPositionControl;
        return outgoing;
    }

    //ID:0x05 MotorDrivePWMOutput
        Command Command::SetMotorDrivePWMOutput(const uint8_t &PWMControllerSelect, const int16_t &PWMOutputValue){
        Command outgoing;
        outgoing.data.PWMControllerSelect = PWMControllerSelect;
        outgoing.data.PWMOutputValue = PWMOutputValue;
        outgoing.data.command = Command::MotorDrivePWMOutput;
        return outgoing;
    }

/*
    // ID:0x06 BaseTaskSize
    Command Command::BaseTaskSize(const uint8_t &size) {
        Command outgoing;
        outgoing.data.size = size;
        outgoing.data.command = e_BaseTaskSize;
        return outgoing;
    }
*/

    // ID:0x07 BaseTaskControl
    Command Command::MoveBaseTaskControl(const uint8_t &index, const int16_t &x, const int16_t &y, const int16_t &theta) {
        Command outgoing;
        outgoing.data.index = index;
        outgoing.data.x = x;
        outgoing.data.y = y;
        outgoing.data.theta = theta;
        outgoing.data.command = Command::BaseTaskControl;
        return outgoing;
    }

    //ID:0x08 RequestFeedbackInfo
    Command Command::SetFeedbackInfo(const uint8_t &RequestFeedback) {
        Command outgoing;
        outgoing.data.request_feedback_flags = RequestFeedback;
        outgoing.data.command = Command::RequestFeedbackInfo;
        return outgoing;
    }

    //ID:0x09 RequestExtra
    Command Command::GetVersionInfo() {
        Command outgoing;
        outgoing.data.request_flags = 0;
        outgoing.data.request_flags |= static_cast<uint8_t>(HardwareVersion);
        outgoing.data.request_flags |= static_cast<uint8_t>(FirmwareVersion);
        outgoing.data.request_flags |= static_cast<uint8_t>(UniqueDeviceID);
        outgoing.data.command = Command::RequestExtra;
        return outgoing;
    }

    //ID:0x0A SetCommandExtra
    Command Command::ExtraCommand(const int8_t &ExCommand) {
        Command outgoing;
        outgoing.data.ExtraList = ExCommand;
        outgoing.data.command = Command::SetCommandExtra;
        return outgoing;
    }

    //ID:0x10 MotorControlSwitch
    Command Command::ControllerSwitch(const uint8_t &CtrlSwitch) {
        Command outgoing;
        outgoing.data.ControllerSwitch = CtrlSwitch;
        outgoing.data.command = Command::MotorControlSwitch;
        return outgoing;
    }

    //ID:0x11 ControllerOutputLimit
    Command Command::ControllerOutputLimitValue(const uint8_t &type, const uint8_t &SN,
                                           const int16_t &UpperLimit, const int16_t &LowerLimit){
        Command outgoing;
        outgoing.data.ControlType = type;
        outgoing.data.ControllerSN = SN;
        outgoing.data.OutputUpperLimit = UpperLimit;
        outgoing.data.OutputLowerLimit = LowerLimit;
        outgoing.data.command = Command::ControllerOutputLimit;
        return outgoing;
    }

    //ID:0x12 ControllerOffset
    Command Command::ControllerOffsetValue(const uint8_t &type, const uint8_t &SN,
                                           const int16_t &Offset){
        Command outgoing;
        outgoing.data.ControlType = type;
        outgoing.data.ControllerSN = SN;
        outgoing.data.ControllerOffset = Offset;
        outgoing.data.command = Command::ControllerOffset;
        return outgoing;

    }

    //ID:0x13 ControllerGain
    Command Command::ControllerGainValue(const uint8_t &type, const uint8_t &SN,
                                         const uint16_t &p_gain, const uint16_t &i_gain,
                                         const uint16_t &d_gain){
        Command outgoing;
        outgoing.data.ControlType = type;
        outgoing.data.ControllerSN = SN;
        outgoing.data.p_gain = p_gain;
        outgoing.data.i_gain = i_gain;
        outgoing.data.d_gain = d_gain;
        outgoing.data.command = Command::ControllerGain;
        return outgoing;
    }

    //ID:0x14 ControllerInfo
    Command Command::ControllerInformation(const uint8_t &type, const uint8_t &SN,
                                     const uint8_t &InfoType){
        Command outgoing;
        outgoing.data.ControlType = type;
        outgoing.data.ControllerSN = SN;
        outgoing.data.InfoType = InfoType;
        outgoing.data.command = Command::ControllerInfo;
        return outgoing;
    }

    //ID:0x30 SensorCalOffset
    Command Command::Sensoroffset(const uint8_t &type, const uint8_t &SN,
                                const int16_t &Offset){
        Command outgoing;
        outgoing.data.SensorType = type;
        outgoing.data.SensorSN = SN;
        outgoing.data.SensorCalOffset = Offset;
        outgoing.data.command = Command::SensorCalOffset;
        return outgoing;

    }

    //ID:0x31 SensorEMG
    Command Command::SensorEMGValue(const uint8_t &type, const uint8_t &SN,
                                  const int16_t &emg_value){
        Command outgoing;
        outgoing.data.SensorType = type;
        outgoing.data.SensorSN = SN;
        outgoing.data.SensorEMGValue = emg_value;
        outgoing.data.command = Command::SensorEMG;
        return outgoing;

    }

    //GauJei : Add a vendor command 0707
    //Command Command::GJVendorCommand(const unsigned char &para01, const unsigned char &para02, const unsigned char &para03)
    //{
    //	  Command outgoing;
    //	  outgoing.data.para01 = para01;
    //	  outgoing.data.para02 = para02;
    //	  outgoing.data.para03 = para03;
    //	  outgoing.data.command = Command::GauJei;
    //	  return outgoing;
    //}

}

