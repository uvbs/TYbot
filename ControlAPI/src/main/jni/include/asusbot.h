#ifndef ASUSBOT_H_
#define ASUSBOT_H_

#include <string.h>
#include <iomanip>

#include "command.h"
#include "txrxQueue.h"
#include "VCP.h"
#include "Mutex.h"
#include "Threads.h"
#include "Parameters.h"
#include "DiffDrive.h"


//#define ExecutionTimeCost
#define debug_uart //Show debug message
	#ifdef debug_uart
		//#define debug_uart_cmdpackage
		#define debug_uart_init
#endif

#define FAIL 0x01
#define PASS 0x00

#define HIBYTE(cn) (0xff00 & (int)(cn)) >> 8
#define LOBYTE(cn) (int(cn)) & 0xff


#ifdef parse_options
extern int JNICommandParse[20];
#endif

namespace asusbot
{
	union union_sint16
	{
	  short word;
	  unsigned char byte[2];
	};


	class Asusbot
	{
		public:

			VCP vcp;
			Parameters parameters;
			DiffDrive diff_drive;

			asusbot::Mutex command_mutex;
			asusbot::Mutex data_mutex;

			Asusbot();
		   ~Asusbot();

			void init(Parameters&paremeters);
			void spin();

			void RCMThread_sleep_us(int ut);
			void RCMThread_sleep_ms(int mt);

			void setBaseControl(const double &linear_velocity, const double &angular_velocity);
			void setNeckControl(const int16_t &yaw_position,
								const int16_t &pitch_position, const int16_t &yaw_time, const int16_t &pitch_time);
			void BaseTaskSize(const uint8_t &size);
			void moveBaseTaskControl(const uint8_t &index, const int16_t &x, const int16_t &y, const int16_t &theta);
			void SetPWMControl(const int8_t &PWMControllerSelect, const int16_t &PWMOutputValue);
			void SetFeedbackInfo(const int8_t &RequestFeedback);
			void GetVersionInfo();
			void SetExtraCommand(const int8_t &ExCommand);
			void SetControllerSwitch(const int8_t &CtrlSwitch);
			void SetControllerOutputLimit(const int8_t &CtrlType, const int8_t &CtrlSN,
										  const int16_t &UpperLimit, const int16_t &LowerLimit);
			void SetControllerOutputOffset(const int8_t &CtrlType, const int8_t &CtrlSN,
										   const int16_t &Offset);
			void SetControllerGain(const int8_t &CtrlType, const int8_t &CtrlSN,
								   const int16_t &p_gain, const int16_t &i_gain, const int16_t &d_gain);

			void GetControllerInfo(const int8_t &CtrlType, const int8_t &CtrlSN, const int8_t &info_type);

			void SetSensorOffset(const uint8_t &SensorType, const uint8_t &SensorSN,
								 const int16_t &Offset);
			void SetSensorEMG(const uint8_t &SensorType, const uint8_t &SensorSN,
								   const int16_t &EMGValue);


			void sendCommand(Command command);
			void packageCommand(uint8_t cmdID, uint8_t cmdSize, uint8_t data[]);
			bool enable();
			bool disable();

			//Getters data protection
			void lockDataAccess();
			void unlockDataAccess();

		private:
			unsigned char CheckSum(unsigned char checksum, unsigned char value);
			struct timespec abstime;
	};
}
#endif
