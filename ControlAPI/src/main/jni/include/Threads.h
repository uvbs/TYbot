/*
 * Threads.h
 *
 *  Created on: 2015年8月25日
 *      Author: mint
 */

#ifndef INCLUDE_THREADS_H_
#define INCLUDE_THREADS_H_
#include <pthread.h>
#include "Parameters.h"
#include "asusbot.h"
#include "VCP.h"



struct Version{
	unsigned char Hardware[4]; //Patch, Minor, Major, unused
	int Firmware[4]; //Patch, Minor, Major, unused
};

struct Encoder{
	int wheel_encoder[2]; //Left, right wheel
    int neck[2]; //yaw, pitch
	int velocity[2];
};

struct _BASETASK{
	int x;
	int y;
	int theta;
};

struct _AVOIDTARGET{
	int x;
	int y;
};


struct SensorStatus{
	unsigned int timestamp;
	unsigned char IRDrop;
	unsigned char Sonar;
	unsigned char OverCurrent;
	unsigned char OverTemperature;
	unsigned char Neck;
	unsigned char Mobile;
	unsigned char ChargerandBattery;
};

struct SensorRAW{
	unsigned char IRDrop[3]; //front right, front left, rear center
	unsigned int Sonar[3]; //front right, front left, rear center
	unsigned char DockingIR[2]; //right, center, left
	unsigned int Accelerometer[3]; //X, Y, Z axis
	unsigned int Gyroscope[3]; //X, Y, Z axis
	unsigned int Ecompass[3]; //X, Y, Z axis
	char MotorTemperature[4];
	unsigned int MotorLife[8];
};

struct ControllerInfo{
    int OutputLimit[2]; //upper, lower
	int Offset;
	unsigned int Gain[3]; //Pgain, Igain, Dgain
};

struct SensorInfo{
	int Offset;
	int EMGValue;
};
void *ReceiverThread(void *argv);

namespace asusbot {

class Threads
{

	public:

		pthread_t rx_thread_id;
		pthread_mutex_t lock;

		Threads();
		 ~Threads();

		void join(void);
		void StartReceiverThread(void);
//	private:
};
} /* namespace asusbot */
#endif /* INCLUDE_THREADS_H_ */
