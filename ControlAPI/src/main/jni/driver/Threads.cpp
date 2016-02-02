/*
 * Threads.c
 *
 *  Created on: 2015年8月25日
 *      Author: mint
 */

#include "../include/Threads.h"
#include <stdio.h> /* printf */

#include "../include/DebugMessage.h"
#define LOG_TAG "Threads"

extern bool PortIsOpen;

SensorStatus sensorstatus;
_BASETASK 		absBaseTask;
_AVOIDTARGET	AvoidTarget;
Encoder encoder;
SensorRAW sensorraw;
Version version;
ControllerInfo controllerinfo;
SensorInfo sensorinfo;



void *ReceiverThread(void *argv);

namespace asusbot
{
	Asusbot asusbot;
//	pthread_t rx_thread_id;
	pthread_t parse_thread_id;

	Threads::Threads()
	{
		LOGI("rxThread create~~");
	}

	Threads::~Threads()
	{
		LOGI("kill rx_thread !!");
		pthread_kill(rx_thread_id, SIGKILL);
	}

	void *ReceiverThread(void *argv)
	{

	}

	void  Threads::StartReceiverThread()
	{
		pthread_mutex_init(&lock, NULL);

		int result;

		result = pthread_create(&rx_thread_id, NULL, ReceiverThread, (void *) this);

		if (result == 0)
		{
			#ifdef THREADS_RECEIVER
			LOGI("Receiver thread create success !!");
			#endif
		}
		else
		{
			LOGE("Receiver thread create fail !!.");
		}

	}

}
