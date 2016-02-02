#include <string.h>
#include <stdio.h>
#include "../include/txrxQueue.h"

#include "../include/DebugMessage.h"
#define LOG_TAG "txrxQueue"

namespace asusbot {
	txrxQueue::txrxQueue(void) {
		Queue_Init();
	}

	txrxQueue::~txrxQueue() {

	}


	void txrxQueue::Queue_Init() {
		Queue.pr = 0;
		Queue.pw = 0;

		memset(Queue.buffer, 0, sizeof(int) * txrxQueue_Buff);
		pthread_mutex_init(&lock, NULL);

	}

	void txrxQueue::Queue_Clear() {
		Queue.pr = 0;
		Queue.pw = 0;
		memset(Queue.buffer, 0, sizeof(int) * txrxQueue_Buff);
	}

	bool txrxQueue::Queue_Is_Empty() {
		if (Queue.pr == Queue.pw)
			return true;
		return false;
	}

	bool txrxQueue::Queue_Is_Full() {
		if ((Queue.pw + 1) % txrxQueue_Buff == Queue.pr)
		{

			LOGE("Queue is full !");
			return true;
		}
		return false;
	}

	int txrxQueue::pop() {
		int data;
		pthread_mutex_lock(&lock);

		if (Queue_Is_Empty())
			return false;

		data = Queue.buffer[Queue.pr];

		Queue.pr++;
		Queue.pr %= txrxQueue_Buff;
		pthread_mutex_unlock(&lock);
		return data;
	}

	bool txrxQueue::push(int data) {

		pthread_mutex_lock(&lock);

		if (Queue_Is_Full())
			return false;

		Queue.buffer[Queue.pw] = data;

		Queue.pw++;
		Queue.pw %= txrxQueue_Buff;
		pthread_mutex_unlock(&lock);
		return true;

	}
}