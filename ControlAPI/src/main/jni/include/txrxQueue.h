#ifndef _TXRXQUEUE_H
#define _TXRXQUEUE_H

#include <cstdlib>
#include <iostream>
#include  <pthread.h>

#define txrxQueue_Buff    4096

namespace asusbot {
	class txrxQueue;

	typedef struct {
		int pr;
		int pw;
		int buffer[txrxQueue_Buff];
	} QUEUE;

	class txrxQueue {
	public:
		txrxQueue(void);

		~txrxQueue();

		pthread_mutex_t lock;
		QUEUE Queue;

		int pop();
		bool push(int data);
		bool Queue_Is_Full();
		bool Queue_Is_Empty();
		void Queue_Init();
		void Queue_Clear();
	};
}
#endif
