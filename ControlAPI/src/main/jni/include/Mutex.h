/*
 * Mutex.h
 *
 *  Created on: 2015年8月25日
 *      Author: mint
 */

#ifndef MUTEX_H_
#define MUTEX_H_

#include <pthread.h>

namespace asusbot {

//GauJei : http://vichargrave.com/mutex-class-in-c/
class Mutex {

public:

    pthread_mutex_t  m_mutex;

    Mutex(){
    	pthread_mutex_init(&m_mutex, NULL);
    }
    virtual ~Mutex(){
    	pthread_mutex_destroy(&m_mutex);
    }
    int lock()  {
    	return  pthread_mutex_lock(&m_mutex);
    }
    int trylock() {
    	return  pthread_mutex_lock(&m_mutex);
    }
    int unlock()  {
    	return  pthread_mutex_unlock(&m_mutex);
    }


};

} /* namespace asusbot */

#endif /* MUTEX_H_ */
