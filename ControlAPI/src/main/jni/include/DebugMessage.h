//
// Created by gaujei on 15年9月22日.
//

#ifndef ASUSBOT_DEBUGMESSAGE_H
#define ASUSBOT_DEBUGMESSAGE_H

#include <android/log.h>

//#define THREADS_RECEIVER
//#define DRIVER_INTERFACE
//#define ASUSBOT_COMMAND
#define VIRTRUAL_COMPORT

//#define DEBUG_FAKETIMESTAMP //for no VCP

#define LOGD(...) ((void)__android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__))
#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__))
#define LOGW(...) ((void)__android_log_print(ANDROID_LOG_WARN, LOG_TAG, __VA_ARGS__))
#define LOGE(...) ((void)__android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__))

#endif //ASUSBOT_DEBUGMESSAGE_H
