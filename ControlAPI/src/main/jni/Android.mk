LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_SRC_FILES := DriverInterface.cpp asusbotmainloop.cpp \
                   driver/asusbot.cpp driver/command.cpp driver/DiffDrive.cpp driver/Threads.cpp \
                   driver/txrxQueue.cpp driver/VCP.cpp \
                   driver/PathFinding.cpp  driver/Initialization.cpp \

LOCAL_LDLIBS += -llog
LOCAL_MODULE := Asusdriver
LOCAL_CFLAGS :=
include $(BUILD_SHARED_LIBRARY)