LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE    := pose_estimation_Madgwick
LOCAL_SRC_FILES := pose_estimation_Madgwick.cpp MadgwickAHRS.cpp MadgwickAHRS.h
LOCAL_C_INCLUDES := .
LOCAL_LDLIBS    := -lm -llog -pthread -landroid
LOCAL_CFLAGS += -fPIE
LOCAL_LDFLAGS += -fPIE -pie

include $(BUILD_EXECUTABLE)
