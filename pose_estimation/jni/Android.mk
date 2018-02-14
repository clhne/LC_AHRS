LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE    := pose_estimation
LOCAL_SRC_FILES := pose_estimation.cpp
LOCAL_C_INCLUDES := .
LOCAL_LDLIBS    := -lm -llog -pthread -landroid
LOCAL_CFLAGS += -fPIE
LOCAL_LDFLAGS += -fPIE -pie

include $(BUILD_EXECUTABLE)
