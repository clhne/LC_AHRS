LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE    := test
LOCAL_SRC_FILES := ../../GenericGyroSpec.c ../../GenericMagSpec.c ../../BsxLibraryConfiguration.c ../../BsxFusionLibrary.c ../../test.cpp
LOCAL_C_INCLUDES := -I.
LOCAL_LDLIBS    := -lm -llog -pthread -landroid
LOCAL_CFLAGS += -fPIE
LOCAL_LDFLAGS += -fPIE -pie
include $(BUILD_EXECUTABLE)
