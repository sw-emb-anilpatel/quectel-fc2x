LOCAL_PATH := $(call my-dir)

# ============================================================
include $(CLEAR_VARS)

LOCAL_REQUIRED_MODULES :=

LOCAL_CFLAGS += -Wno-unused-parameter -Wall -Werror
LOCAL_CPPFLAGS += -Wno-conversion-null
ifeq ($(TARGET_BUILD_VARIANT),userdebug)
LOCAL_CFLAGS += "-DLOG_NDEBUG=0"
endif
LOCAL_MULTILIB: = "both"
LOCAL_CLANG_CFLAGS := -Wno-pointer-bool-conversion

LOCAL_C_INCLUDES += \
	$(LOCAL_PATH) \
	external/libnl/include

LOCAL_SRC_FILES := \
	wifi_hal.cpp \
	common.cpp \
	cpp_bindings.cpp \
	ifaceeventhandler.cpp \
	nan.cpp \
	nan_ind.cpp \
	nan_req.cpp \
	nan_rsp.cpp

LOCAL_MODULE := libnanwifihal
LOCAL_PROPRIETARY_MODULE := true
LOCAL_CLANG := true

LOCAL_SHARED_LIBRARIES += libnl

include $(BUILD_SHARED_LIBRARY)
