LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_MODULE := nantool
ifeq ($(PRODUCT_VENDOR_MOVE_ENABLED),true)
#LOCAL_PROPRIETARY_MODULE := true
endif
LOCAL_MODULE_TAGS := optional
LOCAL_MULTILIB: = "both"
LOCAL_CFLAGS += -D"NAN_CERT_VERSION=4"
LOCAL_CLANG := true
LOCAL_C_INCLUDES := \
	$(TARGET_OUT_HEADERS)/external/libnl/include \
	$(LOCAL_PATH)/inc/

LOCAL_LD_FLAGS := -Wl

LOCAL_SRC_FILES := \
	halProxyDaemon.cpp \
	nan_test.cpp

include $(PREBUILT_SHARED_LIBRARY)

LOCAL_SHARED_LIBRARIES := \
	libnl \
        libnanwifihal

LOCAL_CFLAGS += \
	-DCONFIG_DEBUG \
	-DCONFIG_DEBUG_LOGCAT

include $(BUILD_EXECUTABLE)
