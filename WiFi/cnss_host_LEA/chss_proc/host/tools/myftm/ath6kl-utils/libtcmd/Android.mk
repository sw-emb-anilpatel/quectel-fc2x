LOCAL_PATH:=$(call my-dir)

# Build libtcmd =========================
include $(CLEAR_VARS)

LOCAL_CLANG := true
LOCAL_MODULE := libtcmd
LOCAL_SRC_FILES:= \
    nl80211.c \
    libtcmd.c \
    os.c

ifeq ($(BOARD_HAS_ATH_WLAN_AR6004),true)
	LOCAL_CFLAGS+= -DCONFIG_AR6002_REV6
endif

ifneq ($(wildcard external/libnl-headers),)
LOCAL_C_INCLUDES += external/libnl-headers
else
LOCAL_C_INCLUDES += external/libnl/include
endif

PATH := out/target/product/x86/obj/kernel
LOCAL_C_INCLUDES += ${PATH}/usr/include
LOCAL_ADDITIONAL_DEPENDENCIES := ${PATH}/usr


LOCAL_COPY_HEADERS_TO := libtcmd
LOCAL_COPY_HEADERS := libtcmd.h

LOCAL_CFLAGS += \
	-DWLAN_API_NL80211 \
	-DANDROID \
	-DLIBNL_2 \

ifneq ($(wildcard system/core/libnl_2),)
# ICS ships with libnl 2.0
LOCAL_SHARED_LIBRARIES := libnl_2
else
LOCAL_SHARED_LIBRARIES := libnl
endif


LOCAL_MODULE_OWNER := qcom

include $(BUILD_STATIC_LIBRARY)
