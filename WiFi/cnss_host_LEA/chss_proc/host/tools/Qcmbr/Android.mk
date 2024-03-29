# Copyright (C) 2008 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

ifeq ($(CONFIG_QCMBR_UART), y)
LOCAL_CFLAGS += -DQCMBR_UART_QDART
OBJ_UART = uart.o
endif

LOCAL_SRC_FILES := Qcmbr.c diag_pkt_handler.c  tlvCmd_if_Qdart.c Socket.c  q_os_if.c $(OBJ_UART)
LOCAL_MODULE := qcmbr
#LOCAL_C_INCLUDES += 
LOCAL_CFLAGS := -g -DDEBUG -DLINUX_X86 -DANDROID_X86
#LOCAL_STATIC_LIBRARIES := libcutils

include $(BUILD_EXECUTABLE)

