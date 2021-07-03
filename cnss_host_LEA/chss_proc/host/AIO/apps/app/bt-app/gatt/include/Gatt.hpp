/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef GATT_APP_H
#define GATT_APP_H

#pragma once
#include <map>
#include <string>
#include <hardware/bluetooth.h>
#include <hardware/bt_gatt.h>
#include <stdio.h>
#include <string.h>

#include "osi/include/log.h"
#include "osi/include/thread.h"
#include "osi/include/config.h"
#include "osi/include/semaphore.h"
#include "ipc.h"
#include "Rsp.hpp"

#define MAX_GATT_DEVICES    (1)

#define REMOTE_START_PROFILE            (0x01)
#define ALERT_NOTIFICATION_PROFILE      (0x02)
#define CYCLING_SPEED__cADENCE_PROFILE  (0x04)
#define CYCLING_POWER_PROFILE           (0x08)
#define RUNNING_SPEED_CADENCE_PROFILE   (0x10)
#define HUMAN_INTERFACE_DEVICE_PROFILE  (0x20)
#define HEART_RATE_PROFILE              (0x40)

extern const char *BT_GATT_ENABLED;

#define CHECK_PARAM(x)                                                      \
   if (!x) {                                                                \
       ALOGE("'%s' Param is NULL - exiting from function ", __FUNCTION__);  \
       return false;                                                        \
   }

#define CHECK_PARAM_VOID(x)                                                 \
   if (!x) {                                                                \
       ALOGE("'%s' Void Param is NULL - exiting from function ", __FUNCTION__);  \
       return ;                                                              \
   }

class Gatt {
    private:
        config_t *config;
        const bt_interface_t * bluetooth_interface;
        btgatt_interface_t *gatt_interface;

    public:
        Rsp *rsp;
        bt_uuid_t rsp_uuid;
        int le_supported_profiles;

        Gatt(const bt_interface_t *bt_interface, config_t *config);
        ~Gatt();
        void ProcessEvent(BtEvent* );
        bool GattInterfaceInit(const bt_interface_t *);
        void GattInterfaceCleanup(void);
        void HandleGattIpcMsg(BtIpcMsg *);
        void HandleGattsRegisterAppEvent(GattsRegisterAppEvent *);
        void HandleGattsConnectionEvent(GattsConnectionEvent *);
        void HandleGattsServiceAddedEvent(GattsServiceAddedEvent *);
        void HandleGattsCharacteristicAddedEvent(GattsCharacteristicAddedEvent *);
        void HandleGattsDescriptorAddedEvent(GattsDescriptorAddedEvent *);
        void HandleGattsServiceStartedEvent(GattsServiceStartedEvent *);
        void HandleGattsServiceStoppedEvent(GattsServiceStoppedEvent *);
        void HandleGattsServiceDeletedEvent(GattsServiceDeletedEvent *);
        void HandleGattsRequestWriteEvent(GattsRequestWriteEvent *);
        void HandleGattcRegisterAppEvent(GattcRegisterAppEvent *);
        void HandleRspEnableEvent(RspEnableEvent *);
        void HandleRspDisableEvent(RspDisableEvent *);
        bool HandleEnableGatt(void);
        bool HandleDisableGatt(void);
};
#endif

