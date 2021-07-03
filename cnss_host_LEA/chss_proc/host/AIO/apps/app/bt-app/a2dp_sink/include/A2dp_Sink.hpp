 /*
  * Copyright (c) 2016, The Linux Foundation. All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are
  * met:
  *  * Redistributions of source code must retain the above copyright
  *    notice, this list of conditions and the following disclaimer.
  *  * Redistributions in binary form must reproduce the above
  *    copyright notice, this list of conditions and the following
  *    disclaimer in the documentation and/or other materials provided
  *    with the distribution.
  *  * Neither the name of The Linux Foundation nor the names of its
  *    contributors may be used to endorse or promote products derived
  *    from this software without specific prior written permission.
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

#ifndef A2DP_SINK_APP_H
#define A2DP_SINK_APP_H

#include <map>
#include <string>
#include <hardware/bluetooth.h>
#include <hardware/bt_av.h>
#include <hardware/bt_rc.h>
#include <pthread.h>
#if (defined BT_AUDIO_HAL_INTEGRATION)
#include <hardware/audio.h>
#include <hardware/hardware.h>
#endif

#include "osi/include/log.h"
#include "osi/include/thread.h"
#include "osi/include/config.h"
#include "osi/include/allocator.h"
#include "osi/include/alarm.h"
#include "ipc.h"
#include "utils.h"


typedef enum {
    STATE_NOT_STARTED = 0,
    STATE_DISCONNECTED,
    STATE_PENDING,
    STATE_CONNECTED,
}A2dpSinkState;

#define A2DP_SINK_PCM_FETCH_TIMER_DURATION     40

class A2dp_Sink {

  private:
    config_t *config;
    const bt_interface_t * bluetooth_interface;
    const btav_interface_t *sBtA2dpSinkInterface;
    const btrc_ctrl_interface_t *sBtAvrcpCtrlInterface;
    A2dpSinkState mSinkState;
    bool mAvrcpConnected;

  public:
    A2dp_Sink(const bt_interface_t *bt_interface, config_t *config);
    ~A2dp_Sink();
    void ProcessEvent(BtEvent* pEvent);
    void state_disconnected_handler(BtEvent* pEvent);
    void state_pending_handler(BtEvent* pEvent);
    void state_connected_handler(BtEvent* pEvent);
    void change_state(A2dpSinkState mState);
    char* dump_message(BluetoothEventId event_id);
    pthread_mutex_t lock;
    bt_bdaddr_t mConnectingDevice;
    bt_bdaddr_t mConnectedDevice;
    bt_bdaddr_t mConnectedAvrcpDevice;
    void HandleAvrcpEvents(BtEvent* pEvent);
    void HandleEnableSink();
    void HandleDisableSink();
    void SendPassThruCommandNative(uint8_t key_id);
    ControlStatusType controlStatus;
    uint32_t sample_rate;
    uint8_t channel_count;
    alarm_t *pcm_data_fetch_timer;
#if (defined BT_AUDIO_HAL_INTEGRATION)
    audio_stream_out_t* out_stream;
#endif
    void ConfigureAudioHal();
    void CloseAudioStream();
    size_t pcm_buf_size;
    uint8_t* pcm_buf;
    bool pcm_timer;
    void StartPcmTimer();
    void StopPcmTimer();
    void OnDisconnected();
};

#endif
