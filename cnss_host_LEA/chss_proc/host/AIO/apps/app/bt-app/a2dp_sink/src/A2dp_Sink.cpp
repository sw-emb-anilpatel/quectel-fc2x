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

#include <list>
#include <map>
#include <iostream>
#include <string.h>
#include <hardware/bluetooth.h>
#include <hardware/hardware.h>
#include <hardware/bt_av.h>
#include "Audio_Manager.hpp"

#include "A2dp_Sink.hpp"
#include "Gap.hpp"

#define LOGTAG "A2DP_SINK"
#define LOGTAG_CTRL "AVRCP_CTRL"

using namespace std;
using std::list;
using std::string;

A2dp_Sink *pA2dpSink = NULL;
extern BT_Audio_Manager *pBTAM;

#if (!defined(BT_AUDIO_HAL_INTEGRATION))
#define DUMP_PCM_DATA TRUE
#endif

#if (defined(DUMP_PCM_DATA) && (DUMP_PCM_DATA == TRUE))
FILE *outputPcmSampleFile;
char outputFilename [50] = "/etc/bluetooth/output_sample.pcm";
#endif

#ifdef __cplusplus
extern "C" {
#endif

void BtA2dpSinkMsgHandler(void *msg) {
    BtEvent* pEvent = NULL;
    BtEvent* pCleanupEvent = NULL;
    if(!msg) {
        printf("Msg is NULL, return.\n");
        return;
    }

    pEvent = ( BtEvent *) msg;
    switch(pEvent->event_id) {
        case PROFILE_API_START:
            ALOGD(LOGTAG " enable a2dp sink");
            if (pA2dpSink) {
                pA2dpSink->HandleEnableSink();
            }
            break;
        case PROFILE_API_STOP:
            ALOGD(LOGTAG " disable a2dp sink");
            if (pA2dpSink) {
                pA2dpSink->HandleDisableSink();
            }
            break;
        case A2DP_SINK_CLEANUP_REQ:
            ALOGD(LOGTAG " cleanup a2dp sink");
            if (pA2dpSink) {
                pA2dpSink->CloseAudioStream();
                pA2dpSink->StopPcmTimer();
            }
            pCleanupEvent = new BtEvent;
            pCleanupEvent->event_id = A2DP_SINK_CLEANUP_DONE;
            PostMessage(THREAD_ID_GAP, pCleanupEvent);
            break;
        case AVRCP_CTRL_CONNECTED_CB:
        case AVRCP_CTRL_DISCONNECTED_CB:
        case AVRCP_CTRL_PASS_THRU_CMD_REQ:
            if (pA2dpSink) {
                pA2dpSink->HandleAvrcpEvents(( BtEvent *) msg);
            }
            break;
        default:
            if(pA2dpSink) {
               pA2dpSink->ProcessEvent(( BtEvent *) msg);
            }
            break;
    }
    delete pEvent;
}

#ifdef __cplusplus
}
#endif


void pcm_fetch_timer_handler(void *context) {
    ALOGV(LOGTAG, " pcm_fetch_timer_handler ");

    BtEvent *pEvent = new BtEvent;
    pEvent->a2dpSinkEvent.event_id = A2DP_SINK_FETCH_PCM_DATA;
    PostMessage(THREAD_ID_A2DP_SINK, pEvent);
}

void A2dp_Sink::StartPcmTimer() {
    if(pcm_timer) {
        ALOGD(LOGTAG " PCM Timer still running + ");
        return;
    }
    alarm_set(pcm_data_fetch_timer, A2DP_SINK_PCM_FETCH_TIMER_DURATION,
           pcm_fetch_timer_handler, NULL);
    pcm_timer = true;
}
void A2dp_Sink::StopPcmTimer() {
    if((pcm_data_fetch_timer != NULL) && (pcm_timer)) {
        alarm_cancel(pcm_data_fetch_timer);
        pcm_timer = false;
    }
}
static void bta2dp_connection_state_callback(btav_connection_state_t state, bt_bdaddr_t* bd_addr) {
    ALOGD(LOGTAG " Connection State CB");
    BtEvent *pEvent = new BtEvent;
    memcpy(&pEvent->a2dpSinkEvent.bd_addr, bd_addr, sizeof(bt_bdaddr_t));
    switch( state ) {
        case BTAV_CONNECTION_STATE_DISCONNECTED:
            pEvent->a2dpSinkEvent.event_id = A2DP_SINK_DISCONNECTED_CB;
        break;
        case BTAV_CONNECTION_STATE_CONNECTING:
            pEvent->a2dpSinkEvent.event_id = A2DP_SINK_CONNECTING_CB;
        break;
        case BTAV_CONNECTION_STATE_CONNECTED:
            pEvent->a2dpSinkEvent.event_id = A2DP_SINK_CONNECTED_CB;
        break;
        case BTAV_CONNECTION_STATE_DISCONNECTING:
            pEvent->a2dpSinkEvent.event_id = A2DP_SINK_DISCONNECTING_CB;
        break;
    }
    PostMessage(THREAD_ID_A2DP_SINK, pEvent);
}

static void bta2dp_audio_state_callback(btav_audio_state_t state, bt_bdaddr_t* bd_addr) {
    ALOGD(LOGTAG " Audio State CB");
    BtEvent *pEvent = new BtEvent;
    memcpy(&pEvent->a2dpSinkEvent.bd_addr, bd_addr, sizeof(bt_bdaddr_t));
    switch( state ) {
        case BTAV_AUDIO_STATE_REMOTE_SUSPEND:
            pEvent->a2dpSinkEvent.event_id = A2DP_SINK_AUDIO_SUSPENDED;
        break;
        case BTAV_AUDIO_STATE_STOPPED:
            pEvent->a2dpSinkEvent.event_id = A2DP_SINK_AUDIO_STOPPED;
        break;
        case BTAV_AUDIO_STATE_STARTED:
            pEvent->a2dpSinkEvent.event_id = A2DP_SINK_AUDIO_STARTED;
        break;
    }
    PostMessage(THREAD_ID_A2DP_SINK, pEvent);
}

static void bta2dp_audio_config_callback(bt_bdaddr_t *bd_addr, uint32_t sample_rate,
        uint8_t channel_count) {
    ALOGD(LOGTAG " Audio Config CB sample_rate %d, channel_count %d", sample_rate, channel_count);
    if(pA2dpSink)
    {
        pA2dpSink->sample_rate = sample_rate;
        pA2dpSink->channel_count = channel_count;
    }
}
static void bta2dp_audio_focus_request_callback(bt_bdaddr_t *bd_addr) {
    ALOGD(LOGTAG " bta2dp_audio_focus_request_callback ");
    BtEvent *pEvent = new BtEvent;
    pEvent->a2dpSinkEvent.event_id = A2DP_SINK_FOCUS_REQUEST_CB;
    memcpy(&pEvent->a2dpSinkEvent.bd_addr, bd_addr, sizeof(bt_bdaddr_t));
    PostMessage(THREAD_ID_A2DP_SINK, pEvent);
}

static btav_callbacks_t sBluetoothA2dpSinkCallbacks = {
    sizeof(sBluetoothA2dpSinkCallbacks),
    bta2dp_connection_state_callback,
    bta2dp_audio_state_callback,
    bta2dp_audio_config_callback,
    NULL,
    NULL,
    bta2dp_audio_focus_request_callback,
};

static void btavrcpctrl_passthru_rsp_callback(int id, int key_state) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_passthru_rsp_callback id = %d key_state = %d", id, key_state);
}

static void btavrcpctrl_connection_state_callback(bool state, bt_bdaddr_t* bd_addr) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_connection_state_callback state = %d", state);
    BtEvent *pEvent = new BtEvent;
    memcpy(&pEvent->avrcpCtrlEvent.bd_addr, bd_addr, sizeof(bt_bdaddr_t));
    if (state == true)
        pEvent->avrcpCtrlEvent.event_id = AVRCP_CTRL_CONNECTED_CB;
    else
        pEvent->avrcpCtrlEvent.event_id = AVRCP_CTRL_DISCONNECTED_CB;
    PostMessage(THREAD_ID_A2DP_SINK, pEvent);
}

static void btavrcpctrl_rcfeatures_callback( bt_bdaddr_t* bd_addr, int features) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_rcfeatures_callback features = %d", features);
}

static void btavrcpctrl_getcap_rsp_callback( bt_bdaddr_t *bd_addr, int cap_id,
                uint32_t* supported_values, int num_supported, uint8_t rsp_type) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_getcap_rsp_callback");
}

static void btavrcpctrl_listplayerappsettingattrib_rsp_callback( bt_bdaddr_t *bd_addr,
                          uint8_t* supported_attribs, int num_attrib, uint8_t rsp_type) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_listplayerappsettingattrib_rsp_callback");
}

static void btavrcpctrl_listplayerappsettingvalue_rsp_callback( bt_bdaddr_t *bd_addr,
                       uint8_t* supported_val, uint8_t num_supported, uint8_t rsp_type) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_listplayerappsettingvalue_rsp_callback");
}

static void btavrcpctrl_currentplayerappsetting_rsp_callback( bt_bdaddr_t *bd_addr,
        uint8_t* supported_ids, uint8_t* supported_val, uint8_t num_attrib, uint8_t rsp_type) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_currentplayerappsetting_rsp_callback");
}

static void btavrcpctrl_setplayerappsetting_rsp_callback( bt_bdaddr_t *bd_addr,uint8_t rsp_type) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_setplayerappsetting_rsp_callback");
}

static void btavrcpctrl_notification_rsp_callback( bt_bdaddr_t *bd_addr, uint8_t rsp_type,
        int rsp_len, uint8_t* notification_rsp) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_notification_rsp_callback");
}

static void btavrcpctrl_getelementattrib_rsp_callback(bt_bdaddr_t *bd_addr, uint8_t num_attributes,
       int rsp_len, uint8_t* attrib_rsp, uint8_t rsp_type) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_getelementattrib_rsp_callback");
}

static void btavrcpctrl_getplaystatus_rsp_callback(bt_bdaddr_t *bd_addr, int param_len,
        uint8_t* play_status_rsp, uint8_t rsp_type) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_getplaystatus_rsp_callback");
}

static void btavrcpctrl_setabsvol_cmd_callback(bt_bdaddr_t *bd_addr, uint8_t abs_vol) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_setabsvol_cmd_callback");
}

static void btavrcpctrl_registernotification_absvol_callback(bt_bdaddr_t *bd_addr) {
    ALOGD(LOGTAG_CTRL " btavrcpctrl_registernotification_absvol_callback");
}

static btrc_ctrl_callbacks_t sBluetoothAvrcpCtrlCallbacks = {
   sizeof(sBluetoothAvrcpCtrlCallbacks),
   btavrcpctrl_passthru_rsp_callback,
   btavrcpctrl_connection_state_callback,
   btavrcpctrl_rcfeatures_callback,
   btavrcpctrl_getcap_rsp_callback,
   btavrcpctrl_listplayerappsettingattrib_rsp_callback,
   btavrcpctrl_listplayerappsettingvalue_rsp_callback,
   btavrcpctrl_currentplayerappsetting_rsp_callback,
   btavrcpctrl_setplayerappsetting_rsp_callback,
   btavrcpctrl_notification_rsp_callback,
   btavrcpctrl_getelementattrib_rsp_callback,
   btavrcpctrl_getplaystatus_rsp_callback,
   btavrcpctrl_setabsvol_cmd_callback,
   btavrcpctrl_registernotification_absvol_callback,
};

void A2dp_Sink::SendPassThruCommandNative(uint8_t key_id) {
    if (sBtAvrcpCtrlInterface != NULL) {
        sBtAvrcpCtrlInterface->send_pass_through_cmd(&mConnectedAvrcpDevice, key_id, 0);
        sBtAvrcpCtrlInterface->send_pass_through_cmd(&mConnectedAvrcpDevice, key_id, 1);
    }
}
void A2dp_Sink::HandleAvrcpEvents(BtEvent* pEvent) {
    ALOGD(LOGTAG_CTRL " HandleAvrcpEvents event = %s", dump_message(pEvent->avrcpCtrlEvent.event_id));
    switch(pEvent->avrcpCtrlEvent.event_id) {
    case AVRCP_CTRL_CONNECTED_CB:
        mAvrcpConnected = true;
        memcpy(&mConnectedAvrcpDevice, &pEvent->avrcpCtrlEvent.bd_addr,
                sizeof(bt_bdaddr_t));
        break;
    case AVRCP_CTRL_DISCONNECTED_CB:
        mAvrcpConnected = false;
        memset(&mConnectedAvrcpDevice, 0, sizeof(bt_bdaddr_t));
        break;
    case AVRCP_CTRL_PASS_THRU_CMD_REQ:
        if (!mAvrcpConnected || (memcmp(&mConnectedAvrcpDevice, &mConnectedDevice,
                                                          sizeof(bt_bdaddr_t)) != 0)) {
            ALOGD(LOGTAG_CTRL " Avrcp Not connected/ Not to A2DP Sink ");
            break;
        }
        SendPassThruCommandNative(pEvent->avrcpCtrlEvent.key_id);
        break;
    }
}

void A2dp_Sink::HandleEnableSink(void) {
    BtEvent *pEvent = new BtEvent;
    if (bluetooth_interface != NULL)
    {
        sBtA2dpSinkInterface = (btav_interface_t *)bluetooth_interface->
                get_profile_interface(BT_PROFILE_ADVANCED_AUDIO_SINK_ID);
        if (sBtA2dpSinkInterface == NULL)
        {
             pEvent->profile_start_event.event_id = PROFILE_EVENT_START_DONE;
             pEvent->profile_start_event.profile_id = PROFILE_ID_A2DP_SINK;
             pEvent->profile_start_event.status = false;
             PostMessage(THREAD_ID_GAP, pEvent);
             return;
        }
        change_state(STATE_DISCONNECTED);
        sBtA2dpSinkInterface->init(&sBluetoothA2dpSinkCallbacks, 1, 0);
        pEvent->profile_start_event.event_id = PROFILE_EVENT_START_DONE;
        pEvent->profile_start_event.profile_id = PROFILE_ID_A2DP_SINK;
        pEvent->profile_start_event.status = true;
        // AVRCP Initialization
        sBtAvrcpCtrlInterface = (btrc_ctrl_interface_t *)bluetooth_interface->
                get_profile_interface(BT_PROFILE_AV_RC_CTRL_ID);
        if (sBtAvrcpCtrlInterface != NULL) {
            sBtAvrcpCtrlInterface->init(&sBluetoothAvrcpCtrlCallbacks);
        }
        PostMessage(THREAD_ID_GAP, pEvent);
    }
}

void A2dp_Sink::HandleDisableSink(void) {
   change_state(STATE_NOT_STARTED);
   CloseAudioStream();
   StopPcmTimer();
   if(sBtA2dpSinkInterface != NULL) {
       sBtA2dpSinkInterface->cleanup();
       sBtA2dpSinkInterface = NULL;
   }
   if (sBtAvrcpCtrlInterface != NULL) {
       sBtAvrcpCtrlInterface->cleanup();
       sBtAvrcpCtrlInterface = NULL;
   }
   BtEvent *pEvent = new BtEvent;
    pEvent->profile_stop_event.event_id = PROFILE_EVENT_STOP_DONE;
        pEvent->profile_stop_event.profile_id = PROFILE_ID_A2DP_SINK;
        pEvent->profile_stop_event.status = true;
        PostMessage(THREAD_ID_GAP, pEvent);
}

void A2dp_Sink::ProcessEvent(BtEvent* pEvent) {
    switch(mSinkState) {
        case STATE_DISCONNECTED:
            state_disconnected_handler(pEvent);
            break;
        case STATE_PENDING:
            state_pending_handler(pEvent);
            break;
        case STATE_CONNECTED:
            state_connected_handler(pEvent);
            break;
        case STATE_NOT_STARTED:
            ALOGE(LOGTAG " STATE UNINITIALIZED, return");
            break;
    }
}

char* A2dp_Sink::dump_message(BluetoothEventId event_id) {
    switch(event_id) {
    case A2DP_SINK_API_CONNECT_REQ:
        return"API_CONNECT_REQ";
    case A2DP_SINK_API_DISCONNECT_REQ:
        return "API_DISCONNECT_REQ";
    case A2DP_SINK_DISCONNECTED_CB:
        return "DISCONNECTED_CB";
    case A2DP_SINK_CONNECTING_CB:
        return "CONNECING_CB";
    case A2DP_SINK_CONNECTED_CB:
        return "CONNECTED_CB";
    case A2DP_SINK_DISCONNECTING_CB:
        return "DISCONNECTING_CB";
    case A2DP_SINK_FOCUS_REQUEST_CB:
        return "FOCUS_REQUEST_CB";
    case A2DP_SINK_AUDIO_SUSPENDED:
        return "AUDIO_SUSPENDED_CB";
    case A2DP_SINK_AUDIO_STOPPED:
        return "AUDIO_STOPPED_CB";
    case A2DP_SINK_AUDIO_STARTED:
        return "AUDIO_STARTED_CB";
    case AVRCP_CTRL_CONNECTED_CB:
        return "AVRCP_CTRL_CONNECTED_CB";
    case AVRCP_CTRL_DISCONNECTED_CB:
        return "AVRCP_CTRL_DISCONNECTED_CB";
    case AVRCP_CTRL_PASS_THRU_CMD_REQ:
        return "PASS_THRU_CMD_REQ";
    case BT_AM_CONTROL_STATUS:
        return "AM_CONTROL_STATUS";
    case A2DP_SINK_FETCH_PCM_DATA:
        return "A2DP_SINK_FETCH_PCM_DATA";
    }
    return "UNKNOWN";
}

void A2dp_Sink::state_disconnected_handler(BtEvent* pEvent) {
    char str[18];
    ALOGD(LOGTAG "state_disconnected_handler Processing event %s", dump_message(pEvent->event_id));
    switch(pEvent->event_id) {
        case A2DP_SINK_API_CONNECT_REQ:
            memcpy(&mConnectingDevice, &pEvent->a2dpSinkEvent.bd_addr, sizeof(bt_bdaddr_t));
            if (sBtA2dpSinkInterface != NULL) {
                sBtA2dpSinkInterface->connect(&pEvent->a2dpSinkEvent.bd_addr);
            }
            change_state(STATE_PENDING);
            break;
        case A2DP_SINK_CONNECTING_CB:
            memcpy(&mConnectingDevice, &pEvent->a2dpSinkEvent.bd_addr, sizeof(bt_bdaddr_t));
            bdaddr_to_string(&mConnectingDevice, str, 18);
            cout << "A2DP Sink Connecting to " << str << endl;
            change_state(STATE_PENDING);
            break;
        case A2DP_SINK_CONNECTED_CB:
            memset(&mConnectingDevice, 0, sizeof(bt_bdaddr_t));
            memcpy(&mConnectedDevice, &pEvent->a2dpSinkEvent.bd_addr, sizeof(bt_bdaddr_t));
            bdaddr_to_string(&mConnectedDevice, str, 18);
            cout << "A2DP Sink Connected to " << str << endl;
            change_state(STATE_CONNECTED);
            break;
        default:
            ALOGD(LOGTAG " event not handled %d ", pEvent->event_id);
            break;
    }
}
void A2dp_Sink::state_pending_handler(BtEvent* pEvent) {
    char str[18];
    ALOGD(LOGTAG "state_pending_handler Processing event %s", dump_message(pEvent->event_id));
    switch(pEvent->event_id) {
        case A2DP_SINK_CONNECTING_CB:
            break;
        case A2DP_SINK_CONNECTED_CB:
            memcpy(&mConnectedDevice, &pEvent->a2dpSinkEvent.bd_addr, sizeof(bt_bdaddr_t));
            memset(&mConnectingDevice, 0, sizeof(bt_bdaddr_t));
            bdaddr_to_string(&mConnectedDevice, str, 18);
            cout << "A2DP Sink Connected to " << str << endl;
            change_state(STATE_CONNECTED);
            break;
        case A2DP_SINK_DISCONNECTED_CB:
            cout << "A2DP Sink DisConnected "<< endl;
            memset(&mConnectedDevice, 0, sizeof(bt_bdaddr_t));
            memset(&mConnectingDevice, 0, sizeof(bt_bdaddr_t));

            change_state(STATE_DISCONNECTED);
            break;
        case A2DP_SINK_API_CONNECT_REQ:
            bdaddr_to_string(&mConnectingDevice, str, 18);
            cout << "A2DP Sink Connecting to " << str << endl;
            break;
        default:
            ALOGD(LOGTAG " event not handled %d ", pEvent->event_id);
            break;
    }
}

void A2dp_Sink::state_connected_handler(BtEvent* pEvent) {
    char str[18];
    uint32_t pcm_data_read = 0;
    BtEvent *pControlRequest, *pReleaseControlReq;
    ALOGD(LOGTAG " state_connected_handler Processing event %s", dump_message(pEvent->event_id));
    switch(pEvent->event_id) {
        case A2DP_SINK_API_CONNECT_REQ:
            bdaddr_to_string(&mConnectedDevice, str, 18);
            cout << "A2DP Sink Connected to " << str << endl;
            break;
        case A2DP_SINK_API_DISCONNECT_REQ:
            CloseAudioStream();
            // release control
            pReleaseControlReq = new BtEvent;
            pReleaseControlReq->btamControlRelease.event_id = BT_AM_RELEASE_CONTROL;
            pReleaseControlReq->btamControlRelease.profile_id = PROFILE_ID_A2DP_SINK;
            PostMessage(THREAD_ID_BT_AM, pReleaseControlReq);

            bdaddr_to_string(&mConnectedDevice, str, 18);
            cout << "A2DP Sink DisConnecting from " << str << endl;
            memset(&mConnectedDevice, 0, sizeof(bt_bdaddr_t));
            memset(&mConnectingDevice, 0, sizeof(bt_bdaddr_t));
            if (sBtA2dpSinkInterface != NULL) {
                sBtA2dpSinkInterface->disconnect(&pEvent->a2dpSinkEvent.bd_addr);
            }
            change_state(STATE_PENDING);
            break;
        case A2DP_SINK_DISCONNECTED_CB:
            CloseAudioStream();
            // release control
            pReleaseControlReq = new BtEvent;
            pReleaseControlReq->btamControlRelease.event_id = BT_AM_RELEASE_CONTROL;
            pReleaseControlReq->btamControlRelease.profile_id = PROFILE_ID_A2DP_SINK;
            PostMessage(THREAD_ID_BT_AM, pReleaseControlReq);

            memset(&mConnectedDevice, 0, sizeof(bt_bdaddr_t));
            memset(&mConnectingDevice, 0, sizeof(bt_bdaddr_t));
            cout << "A2DP Sink DisConnected " << endl;
            change_state(STATE_DISCONNECTED);
            break;
        case A2DP_SINK_DISCONNECTING_CB:
            CloseAudioStream();
            // release control
            pReleaseControlReq = new BtEvent;
            pReleaseControlReq->btamControlRelease.event_id = BT_AM_RELEASE_CONTROL;
            pReleaseControlReq->btamControlRelease.profile_id = PROFILE_ID_A2DP_SINK;
            PostMessage(THREAD_ID_BT_AM, pReleaseControlReq);

            cout << "A2DP Sink DisConnecting " << endl;
            change_state(STATE_PENDING);
            break;
        case A2DP_SINK_AUDIO_STARTED:
        case A2DP_SINK_FOCUS_REQUEST_CB:
            pControlRequest = new BtEvent;
            pControlRequest->btamControlReq.event_id = BT_AM_REQUEST_CONTROL;
            pControlRequest->btamControlReq.profile_id = PROFILE_ID_A2DP_SINK;
            pControlRequest->btamControlReq.request_type = REQUEST_TYPE_PERMANENT;
            PostMessage(THREAD_ID_BT_AM, pControlRequest);
            break;
        case A2DP_SINK_FETCH_PCM_DATA:
           pcm_timer = false;
           // first start next timer
           StartPcmTimer();

            // fetch PCM data from fluoride
            if ((sBtA2dpSinkInterface != NULL) && ( pcm_buf != NULL)) {
                pcm_data_read =  sBtA2dpSinkInterface->get_pcm_data(pcm_buf, pcm_buf_size);
                ALOGD(LOGTAG " pcm_data_read = %d", pcm_data_read);
            }
#if (defined(BT_AUDIO_HAL_INTEGRATION))
            if ((pBTAM->GetAudioDevice() != NULL) && (out_stream != NULL)) {
                out_stream->write(out_stream, pcm_buf, pcm_data_read);
            }
#endif
#if (defined(DUMP_PCM_DATA) && (DUMP_PCM_DATA == TRUE))
           if ((outputPcmSampleFile) && (pcm_buf != NULL))
           {
              fwrite ((void*)pcm_buf, 1, (size_t)(pcm_data_read), outputPcmSampleFile);
           }
#endif
            break;
        case BT_AM_CONTROL_STATUS:
            ALOGD(LOGTAG "earlier status = %d  new status = %d", controlStatus,
                                                      pEvent->btamControlStatus.status_type);
            controlStatus = pEvent->btamControlStatus.status_type;
            switch(controlStatus) {
                case STATUS_LOSS:
                    // inform bluedroid
                    if (sBtA2dpSinkInterface != NULL) {
                        sBtA2dpSinkInterface->audio_focus_state(0);
                    }
                    // send pause to remote
                    SendPassThruCommandNative(CMD_ID_PAUSE);
                    // release control
                    pReleaseControlReq = new BtEvent;
                    pReleaseControlReq->btamControlRelease.event_id = BT_AM_RELEASE_CONTROL;
                    pReleaseControlReq->btamControlRelease.profile_id = PROFILE_ID_A2DP_SINK;
                    PostMessage(THREAD_ID_BT_AM, pReleaseControlReq);
                    CloseAudioStream();
                    StopPcmTimer();
                    break;
                case STATUS_LOSS_TRANSIENT:
                    // inform bluedroid
                    if (sBtA2dpSinkInterface != NULL) {
                        sBtA2dpSinkInterface->audio_focus_state(0);
                    }
                    // send pause to remote
                    SendPassThruCommandNative(CMD_ID_PAUSE);
                    CloseAudioStream();
                    StopPcmTimer();
                    break;
                case STATUS_GAIN:
                    // inform bluedroid
                    if (sBtA2dpSinkInterface != NULL) {
                        sBtA2dpSinkInterface->audio_focus_state(3);
                    }
                    ConfigureAudioHal();
                    StartPcmTimer();
                    break;
                case STATUS_REGAINED:
                    // inform bluedroid
                    if (sBtA2dpSinkInterface != NULL) {
                        sBtA2dpSinkInterface->audio_focus_state(3);
                    }
                    ConfigureAudioHal();
                    StartPcmTimer();
                    // send play to remote
                    SendPassThruCommandNative(CMD_ID_PLAY);
                    break;
            }
            break;
        case A2DP_SINK_AUDIO_SUSPENDED:
        case A2DP_SINK_AUDIO_STOPPED:
            // release focus in this case.
            CloseAudioStream();
            StopPcmTimer();
            if (controlStatus != STATUS_LOSS_TRANSIENT) {
                pReleaseControlReq = new BtEvent;
                pReleaseControlReq->btamControlRelease.event_id = BT_AM_RELEASE_CONTROL;
                pReleaseControlReq->btamControlRelease.profile_id = PROFILE_ID_A2DP_SINK;
                PostMessage(THREAD_ID_BT_AM, pReleaseControlReq);
            }
            break;
        default:
            ALOGD(LOGTAG " event not handled %d ", pEvent->event_id);
            break;
    }
}
void A2dp_Sink::ConfigureAudioHal() {
#if (defined BT_AUDIO_HAL_INTEGRATION)
    audio_hw_device_t* audio_device;
    audio_config_t config;
    audio_io_handle_t handle = 0x07;

    ALOGD(LOGTAG " sample_rate = %d, channel_count = %d", sample_rate, channel_count);
    if (!sample_rate || !channel_count) {
        return;
    }

    if (out_stream != NULL) {
        ALOGD(LOGTAG " HAL already configured ");
        return;
    }
    // HAL is not yet configured, configure it now.
   config.offload_info.size = sizeof(audio_offload_info_t);
   config.offload_info.sample_rate = sample_rate;
   config.offload_info.format = AUDIO_FORMAT_PCM_16_BIT;
   config.offload_info.version = AUDIO_OFFLOAD_INFO_VERSION_CURRENT;
   config.channel_mask = audio_channel_out_mask_from_count(channel_count);
   config.offload_info.channel_mask = audio_channel_out_mask_from_count(channel_count);
    if (pBTAM != NULL) {
        audio_device = pBTAM->GetAudioDevice();
        if(audio_device != NULL) {
            // 2 refers to speaker
            ALOGD(LOGTAG, " opening output stream ");
            audio_device->open_output_stream(audio_device, handle, 2, AUDIO_OUTPUT_FLAG_DIRECT_PCM,
                   &config, &out_stream, "bt_a2dp_sink");
        }
        if (out_stream != NULL) {
            pcm_buf_size = out_stream->common.get_buffer_size(&out_stream->common);
            ALOGD(LOGTAG " pcm buf size %d", pcm_buf_size);
            pcm_buf = (uint8_t*)osi_malloc(pcm_buf_size);
        }
    }
#endif
#if (defined(DUMP_PCM_DATA) && (DUMP_PCM_DATA == TRUE))
    if (!sample_rate || !channel_count) {
        return;
    }
    switch(sample_rate) {
    case 44100:
        pcm_buf_size = 7065;
        break;
    case 48000:
        pcm_buf_size = 7680;
        break;
    }
    pcm_buf = (uint8_t*)osi_malloc(pcm_buf_size);
    if (outputPcmSampleFile == NULL)
        outputPcmSampleFile = fopen(outputFilename, "ab");
#endif
}
void A2dp_Sink::CloseAudioStream() {
#if (defined BT_AUDIO_HAL_INTEGRATION)
    audio_hw_device_t* audio_device;
    if (pBTAM != NULL) {
        audio_device = pBTAM->GetAudioDevice();
        if((audio_device != NULL) && (out_stream != NULL)) {
            // 2 refers to speaker
            ALOGD(LOGTAG, " closing output stream ");
            audio_device->close_output_stream(audio_device, out_stream);
            out_stream = NULL;
        }
        if (pcm_buf != NULL) {
            osi_free(pcm_buf);
            pcm_buf = NULL;
        }
    }
#endif
#if (defined(DUMP_PCM_DATA) && (DUMP_PCM_DATA == TRUE))
    if (outputPcmSampleFile)
    {
        fclose(outputPcmSampleFile);
    }
    outputPcmSampleFile = NULL;
    if (pcm_buf != NULL) {
        osi_free(pcm_buf);
        pcm_buf = NULL;
    }
#endif
}
void A2dp_Sink::OnDisconnected() {
    ALOGD(LOGTAG " onDisconnected ");
    StopPcmTimer();
    CloseAudioStream();
}
void A2dp_Sink::change_state(A2dpSinkState mState) {
   ALOGD(LOGTAG " current State = %d, new state = %d", mSinkState, mState);
   pthread_mutex_lock(&lock);
   mSinkState = mState;
   if (mSinkState == STATE_DISCONNECTED)
   {
        OnDisconnected();
   }
   ALOGD(LOGTAG " state changes to %d ", mState);
   pthread_mutex_unlock(&lock);
}
A2dp_Sink :: A2dp_Sink(const bt_interface_t *bt_interface, config_t *config) {

    this->bluetooth_interface = bt_interface;
    this->config = config;
    sBtA2dpSinkInterface = NULL;
    sBtAvrcpCtrlInterface = NULL;
    mSinkState = STATE_NOT_STARTED;
    controlStatus = STATUS_LOSS;
    mAvrcpConnected = false;
    channel_count = 0;
    sample_rate = 0;
    memset(&mConnectedDevice, 0, sizeof(bt_bdaddr_t));
    memset(&mConnectingDevice, 0, sizeof(bt_bdaddr_t));
    memset(&mConnectedAvrcpDevice, 0, sizeof(bt_bdaddr_t));
    pthread_mutex_init(&this->lock, NULL);
    pcm_data_fetch_timer = alarm_new();
    pcm_buf = NULL;
    pcm_timer = false;
#if (defined BT_AUDIO_HAL_INTEGRATION)
    out_stream =  NULL;
#endif
#if (defined(DUMP_PCM_DATA) && (DUMP_PCM_DATA == TRUE))
    outputPcmSampleFile =  NULL;
#endif
}

A2dp_Sink :: ~A2dp_Sink() {
    pthread_mutex_destroy(&lock);
    mAvrcpConnected = false;
    controlStatus = STATUS_LOSS;
    alarm_free(pcm_data_fetch_timer);
    pcm_data_fetch_timer = NULL;
#if (defined BT_AUDIO_HAL_INTEGRATION)
    out_stream = NULL;
#endif
    if (pcm_buf != NULL) {
        osi_free(pcm_buf);
        pcm_buf = NULL;
    }
}
