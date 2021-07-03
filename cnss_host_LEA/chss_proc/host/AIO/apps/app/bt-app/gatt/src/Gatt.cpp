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

#include <list>
#include <map>
#include "osi/include/log.h"
#include "Gatt.hpp"
#include "Rsp.hpp"

#define LOGTAG "GATT "
#define UNUSED
#define MAX_NUM_HANDLES     (1)

using namespace std;
using std::list;
using std::string;

Gatt *g_gatt = NULL;
static const bt_bdaddr_t bd_addr_null={0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#ifdef __cplusplus
extern "C"
{
#endif

void btgattc_register_app_cb(int status, int clientIf, bt_uuid_t *app_uuid)
{
    ALOGD(LOGTAG "(%s) status (%d) client_if (%d)",__FUNCTION__,status, clientIf);

    BtEvent *event = new BtEvent;
    CHECK_PARAM_VOID(event)

    event->event_id = BTGATTC_REGISTER_APP_EVENT;
    event->gattc_register_app_event.status = status;
    event->gattc_register_app_event.clientIf = clientIf;

    PostMessage(THREAD_ID_GATT, event);
}

void btgattc_scan_result_cb(bt_bdaddr_t* bda, int rssi, uint8_t* adv_data)
{
    UNUSED
}

void btgattc_open_cb(int conn_id, int status, int clientIf, bt_bdaddr_t* bda)
{
    UNUSED
}

void btgattc_close_cb(int conn_id, int status, int clientIf, bt_bdaddr_t* bda)
{
    UNUSED
}

void btgattc_search_complete_cb(int conn_id, int status)
{
    UNUSED
}

void btgattc_search_result_cb(int conn_id, btgatt_srvc_id_t *srvc_id)
{
    UNUSED
}

void btgattc_get_characteristic_cb(int conn_id, int status,
                btgatt_srvc_id_t *srvc_id, btgatt_gatt_id_t *char_id,
                int char_prop)
{
    UNUSED
}

void btgattc_get_descriptor_cb(int conn_id, int status,
                btgatt_srvc_id_t *srvc_id, btgatt_gatt_id_t *char_id,
                btgatt_gatt_id_t *descr_id)
{
    UNUSED
}

void btgattc_get_included_service_cb(int conn_id, int status,
                btgatt_srvc_id_t *srvc_id, btgatt_srvc_id_t *incl_srvc_id)
{
    UNUSED
}

void btgattc_register_for_notification_cb(int conn_id, int registered,
                                                    int status, btgatt_srvc_id_t *srvc_id,
                                                    btgatt_gatt_id_t *char_id)
{
    UNUSED
}

void btgattc_notify_cb(int conn_id, btgatt_notify_params_t *p_data)
{
    UNUSED
}

void btgattc_read_characteristic_cb(int conn_id, int status,
    btgatt_read_params_t *p_data)
{
    UNUSED
}

void btgattc_write_characteristic_cb(int conn_id, int status,
    btgatt_write_params_t *p_data)
{
    UNUSED
}

void btgattc_execute_write_cb(int conn_id, int status)
{
    UNUSED
}

void btgattc_read_descriptor_cb(int conn_id, int status, btgatt_read_params_t *p_data)
{
    UNUSED
}

void btgattc_write_descriptor_cb(int conn_id, int status, btgatt_write_params_t *p_data)
{
    UNUSED
}

void btgattc_remote_rssi_cb(int client_if,bt_bdaddr_t* bda, int rssi, int status)
{
    UNUSED
}

void btgattc_advertise_cb(int status, int client_if)
{
    fprintf(stdout,"(%s): status (%d) client_if (%d)\n",__FUNCTION__,status, client_if);
}

void btgattc_configure_mtu_cb(int conn_id, int status, int mtu)
{
    UNUSED
}

void btgattc_scan_filter_cfg_cb(int action, int client_if, int status, int filt_type, int avbl_space)
{
    UNUSED
}

void btgattc_scan_filter_param_cb(int action, int client_if, int status, int avbl_space)
{
    UNUSED
}

void btgattc_scan_filter_status_cb(int action, int client_if, int status)
{
    UNUSED
}

void btgattc_multiadv_enable_cb(int client_if, int status)
{
    UNUSED
}

void btgattc_multiadv_update_cb(int client_if, int status)
{
    UNUSED
}

void btgattc_multiadv_setadv_data_cb(int client_if, int status)
{
    UNUSED
}

void btgattc_multiadv_disable_cb(int client_if, int status)
{
    UNUSED
}

void btgattc_congestion_cb(int conn_id, bool congested)
{
    UNUSED
}

void btgattc_batchscan_cfg_storage_cb(int client_if, int status)
{
    UNUSED
}
void btgattc_batchscan_startstop_cb(int startstop_action, int client_if, int status)
{
    UNUSED

}

void btgattc_batchscan_reports_cb(int client_if, int status, int report_format,
                                            int num_records, int data_len, uint8_t *p_rep_data)
{
    UNUSED
}

void btgattc_batchscan_threshold_cb(int client_if)
{
    UNUSED
}

void btgattc_track_adv_event_cb(btgatt_track_adv_info_t *p_adv_track_info)
{
    UNUSED
}

void btgattc_scan_parameter_setup_completed_cb(int client_if, btgattc_error_t status)
{
    UNUSED
}

static const btgatt_client_callbacks_t sGattClientCallbacks = {
    btgattc_register_app_cb,
    btgattc_scan_result_cb,
    btgattc_open_cb,
    btgattc_close_cb,
    btgattc_search_complete_cb,
    btgattc_search_result_cb,
    btgattc_get_characteristic_cb,
    btgattc_get_descriptor_cb,
    btgattc_get_included_service_cb,
    btgattc_register_for_notification_cb,
    btgattc_notify_cb,
    btgattc_read_characteristic_cb,
    btgattc_write_characteristic_cb,
    btgattc_read_descriptor_cb,
    btgattc_write_descriptor_cb,
    btgattc_execute_write_cb,
    btgattc_remote_rssi_cb,
    btgattc_advertise_cb,
    btgattc_configure_mtu_cb,
    btgattc_scan_filter_cfg_cb,
    btgattc_scan_filter_param_cb,
    btgattc_scan_filter_status_cb,
    btgattc_multiadv_enable_cb,
    btgattc_multiadv_update_cb,
    btgattc_multiadv_setadv_data_cb,
    btgattc_multiadv_disable_cb,
    btgattc_congestion_cb,
    btgattc_batchscan_cfg_storage_cb,
    btgattc_batchscan_startstop_cb,
    btgattc_batchscan_reports_cb,
    btgattc_batchscan_threshold_cb,
    btgattc_track_adv_event_cb,
    btgattc_scan_parameter_setup_completed_cb
};

/**
 * GATT Server Callback Implementation
 */
void btgatts_register_app_cb(int status, int server_if, bt_uuid_t *uuid)
{
    ALOGD(LOGTAG "(%s) status (%d) server_if (%d)",__FUNCTION__, status, server_if);

    BtEvent *event = new BtEvent;
    CHECK_PARAM_VOID(event)

    event->event_id = BTGATTS_REGISTER_APP_EVENT;
    event->gatts_register_app_event.status = status;
    event->gatts_register_app_event.server_if = server_if;
    event->gatts_register_app_event.uuid = uuid;
    PostMessage(THREAD_ID_GATT, event);
}

void btgatts_connection_cb(int conn_id, int server_if, int connected, bt_bdaddr_t *bda)
{

    char c_address[32];
    sprintf(c_address, "%02X:%02X:%02X:%02X:%02X:%02X",
            bda->address[0], bda->address[1], bda->address[2],
            bda->address[3], bda->address[4], bda->address[5]);

    fprintf(stdout, "(%s) connid (%d) server_if (%d) status (%d) bda (%s)\n",__FUNCTION__, conn_id,
            server_if, connected, c_address);

    BtEvent *event = new BtEvent;
    CHECK_PARAM_VOID(event)

    event->event_id = BTGATTS_CONNECTION_EVENT;
    event->gatts_connection_event.conn_id = conn_id;
    event->gatts_connection_event.server_if = server_if;
    event->gatts_connection_event.connected = connected;
    event->gatts_connection_event.bda = bda;
    PostMessage(THREAD_ID_GATT, event);

}

void btgatts_service_added_cb(int status, int server_if,
                              btgatt_srvc_id_t *srvc_id, int srvc_handle)
{
    ALOGD (LOGTAG "(%s) status (%d) server_if (%d), srvc_handle(%d))",__FUNCTION__,
                status, server_if, srvc_handle);

    BtEvent *event = new BtEvent;
    CHECK_PARAM_VOID(event)

    event->event_id = BTGATTS_SERVICE_ADDED_EVENT;
    event->gatts_service_added_event.status = status;
    event->gatts_service_added_event.server_if = server_if;
    event->gatts_service_added_event.srvc_id = srvc_id ;
    event->gatts_service_added_event.srvc_handle = srvc_handle ;

    PostMessage(THREAD_ID_GATT, event);

}

void btgatts_included_service_added_cb(int status, int server_if, int srvc_handle,
        int incl_srvc_handle)
{
    ALOGD (LOGTAG "(%s) Status:(%d)",__FUNCTION__, status);

    BtEvent *event = new BtEvent;
    CHECK_PARAM_VOID(event)

    event->event_id = BTGATTS_INCLUDED_SERVICE_ADDED_EVENT;
    event->gatts_included_service_added_event.status = status;
    event->gatts_included_service_added_event.server_if = server_if;
    event->gatts_included_service_added_event.srvc_handle = srvc_handle ;
    event->gatts_included_service_added_event.incl_srvc_handle = incl_srvc_handle ;

    PostMessage(THREAD_ID_GATT, event);
}

void btgatts_characteristic_added_cb(int status, int server_if, bt_uuid_t *char_id,
                                                int srvc_handle, int char_handle)
{
    ALOGD (LOGTAG "(%s) Status:(%d)",__FUNCTION__, status);

    BtEvent *event = new BtEvent;
    CHECK_PARAM_VOID(event)

    event->event_id = BTGATTS_CHARACTERISTIC_ADDED_EVENT;
    event->gatts_characteristic_added_event.status = status;
    event->gatts_characteristic_added_event.server_if = server_if;
    event->gatts_characteristic_added_event.char_id = char_id;
    event->gatts_characteristic_added_event.srvc_handle = srvc_handle ;
    event->gatts_characteristic_added_event.char_handle = char_handle ;

    PostMessage(THREAD_ID_GATT, event);
}

void btgatts_descriptor_added_cb(int status, int server_if, bt_uuid_t *descr_id,
                                            int srvc_handle, int descr_handle)
{
    ALOGD (LOGTAG "(%s) Status:(%d)",__FUNCTION__, status);

    BtEvent *event = new BtEvent;
    CHECK_PARAM_VOID(event)

    event->event_id = BTGATTS_DESCRIPTOR_ADDED_EVENT;
    event->gatts_descriptor_added_event.status = status;
    event->gatts_descriptor_added_event.server_if = server_if;
    event->gatts_descriptor_added_event.descr_id  = descr_id ;
    event->gatts_descriptor_added_event.srvc_handle = srvc_handle ;
    event->gatts_descriptor_added_event.descr_handle = descr_handle ;

    PostMessage(THREAD_ID_GATT, event);
}

void btgatts_service_started_cb(int status, int server_if, int srvc_handle)
{
    ALOGD (LOGTAG "(%s) Status:(%d)",__FUNCTION__, status);

    BtEvent *event = new BtEvent;
    CHECK_PARAM_VOID(event)

    event->event_id = BTGATTS_SERVICE_STARTED_EVENT;
    event->gatts_service_started_event.status = status;
    event->gatts_service_started_event.server_if = server_if;
    event->gatts_service_started_event.srvc_handle = srvc_handle ;

    PostMessage(THREAD_ID_GATT, event);
}

void btgatts_service_stopped_cb(int status, int server_if, int srvc_handle)
{
    ALOGD (LOGTAG "(%s) Status:(%d)",__FUNCTION__, status);

    BtEvent *event = new BtEvent;
    CHECK_PARAM_VOID(event)

    event->event_id = BTGATTS_SERVICE_STOPPED_EVENT;
    event->gatts_service_stopped_event.status = status;
    event->gatts_service_stopped_event.server_if = server_if;
    event->gatts_service_stopped_event.srvc_handle = srvc_handle ;

    PostMessage(THREAD_ID_GATT, event);
}

void btgatts_service_deleted_cb(int status, int server_if, int srvc_handle)
{
    ALOGD (LOGTAG "(%s) Status:(%d)",__FUNCTION__, status);

    BtEvent *event = new BtEvent;
    CHECK_PARAM_VOID(event)

    event->event_id = BTGATTS_SERVICE_DELETED_EVENT;
    event->gatts_service_deleted_event.status = status;
    event->gatts_service_deleted_event.server_if = server_if;
    event->gatts_service_deleted_event.srvc_handle = srvc_handle ;

    PostMessage(THREAD_ID_GATT, event);
}

void btgatts_request_read_cb(int conn_id, int trans_id, bt_bdaddr_t *bda, int attr_handle,
                                        int offset, bool is_long)
{
    char c_address[32];
    sprintf(c_address, "%02X:%02X:%02X:%02X:%02X:%02X",
            bda->address[0], bda->address[1], bda->address[2],
            bda->address[3], bda->address[4], bda->address[5]);

    ALOGD (LOGTAG "(%s) connid:(%d) bda (%s)",__FUNCTION__, conn_id, c_address);

    BtEvent *event = new BtEvent;
    CHECK_PARAM_VOID(event)

    event->event_id = BTGATTS_REQUEST_READ_EVENT;
    event->gatts_request_read_event.conn_id = conn_id;
    event->gatts_request_read_event.trans_id = trans_id;
    event->gatts_request_read_event.bda = bda;
    event->gatts_request_read_event.attr_handle = attr_handle;
    event->gatts_request_read_event.offset = offset;
    event->gatts_request_read_event.is_long = is_long ;

    PostMessage(THREAD_ID_GATT, event);

}

void btgatts_request_write_cb(int conn_id, int trans_id, bt_bdaddr_t *bda, int attr_handle,
                                        int offset, int length, bool need_rsp, bool is_prep,
                                        uint8_t* value)
{
    char c_address[32];
    sprintf(c_address, "%02X:%02X:%02X:%02X:%02X:%02X",
            bda->address[0], bda->address[1], bda->address[2],
           bda->address[3], bda->address[4], bda->address[5]);

    fprintf(stdout, "(%s) connid:(%d) bdaddr:(%s) value (%s) need_rsp (%d)\n",__FUNCTION__, conn_id,
            c_address, value, need_rsp);

    BtEvent *event = new BtEvent;
    CHECK_PARAM_VOID(event)

    event->event_id = BTGATTS_REQUEST_WRITE_EVENT;
    event->gatts_request_write_event.conn_id = conn_id;
    event->gatts_request_write_event.trans_id = trans_id;
    event->gatts_request_write_event.bda = bda;
    event->gatts_request_write_event.attr_handle = attr_handle;
    event->gatts_request_write_event.offset = offset;
    event->gatts_request_write_event.length = length;
    event->gatts_request_write_event.need_rsp = need_rsp;
    event->gatts_request_write_event.is_prep = is_prep;
    event->gatts_request_write_event.value = value;

    PostMessage(THREAD_ID_GATT, event);
}

void btgatts_request_exec_write_cb(int conn_id, int trans_id,
                                                bt_bdaddr_t *bda, int exec_write)
{

    char c_address[32];
    sprintf(c_address, "%02X:%02X:%02X:%02X:%02X:%02X",
            bda->address[0], bda->address[1], bda->address[2],
           bda->address[3], bda->address[4], bda->address[5]);

    ALOGD (LOGTAG "(%s) connid:(%d) bda:(%s)",__FUNCTION__, conn_id, c_address);
    BtEvent *event = new BtEvent;
    CHECK_PARAM_VOID(event)

    event->event_id = BTGATTS_REQUEST_EXEC_WRITE_EVENT;
    event->gatts_request_exec_write_event.conn_id = conn_id;
    event->gatts_request_exec_write_event.trans_id = trans_id;
    event->gatts_request_exec_write_event.bda = bda;
    event->gatts_request_exec_write_event.exec_write = exec_write;

    PostMessage(THREAD_ID_GATT, event);
}

void btgatts_response_confirmation_cb(int status, int handle)
{
    ALOGD (LOGTAG "(%s) status:(%d) handle:(%d)",__FUNCTION__, status, handle);

    BtEvent *event = new BtEvent;
    CHECK_PARAM_VOID(event)

    event->event_id = BTGATTS_RESPONSE_CONFIRMATION_EVENT;
    event->gatts_response_confirmation_event.status = status;
    event->gatts_response_confirmation_event.handle = handle ;

    PostMessage(THREAD_ID_GATT, event);

}

void btgatts_indication_sent_cb(int conn_id, int status)
{
    ALOGD (LOGTAG "(%s) conn_id (%d) status:(%d)",__FUNCTION__, conn_id, status);

    BtEvent *event = new BtEvent;
    CHECK_PARAM_VOID(event)

    event->event_id = BTGATTS_INDICATION_SENT_EVENT;
    event->gatts_indication_sent_event.status = status;
    event->gatts_indication_sent_event.conn_id = conn_id ;

    PostMessage(THREAD_ID_GATT, event);
}

void btgatts_congestion_cb(int conn_id, bool congested)
{
    ALOGD (LOGTAG "(%s) contested (%d) conn_id:(%d)",__FUNCTION__, congested,conn_id);

    BtEvent *event = new BtEvent;
    CHECK_PARAM_VOID(event)

    event->event_id = BTGATTS_CONGESTION_EVENT;
    event->gatts_congestion_event.congested = congested;
    event->gatts_congestion_event.conn_id = conn_id ;

    PostMessage(THREAD_ID_GATT, event);
}

void btgatts_mtu_changed_cb(int conn_id, int mtu)
{
    ALOGD (LOGTAG "(%s) conn_id:(%d) Mtu (%d)",__FUNCTION__, conn_id, mtu);

    BtEvent *event = new BtEvent;
    CHECK_PARAM_VOID(event)

    event->event_id = BTGATTS_MTU_CHANGED_EVENT;
    event->gatts_mtu_changed_event.conn_id = conn_id;
    event->gatts_mtu_changed_event.mtu = mtu ;

    PostMessage(THREAD_ID_GATT, event);

}

static const btgatt_server_callbacks_t sGattServerCallbacks = {
    btgatts_register_app_cb,
    btgatts_connection_cb,
    btgatts_service_added_cb,
    btgatts_included_service_added_cb,
    btgatts_characteristic_added_cb,
    btgatts_descriptor_added_cb,
    btgatts_service_started_cb,
    btgatts_service_stopped_cb,
    btgatts_service_deleted_cb,
    btgatts_request_read_cb,
    btgatts_request_write_cb,
    btgatts_request_exec_write_cb,
    btgatts_response_confirmation_cb,
    btgatts_indication_sent_cb,
    btgatts_congestion_cb,
    btgatts_mtu_changed_cb
};
/**
 * GATT callbacks
 */
static btgatt_callbacks_t sGattCallbacks = {
    sizeof(btgatt_callbacks_t),
    &sGattClientCallbacks,
    &sGattServerCallbacks
};

void BtGattMsgHandler(void *msg) {
    BtEvent* event = NULL;
    bool status = false;
    if(!msg)
    {
        ALOGE("(%s) Msg is null, return.\n",__FUNCTION__);
        return;
    }
    event = ( BtEvent *) msg;
    ALOGD (LOGTAG "(%s) event id (%d)",__FUNCTION__, (int) event->event_id);
    switch(event->event_id) {
        case PROFILE_API_START:
            if (g_gatt) {
                status = g_gatt->HandleEnableGatt();
                BtEvent *start_event = new BtEvent;
                CHECK_PARAM_VOID (start_event)

                start_event->profile_start_event.event_id = PROFILE_EVENT_START_DONE;
                start_event->profile_start_event.profile_id = PROFILE_ID_GATT;
                start_event->profile_start_event.status = status;
                PostMessage(THREAD_ID_GAP, start_event);
            }
            break;
       case PROFILE_API_STOP:
            if (g_gatt) {
                status = g_gatt->HandleDisableGatt();
                BtEvent *stop_event = new BtEvent;
                CHECK_PARAM_VOID (stop_event)

                stop_event->profile_start_event.event_id = PROFILE_EVENT_STOP_DONE;
                stop_event->profile_start_event.profile_id = PROFILE_ID_GATT;
                stop_event->profile_start_event.status = status;
                PostMessage(THREAD_ID_GAP, stop_event);
            }
            break;
        default:
            if(g_gatt) {
                ALOGD (LOGTAG "(%s) Received Message %d",__FUNCTION__, (int) event->event_id);
                g_gatt->ProcessEvent(( BtEvent *) msg);
            }
            break;
    }
    delete event;
}
#ifdef __cplusplus
}
#endif

void Gatt::HandleGattIpcMsg(BtIpcMsg *ipc_msg)
{

    CHECK_PARAM_VOID(ipc_msg)
    fprintf(stdout, "(%s) ipcMsg->type: %d, ipcMsg->status = %d wlan state (%d)\n",__FUNCTION__,
            ipc_msg->type, ipc_msg->status, rsp->GetDeviceState());

    if ((le_supported_profiles & REMOTE_START_PROFILE) && rsp)
    {
        int status = 0; //invalid. (1,,2,3 are valid states)
        if (rsp->GetDeviceState() == WLAN_TRANSACTION_PENDING)
        {
            if (ipc_msg->type == BT_IPC_REMOTE_START_WLAN )
            {
                (ipc_msg->status == 0) ? status = WLAN_ACTIVE: status = WLAN_INACTIVE;
            }
            rsp->SetDeviceState(status);
        }
    }
}

void Gatt::HandleGattcRegisterAppEvent(GattcRegisterAppEvent *event)
{
    ALOGD(LOGTAG  "(%s) client_if =%d status =%d uuid =%x",__FUNCTION__, event->clientIf,
            event->status, event->app_uuid);
    if ((le_supported_profiles & REMOTE_START_PROFILE) && rsp)
    {
        rsp->SetRSPClientAppData(event);
        if (event->status == BT_STATUS_SUCCESS)
        {
            rsp->ClientSetAdvData("Remote Start Profile");
            rsp->StartAdvertisement();
        } else {
            ALOGE (LOGTAG "(%s) Failed to Register Client App",__FUNCTION__, event->clientIf);
        }
    }
}

void Gatt::HandleGattsRegisterAppEvent(GattsRegisterAppEvent *event)
{
    ALOGD(LOGTAG  "(%s) server_if =%d status =%d uuid =%x",__FUNCTION__, event->server_if,
            event->status, event->uuid->uu);

    if ((le_supported_profiles & REMOTE_START_PROFILE) && rsp)
    {
        rsp->SetRSPAppData(event);
        if(event->status == BT_STATUS_SUCCESS)
        {
            rsp->AddService();
        } else {
        ALOGE (LOGTAG "(%s) Failed to Register Server App",__FUNCTION__, event->server_if);
        }
    }
}

void Gatt::HandleGattsConnectionEvent(GattsConnectionEvent *event)
{
    fprintf(stdout,"(%s) conn_id (%d) server_if (%d) connected (%d)\n",__FUNCTION__, event->conn_id,
            event->server_if, event->connected);

    char c_address[32];
    sprintf(c_address, "%02X:%02X:%02X:%02X:%02X:%02X",
                event->bda->address[0], event->bda->address[1], event->bda->address[2],
                event->bda->address[3], event->bda->address[4], event->bda->address[5]);

    if ((le_supported_profiles & REMOTE_START_PROFILE) && rsp)
    {
        rsp->SetRSPConnectionData(event);
        if (event->connected)
        {
            rsp->StopAdvertisement();
        }
    }
}

void Gatt::HandleGattsServiceAddedEvent(GattsServiceAddedEvent *event)
{
    ALOGD(LOGTAG  "(%s) event_id =%d status =%d server_if =%d,service_handle =%d",__FUNCTION__,
            event->event_id, event->status, event->server_if,event->srvc_handle);

    if ((le_supported_profiles & REMOTE_START_PROFILE) && rsp)
    {
        rsp->SetRSPSrvcData(event);
        if (event->status == BT_STATUS_SUCCESS)
        {
            rsp->AddCharacteristics();
        } else {
            ALOGE (LOGTAG "(%s) Failed to Add_Service",__FUNCTION__, event->server_if);
        }
    }
}

void Gatt::HandleGattsCharacteristicAddedEvent(GattsCharacteristicAddedEvent *event)
{
    ALOGD(LOGTAG  "(%s) char_handle =%d char_id =%d status =%d server_if =%d,service_handle =%d",
            __FUNCTION__, event->char_handle, event->char_id, event->status,
            event->server_if, event->srvc_handle);

    if ((le_supported_profiles & REMOTE_START_PROFILE) && rsp)
    {
        rsp->SetRSPCharacteristicData(event);
        if (event->status == BT_STATUS_SUCCESS)
        {
            rsp->AddDescriptor();
        } else {
            ALOGE (LOGTAG "(%s) Failed to Add Characteristics",__FUNCTION__, event->server_if);
        }
    }
}

void Gatt::HandleGattsDescriptorAddedEvent(GattsDescriptorAddedEvent *event)
{
    ALOGD(LOGTAG  "(%s) desc_handle =%d desc_id =%d status =%d server_if =%d, srvc_handle=%x,"
            "service_handle =%d",__FUNCTION__, event->descr_handle, event->descr_id,
            event->status, event->server_if,event->srvc_handle);

    if ((le_supported_profiles & REMOTE_START_PROFILE) && rsp)
    {
        rsp->SetRSPDescriptorData(event);
        if (event->status == BT_STATUS_SUCCESS)
        {
            rsp->StartService();
        } else {
            ALOGE (LOGTAG "(%s) Failed to Add Descriptor",__FUNCTION__, event->server_if);
        }
    }
}
void Gatt::HandleGattsServiceStartedEvent(GattsServiceStartedEvent *event)
{
    fprintf(stdout, "(%s) status =%d server_if =%d,service_handle =%d\n",__FUNCTION__,
            event->status, event->server_if, event->srvc_handle);

    if ((le_supported_profiles & REMOTE_START_PROFILE) && rsp)
    {
        if (event->status == BT_STATUS_SUCCESS)
        {
            rsp->RegisterClient();
        } else {
            ALOGE (LOGTAG "(%s) Failed to Start RSP Service",__FUNCTION__, event->server_if);
        }
    }
}

void Gatt::HandleGattsServiceStoppedEvent(GattsServiceStoppedEvent *event)
{
    ALOGD(LOGTAG  "Handler :(%s) status(%d) server_if(%d) srvc_handle(%d)",__FUNCTION__,
            event->status, event->server_if, event->srvc_handle);
    if ((le_supported_profiles & REMOTE_START_PROFILE) && rsp)
    {
        if (!event->status)
            rsp->DeleteService();
    }
    ALOGD(LOGTAG  "RSP Service stopped successfully, deleting the service");
}

void Gatt::HandleGattsServiceDeletedEvent(GattsServiceDeletedEvent *event)
{
    ALOGD(LOGTAG  "Handler :(%s) status(%d) server_if(%d) srvc_handle(%d)",__FUNCTION__,
            event->status, event->server_if, event->srvc_handle);

    if ((le_supported_profiles & REMOTE_START_PROFILE) && rsp)
    {
        if (!event->status)
        {
            rsp->CleanUp(event->server_if);
            delete rsp;
            rsp = NULL;
        }
    }
    fprintf(stdout,"RSP Service stopped & Unregistered successfully\n");
}

void Gatt::HandleGattsRequestWriteEvent(GattsRequestWriteEvent *event)
{
    char c_address[32];
    sprintf(c_address, "%02X:%02X:%02X:%02X:%02X:%02X",
                event->bda->address[0], event->bda->address[1], event->bda->address[2],
                event->bda->address[3], event->bda->address[4], event->bda->address[5]);

    ALOGD(LOGTAG "(%s) conn_id :(%d) bda: (%s)",__FUNCTION__, event->conn_id, c_address);
    if ((le_supported_profiles & REMOTE_START_PROFILE) && rsp)
    {
        rsp->SendResponse(event);
    }
}

void Gatt::HandleRspEnableEvent(RspEnableEvent *event)
{
    if ((le_supported_profiles & REMOTE_START_PROFILE) && rsp)
    {
        rsp->SetRSPAttrData(event);
        rsp->RegisterApp();
    }
}

void Gatt::HandleRspDisableEvent(RspDisableEvent *event)
{
    fprintf(stdout, "(%s) server_if (%d)\n",__FUNCTION__, event->server_if);

    if ((le_supported_profiles & REMOTE_START_PROFILE) && rsp)
    {
        rsp->StopService();
    }
}

void Gatt::ProcessEvent(BtEvent* event)
{
    CHECK_PARAM_VOID(event)
    ALOGD(LOGTAG "(%s) Processing event %d",__FUNCTION__, event->event_id);

    switch(event->event_id) {
        case SKT_API_IPC_MSG_READ:
            HandleGattIpcMsg((BtIpcMsg *)(&event->bt_ipc_msg_event.ipc_msg));
            break;
        case RSP_ENABLE_EVENT:
            HandleRspEnableEvent((RspEnableEvent *)event);
            break;
        case RSP_DISABLE_EVENT:
            HandleRspDisableEvent((RspDisableEvent *)event);
            break;
        case BTGATTS_REGISTER_APP_EVENT:
            HandleGattsRegisterAppEvent((GattsRegisterAppEvent *)event);
            break;
        case BTGATTS_CONNECTION_EVENT:
            HandleGattsConnectionEvent((GattsConnectionEvent *)event);
            break;
        case BTGATTS_SERVICE_ADDED_EVENT:
            HandleGattsServiceAddedEvent((GattsServiceAddedEvent *)event);
            break;
        case BTGATTS_CHARACTERISTIC_ADDED_EVENT:
            HandleGattsCharacteristicAddedEvent((GattsCharacteristicAddedEvent *)event);
            break;
        case BTGATTS_DESCRIPTOR_ADDED_EVENT:
            HandleGattsDescriptorAddedEvent((GattsDescriptorAddedEvent *)event);
            break;
        case BTGATTS_SERVICE_STARTED_EVENT:
            HandleGattsServiceStartedEvent((GattsServiceStartedEvent *)event);
            break;
        case BTGATTS_SERVICE_STOPPED_EVENT:
            HandleGattsServiceStoppedEvent((GattsServiceStoppedEvent *)event);
            break;
        case BTGATTS_SERVICE_DELETED_EVENT:
            HandleGattsServiceDeletedEvent((GattsServiceDeletedEvent *)event);
            break;
        case BTGATTS_REQUEST_WRITE_EVENT:
            HandleGattsRequestWriteEvent((GattsRequestWriteEvent *)event);
            break;
        case BTGATTC_REGISTER_APP_EVENT:
            HandleGattcRegisterAppEvent((GattcRegisterAppEvent *) event);
            break;
        case BTGATTS_REQUEST_READ_EVENT:
        case BTGATTS_INCLUDED_SERVICE_ADDED_EVENT:
        case BTGATTS_REQUEST_EXEC_WRITE_EVENT:
        case BTGATTS_RESPONSE_CONFIRMATION_EVENT:
        case BTGATTS_INDICATION_SENT_EVENT:
        case BTGATTS_CONGESTION_EVENT:
        case BTGATTS_MTU_CHANGED_EVENT:
        case BTGATTC_SCAN_RESULT_EVENT:
        case BTGATTC_OPEN_EVENT:
        case BTGATTC_CLOSE_EVENT:
        case BTGATTC_SEARCH_COMPLETE_EVENT:
        case BTGATTC_SEARCH_RESULT_EVENT:
        case BTGATTC_GET_CHARACTERISTIC_EVENT:
        case BTGATTC_GET_DESCRIPTOR_EVENT:
        case BTGATTC_GET_INCLUDED_SERVICE_EVENT:
        case BTGATTC_REGISTER_FOR_NOTIFICATION_EVENT:
        case BTGATTC_NOTIFY_EVENT:
        case BTGATTC_READ_CHARACTERISTIC_EVENT:
        case BTGATTC_WRITE_CHARACTERISTIC_EVENT:
        case BTGATTC_READ_DESCRIPTOR_EVENT:
        case BTGATTC_WRITE_DESCRIPTOR_EVENT:
        case BTGATTC_EXECUTE_WRITE_EVENT:
        case BTGATTC_REMOTE_RSSI_EVENT:
        case BTGATTC_ADVERTISE_EVENT:
        case BTGATTC_CONFIGURE_MTU_EVENT:
        case BTGATTC_SCAN_FILTER_CFG_EVENT:
        case BTGATTC_SCAN_FILTER_PARAM_EVENT:
        case BTGATTC_SCAN_FILTER_STATUS_EVENT:
        case BTGATTC_MULTIADV_ENABLE_EVENT:
        case BTGATTC_MULTIADV_UPDATE_EVENT:
        case BTGATTC_MULTIADV_SETADV_DATA_EVENT:
        case BTGATTC_MULTIADV_DISABLE_EVENT:
        case BTGATTC_CONGESTION_EVENT:
        case BTGATTC_BATCHSCAN_CFG_STORAGE_EVENT:
        case BTGATTC_BATCHSCAN_STARTSTOP_EVENT:
        case BTGATTC_BATCHSCAN_REPORTS_EVENT:
        case BTGATTC_BATCHSCAN_THRESHOLD_EVENT:
        case BTGATTC_TRACK_ADV_EVENT_EVENT:
        case BTGATTC_SCAN_PARAMETER_SETUP_COMPLETED_EVENT:
        default: //All fall-through, enable as needed
            ALOGD(LOGTAG  "(%s) Unhandled Event(%d)",__FUNCTION__, event->event_id);
            break;
    }
}

bool Gatt::GattInterfaceInit(const bt_interface_t *bt_interface)
{
    gatt_interface = (btgatt_interface_t*) bt_interface->get_profile_interface(BT_PROFILE_GATT_ID);
    if (gatt_interface == NULL)
    {
        ALOGE(LOGTAG "(%s) Failed to init gatt profile",__FUNCTION__);
        return false;
    }
    bt_status_t status = gatt_interface->init(&sGattCallbacks);
    return (status != BT_STATUS_SUCCESS ? false : true);
}

void Gatt::GattInterfaceCleanup()
{
    if (gatt_interface) {
        gatt_interface->cleanup();
        gatt_interface = NULL;
    }
}

Gatt::Gatt(const bt_interface_t *bt_interface, config_t *config)
{
    ALOGD(LOGTAG "(%s) Starting Up Gatt Instance",__FUNCTION__);
    this->gatt_interface = NULL;
    this->bluetooth_interface = bt_interface;
    this->config = config;
}
Gatt::~Gatt()
{
    ALOGD(LOGTAG  "(%s) Cleaning up GATT Interface",__FUNCTION__);
    GattInterfaceCleanup();
}

bool Gatt::HandleEnableGatt()
{
    if (GattInterfaceInit(bluetooth_interface)!= true) {
            ALOGE(LOGTAG "(%s) Gatt Initialization Failed",__FUNCTION__);
            return false;
    } else {
        le_supported_profiles = config_get_int(config,CONFIG_DEFAULT_SECTION,
                                                "BtBleSupportedProfiles",0);
        ALOGD(LOGTAG "(%s) Le_Supported_Profiles (%x)",__FUNCTION__,le_supported_profiles);
        if (le_supported_profiles & REMOTE_START_PROFILE)
        {
            rsp = new Rsp(gatt_interface);
            if (!rsp) {
                ALOGE(LOGTAG "(%s) RSP Alloc failed return failure",__FUNCTION__);
                return false;
            }
            for (int i = 0; i < 16; i++) {
               rsp_uuid.uu[i] = 0x30; //Proprietary UUID.
            }
            return true;
        }
    }
}
bool Gatt::HandleDisableGatt()
{
    bool status = true;
    ALOGD(LOGTAG  "(%s) Closing Gatt Instance",__FUNCTION__);
    if ((le_supported_profiles & REMOTE_START_PROFILE) && rsp) {
        status = rsp->StopService();
    }
    return status;
}

