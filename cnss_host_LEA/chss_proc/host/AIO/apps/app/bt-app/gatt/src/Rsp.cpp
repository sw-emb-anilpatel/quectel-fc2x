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

#include "Gatt.hpp"
#include "Rsp.hpp"

#define LOGTAG "RSP "

Rsp::Rsp(btgatt_interface_t *gatt_itf)
{
    gatt_interface = gatt_itf;
    SetDeviceState(WLAN_INACTIVE);
}

Rsp::~Rsp()
{
    ALOGD(LOGTAG "(%s) RSP DeInitialized",__FUNCTION__);
    SetDeviceState(WLAN_INACTIVE);
}

bool Rsp::CopyUUID(bt_uuid_t *uuid)
{
    CHECK_PARAM(uuid)
    for (int i = 0; i < 16; i++) {
        uuid->uu[i] = 0x30; //Proprietary UUID ( all 0x30s.)
    }
    return true;
}

bool Rsp::CopyParams(bt_uuid_t *uuid_dest, bt_uuid_t *uuid_src)
{
    CHECK_PARAM(uuid_dest)
    CHECK_PARAM(uuid_src)

    for (int i = 0; i < 16; i++) {
        uuid_dest->uu[i] = uuid_src->uu[i];
    }
    return true;
}

bool Rsp::MatchParams(bt_uuid_t *uuid_dest, bt_uuid_t *uuid_src)
{
    CHECK_PARAM(uuid_dest)
    CHECK_PARAM(uuid_src)

    for (int i = 0; i < 16; i++) {
        if(uuid_dest->uu[i] != uuid_src->uu[i])
            return false;
    }
    ALOGD(LOGTAG "(%s) UUID Matches",__FUNCTION__);
    return true;
}

bool Rsp::EnableRSP()
{
    ALOGD(LOGTAG "(%s) Enable RSP Initiated",__FUNCTION__);

    BtEvent *event = new BtEvent;
    CHECK_PARAM(event)

    event->event_id = RSP_ENABLE_EVENT;

    CopyUUID(&event->rsp_enable_event.characteristics_uuid);
    CopyUUID(&event->rsp_enable_event.descriptor_uuid);
    CopyUUID(&event->rsp_enable_event.server_uuid);
    CopyUUID(&event->rsp_enable_event.service_uuid);

    ALOGD(LOGTAG "(%s) Posting: RSP_ENABLE_EVENT",__FUNCTION__);
    PostMessage(THREAD_ID_GATT, event);
}

bool Rsp::DisableRSP(int server_if)
{
    ALOGD(LOGTAG "(%s) Disable RSP Initiated",__FUNCTION__);

    BtEvent *event = new BtEvent;
    CHECK_PARAM(event)

    event->event_id = RSP_DISABLE_EVENT;
    event->rsp_disable_event.server_if = server_if;
    PostMessage(THREAD_ID_GATT, event);
}

bool Rsp::RegisterApp()
{
    if (GetGattInterface() == NULL)
    {
        ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
        return false;
    }
    bt_uuid_t server_uuid = GetRSPAttrData()->server_uuid;
    return GetGattInterface()->server->register_server(&server_uuid) == BT_STATUS_SUCCESS;
}

bool Rsp::RegisterClient()
{
    if (GetGattInterface() == NULL)
    {
        ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
        return false;
    }
    bt_uuid_t client_uuid = GetRSPAttrData()->server_uuid;
    client_uuid.uu[0] = 0xff;
    return GetGattInterface()->client->register_client(&client_uuid) == BT_STATUS_SUCCESS;
}

bool Rsp::UnregisterClient(int client_if)
{
    if (GetGattInterface() == NULL)
    {
        ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
        return false;
    }
    return GetGattInterface()->client->unregister_client(client_if) == BT_STATUS_SUCCESS;
}

bool Rsp::ClientSetAdvData(char *str)
{
    bt_status_t        Ret;
    bool              SetScanRsp        = false;
    bool              IncludeName       = true;
    bool              IncludeTxPower    = false;
    int               min_conn_interval = RSP_MIN_CI;
    int               max_conn_interval = RSP_MAX_CI;

    GetGattInterface()->client->set_adv_data(GetRSPClientAppData()->clientIf, SetScanRsp,
                                                IncludeName, IncludeTxPower, min_conn_interval,
                                                max_conn_interval, 0,strlen(str), str,
                                                strlen(str), str, 0,NULL);
}

void Rsp::CleanUp(int server_if)
{
    UnregisterServer(server_if);
    UnregisterClient(GetRSPClientAppData()->clientIf);
}

bool Rsp::UnregisterServer(int server_if)
{
    if (GetGattInterface() == NULL)
    {
        ALOGE(LOGTAG  "Gatt Interface Not present");
        return false;
    }
    return GetGattInterface()->server->unregister_server(server_if) == BT_STATUS_SUCCESS;
}

bool Rsp::StartAdvertisement()
{
    if (GetGattInterface() == NULL)
    {
        ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
        return false;
    }
    ALOGD(LOGTAG  "(%s) Listening on the interface (%d) ",__FUNCTION__,
            GetRSPAppData()->server_if);
    SetDeviceState(WLAN_INACTIVE);
    return GetGattInterface()->client->listen(GetRSPClientAppData()->clientIf, true);
}

bool Rsp::SendResponse(GattsRequestWriteEvent *event)
{
    if (GetGattInterface() == NULL)
    {
        ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
        return false;
    }
    CHECK_PARAM(event)
    btgatt_response_t att_resp;
    int response = -1;
    memset(att_resp.attr_value.value,0,BTGATT_MAX_ATTR_LEN);
    memcpy(att_resp.attr_value.value, event->value, event->length);
    att_resp.attr_value.handle = event->attr_handle;
    att_resp.attr_value.offset = event->offset;
    att_resp.attr_value.len = event->length;
    att_resp.attr_value.auth_req = 0;

    if(!strncasecmp((const char *)(event->value), "on", 2)) {
        if (GetDeviceState() == WLAN_INACTIVE)
        {
            HandleWlanOn();
            SetDeviceState(WLAN_TRANSACTION_PENDING);
        }
        response = 0;
    } else {
        response = -1;
    }

    ALOGD(LOGTAG "(%s) Sending RSP response to write (%d) value (%s) State (%d)",__FUNCTION__,
            GetRSPAppData()->server_if, event->value,GetDeviceState());

    return GetGattInterface()->server->send_response(event->conn_id, event->trans_id,
                                                         response, &att_resp);
}

bool Rsp::HandleWlanOn()
{
    BtEvent *event = new BtEvent;
    CHECK_PARAM(event);
    event->event_id = SKT_API_IPC_MSG_WRITE;
    event->bt_ipc_msg_event.ipc_msg.type = BT_IPC_REMOTE_START_WLAN;
    event->bt_ipc_msg_event.ipc_msg.status = INITIATED;
    StopAdvertisement();
    ALOGD(LOGTAG "(%s) Posting wlan start to main thread",__FUNCTION__);
    PostMessage (THREAD_ID_MAIN, event);
    return true;
}

bool Rsp::StopAdvertisement()
{
    if (GetGattInterface() == NULL)
    {
        ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
        return false;
    }
    ALOGD(LOGTAG "(%s) Stopping listen on the interface (%d) ",__FUNCTION__,
            GetRSPAppData()->server_if);
    return GetGattInterface()->client->listen(GetRSPClientAppData()->clientIf, false);
}

bool Rsp::AddService()
{
    if (GetGattInterface() == NULL)
    {
        ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
        return false;
    }
    btgatt_srvc_id_t srvc_id;
    srvc_id.id.inst_id = 0;   // 1 instance
    srvc_id.is_primary = 1;   // Primary addition
    srvc_id.id.uuid = GetRSPAttrData()->service_uuid;
    return GetGattInterface()->server->add_service(GetRSPAppData()->server_if, &srvc_id,4)
                                                        ==BT_STATUS_SUCCESS;
}

bool Rsp::DisconnectServer()
{
    int server_if = GetRSPConnectionData()->server_if;
    bt_bdaddr_t * bda = GetRSPConnectionData()->bda;
    int conn_id = GetRSPConnectionData()->conn_id;
    ALOGD(LOGTAG  "(%s) Disconnecting interface (%d), connid (%d) ",__FUNCTION__,
            server_if, conn_id);
    return GetGattInterface()->server->disconnect(server_if, bda, conn_id) == BT_STATUS_SUCCESS;
}

bool Rsp::DeleteService()
{
    if (GetGattInterface() == NULL)
    {
        ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
        return false;
    }
    bool status = false;
    int srvc_handle = GetRspSrvcData()->srvc_handle;
    return GetGattInterface()->server->delete_service(GetRSPAppData()->server_if,
                                                            srvc_handle) == BT_STATUS_SUCCESS;
}

bool Rsp::AddCharacteristics()
{
    if (GetGattInterface() == NULL)
    {
        ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
        return false;
    }
    bt_uuid_t char_uuid;
    CopyParams(&char_uuid, &(GetRspSrvcData()->srvc_id->id.uuid));
    int srvc_handle = GetRspSrvcData()->srvc_handle;
    int server_if = GetRspSrvcData()->server_if;
    ALOGD(LOGTAG  "(%s) Adding Characteristics server_if (%d), srvc_handle (%d)",
            __FUNCTION__, server_if,srvc_handle);
    return GetGattInterface()->server->add_characteristic(server_if, srvc_handle, &char_uuid,
                                                            GATT_PROP_WRITE, GATT_PERM_WRITE)
                                                            ==BT_STATUS_SUCCESS;
}

bool Rsp::AddDescriptor(void)
{
    if (GetGattInterface() == NULL)
    {
        ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
        return false;
    }

    bt_uuid_t desc_uuid;
    desc_uuid = GetRSPAttrData()->descriptor_uuid;
    int srvc_handle = GetRspSrvcData()->srvc_handle;
    return GetGattInterface()->server->add_descriptor(GetRSPAppData()->server_if,
                                                        srvc_handle, &desc_uuid,
                                                        GATT_PERM_READ) == BT_STATUS_SUCCESS;
}

bool Rsp::StartService()
{
    if (GetGattInterface() == NULL)
    {
        ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
        return false;
    }

    int srvc_handle = GetRspSrvcData()->srvc_handle;
    return GetGattInterface()->server->start_service(GetRSPAppData()->server_if,
                                                        srvc_handle, GATT_TRANSPORT_LE)
                                                        == BT_STATUS_SUCCESS;
}

bool Rsp::StopService()
{
    if (GetGattInterface() == NULL)
    {
        ALOGE(LOGTAG  "(%s) Gatt Interface Not present",__FUNCTION__);
        return false;
    }

    int srvc_handle = GetRspSrvcData()->srvc_handle;
    return GetGattInterface()->server->stop_service(GetRSPAppData()->server_if,
                                                        srvc_handle) == BT_STATUS_SUCCESS;
}

