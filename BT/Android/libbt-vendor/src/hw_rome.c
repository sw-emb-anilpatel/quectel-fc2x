/*
 *
 *  Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *  Not a Contribution.
 *
 *  Copyright 2012 The Android Open Source Project
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you
 *  may not use this file except in compliance with the License. You may
 *  obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 *  implied. See the License for the specific language governing
 *  permissions and limitations under the License.
 *
 */

/******************************************************************************
 *
 *  Filename:      hw_rome.c
 *
 *  Description:   Contains controller-specific functions, like
 *                      firmware patch download
 *                      low power mode operations
 *
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

#define LOG_TAG   "bt_vendor"

#define UNUSED(x) (void)x

#include "hw_rome.h"

#include <ctype.h>
#include <cutils/properties.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <log/log.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "bt_hci_bdroid.h"
#include "bt_vendor_qcom.h"
#include "hci_uart.h"

#define BT_VERSION_FILEPATH "/data/misc/bluedroid/bt_fw_version.txt"

#ifdef __cplusplus
}
#endif

int read_vs_hci_event(int fd, unsigned char *buf, int size);

/******************************************************************************
**  Variables
******************************************************************************/
FILE *file;
unsigned char *phdr_buffer;
unsigned char *pdata_buffer = NULL;
patch_info rampatch_patch_info;
int rome_ver = ROME_VER_UNKNOWN;
uint32_t g_soc_id = 0;
unsigned char gTlv_type;
unsigned char gTlv_dwndCfg;
static unsigned int wipower_flag = 0;
static unsigned int wipower_handoff_ready = 0;
char *rampatch_file_path;
char *nvm_file_path;
extern char enable_extldo;
unsigned char wait_vsc_evt = TRUE;
bool patch_dnld_pending = FALSE;
int g_enable_ibs = 0;
int dnld_fd = -1;
/******************************************************************************
**  Extern variables
******************************************************************************/
extern uint8_t vnd_local_bd_addr[6];

/*****************************************************************************
**   Functions
*****************************************************************************/
int do_write(int fd, unsigned char *buf, int len)
{
    int ret = 0;
    int write_offset = 0;
    int write_len = len;
    do
    {
        ret = write(fd, buf + write_offset, write_len);
        if (ret < 0)
        {
            ALOGE("%s, write failed ret = %d err = %s", __func__, ret, strerror(errno));
            return -1;
        }
        else if (ret == 0)
        {
            ALOGE("%s, write failed with ret 0 err = %s", __func__, strerror(errno));
            return 0;
        }
        else
        {
            if (ret < write_len)
            {
                ALOGD("%s, Write pending,do write ret = %d err = %s", __func__, ret, strerror(errno));
                write_len = write_len - ret;
                write_offset = ret;
            }
            else
            {
                ALOGV("Write successful");
                break;
            }
        }
    }
    while (1);
    return len;
}

int get_vs_hci_event(unsigned char *rsp)
{
    int err = 0;
    unsigned char paramlen = 0;
    unsigned char EMBEDDED_MODE_CHECK = 0x02;
    FILE *btversionfile = 0;
    unsigned int productid = 0;
    unsigned short patchversion = 0;
    char build_label[255];
    int build_lbl_len;
    unsigned short buildversion = 0;
    unsigned int opcode = 0;
    unsigned char subOpcode = 0;
    unsigned int ocf = 0;
    unsigned int ogf = 0;
    unsigned char status = 0;
    // bool ret  = false;
    unsigned char tu8;

    if ((rsp[EVENTCODE_OFFSET] == VSEVENT_CODE) || (rsp[EVENTCODE_OFFSET] == EVT_CMD_COMPLETE))
        ALOGI("%s: Received HCI-Vendor Specific event", __FUNCTION__);
    else
    {
        ALOGI("%s: Failed to receive HCI-Vendor Specific event", __FUNCTION__);
        err = -EIO;
        goto failed;
    }
    paramlen = rsp[EVT_PLEN];
    ALOGI("%s: Parameter Length: 0x%x", __func__, paramlen);
    if (!unified_hci)
    {
        ocf = rsp[CMD_RSP_OFFSET];
        subOpcode = rsp[RSP_TYPE_OFFSET];
        status = rsp[CMD_STATUS_OFFSET];
        ALOGI("%s: Command response: 0x%x", __func__, ocf);
        ALOGI("%s: Response type   : 0x%x", __func__, subOpcode);
    }
    else
    {
        opcode = rsp[5] << 8 | rsp[4];
        ocf = opcode & 0x03ff;
        ogf = opcode >> 10;
        status = rsp[6];
        subOpcode = rsp[7];
        ALOGI("%s: Opcode: 0x%x", __func__, opcode);
        ALOGI("%s: ocf: 0x%x", __func__, ocf);
        ALOGI("%s: ogf: 0x%x", __func__, ogf);
        ALOGI("%s: Status: 0x%x", __func__, status);
        ALOGI("%s: Sub-Opcode: 0x%x", __func__, subOpcode);
    }

    /* Check the status of the operation */
    switch (ocf)
    {
    case EDL_CMD_REQ_RES_EVT:
        ALOGI("%s: Command Request Response", __FUNCTION__);
        switch (subOpcode)
        {
        case EDL_PATCH_VER_RES_EVT:
        case EDL_APP_VER_RES_EVT:
            if (!unified_hci)
            {
                productid = (unsigned int)(rsp[PATCH_PROD_ID_OFFSET + 3] << 24 | rsp[PATCH_PROD_ID_OFFSET + 2] << 16 |
                                           rsp[PATCH_PROD_ID_OFFSET + 1] << 8 | rsp[PATCH_PROD_ID_OFFSET]);
                patchversion = (unsigned short)(rsp[PATCH_PATCH_VER_OFFSET + 1] << 8 | rsp[PATCH_PATCH_VER_OFFSET]);
                buildversion = (int)(rsp[PATCH_ROM_BUILD_VER_OFFSET + 1] << 8 | rsp[PATCH_ROM_BUILD_VER_OFFSET]);

                ALOGI("\t Current Product ID\t\t: 0x%08x", productid);
                ALOGI("\t Current Patch Version\t\t: 0x%04x", patchversion);
                ALOGI("\t Current ROM Build Version\t: 0x%04x", buildversion);

                if (paramlen - 10)
                {
                    g_soc_id = (unsigned int)(rsp[PATCH_SOC_VER_OFFSET + 3] << 24 | rsp[PATCH_SOC_VER_OFFSET + 2] << 16 |
                                              rsp[PATCH_SOC_VER_OFFSET + 1] << 8 | rsp[PATCH_SOC_VER_OFFSET]);
                    ALOGI("\t Current SOC Version\t\t: 0x%08x", g_soc_id);
                }
            }
            else
            {
                productid = (unsigned int)(rsp[PATCH_PROD_ID_OFFSET_UNIFIED + 3] << 24 |
                                           rsp[PATCH_PROD_ID_OFFSET_UNIFIED + 2] << 16 |
                                           rsp[PATCH_PROD_ID_OFFSET_UNIFIED + 1] << 8 | rsp[PATCH_PROD_ID_OFFSET_UNIFIED]);
                ALOGI("\t unified Current Product ID\t\t: 0x%08x", productid);

                /* Patch Version indicates FW patch version */
                patchversion =
                    (unsigned short)(rsp[PATCH_PATCH_VER_OFFSET_UNIFIED + 1] << 8 | rsp[PATCH_PATCH_VER_OFFSET_UNIFIED]);
                ALOGI("\t unified Current Patch Version\t\t: 0x%04x", patchversion);

                /* ROM Build Version indicates ROM build version like 1.0/1.1/2.0 */
                buildversion =
                    (int)(rsp[PATCH_ROM_BUILD_VER_OFFSET_UNIFIED + 1] << 8 | rsp[PATCH_ROM_BUILD_VER_OFFSET_UNIFIED]);
                ALOGI("\t unified Current ROM Build Version\t: 0x%04x", buildversion);

                if ((paramlen - 14) > 0)
                {
                    g_soc_id = (unsigned int)(rsp[PATCH_SOC_VER_OFFSET_UNIFIED + 3] << 24 |
                                              rsp[PATCH_SOC_VER_OFFSET_UNIFIED + 2] << 16 |
                                              rsp[PATCH_SOC_VER_OFFSET_UNIFIED + 1] << 8 | rsp[PATCH_SOC_VER_OFFSET_UNIFIED]);
                    ALOGI("\t unified Current SOC Version\t\t: 0x%08x", g_soc_id);
                }
            }
            if (NULL != (btversionfile = fopen(BT_VERSION_FILEPATH, "wb")))
            {
                fprintf(btversionfile, "Bluetooth Controller Product ID    : 0x%08x\n", productid);
                fprintf(btversionfile, "Bluetooth Controller Patch Version : 0x%04x\n", patchversion);
                fprintf(btversionfile, "Bluetooth Controller Build Version : 0x%04x\n", buildversion);
                fprintf(btversionfile, "Bluetooth Controller SOC Version   : 0x%08x\n", g_soc_id);
                fclose(btversionfile);
            }
            else
            {
                ALOGE("Failed to dump SOC version info. Errno:%d", errno);
            }
            /* Rome Chipset Version can be decided by Patch version and SOC version,
            Upper 2 bytes will be used for Patch version and Lower 2 bytes will be
            used for SOC as combination for BT host driver */
            rome_ver = (buildversion << 16) | (g_soc_id & 0x0000ffff);

            break;
        case EDL_PATCH_TLV_REQ_CMD:
        case EDL_TVL_DNLD_RES_EVT:
        case EDL_CMD_EXE_STATUS_EVT:
            err = status;
            switch (err)
            {
            case HCI_CMD_SUCCESS:
                ALOGI("%s: Download Packet successfully!", __FUNCTION__);
                break;
            case PATCH_LEN_ERROR:
                ALOGI(
                    "%s: Invalid patch length argument passed for EDL PATCH "
                    "SET REQ cmd",
                    __FUNCTION__);
                break;
            case PATCH_VER_ERROR:
                ALOGI(
                    "%s: Invalid patch version argument passed for EDL PATCH "
                    "SET REQ cmd",
                    __FUNCTION__);
                break;
            case PATCH_CRC_ERROR:
                ALOGI("%s: CRC check of patch failed!!!", __FUNCTION__);
                break;
            case PATCH_NOT_FOUND:
                ALOGI("%s: Invalid patch data!!!", __FUNCTION__);
                break;
            case TLV_TYPE_ERROR:
                ALOGI("%s: TLV Type Error !!!", __FUNCTION__);
                break;
            default:
                ALOGI("%s: Undefined error (0x%x)", __FUNCTION__, err);
                break;
            }
            break;
        case EDL_GET_BUILD_INFO:
        case HCI_VS_GET_BUILD_VER_EVT:
            if (!unified_hci)
            {
                build_lbl_len = rsp[5];
                memcpy(build_label, &rsp[6], build_lbl_len);
            }
            else
            {
                build_lbl_len = rsp[8];
                memcpy(build_label, &rsp[9], build_lbl_len);
            }

            *(build_label + build_lbl_len) = '\0';

            ALOGI("BT SoC FW SU Build info: %s, %d", build_label, build_lbl_len);
            if (NULL != (btversionfile = fopen(BT_VERSION_FILEPATH, "a+b")))
            {
                fprintf(btversionfile, "Bluetooth Contoller SU Build info  : %s\n", build_label);
                fclose(btversionfile);
            }
            else
            {
                ALOGI("Failed to dump  FW SU build info. Errno:%d", errno);
            }
            break;
        }
        break;

    case NVM_ACCESS_CODE:
        ALOGI("%s: NVM Access Code!!!", __FUNCTION__);
        err = HCI_CMD_SUCCESS;
        break;

    case EDL_SET_BAUDRATE_CMD_OCF:
    case EDL_SET_BAUDRATE_RSP_EVT:
        tu8 = unified_hci ? status : rsp[BAUDRATE_RSP_STATUS_OFFSET];
        /* Rome 1.1 has bug with the response, so it should ignore it. */
        if (tu8 != BAUDRATE_CHANGE_SUCCESS)
        {
            ALOGE("%s: Set Baudrate request failed - 0x%x", __FUNCTION__, rsp[CMD_STATUS_OFFSET]);
            err = -1;
        }
        break;
    case EDL_WIP_QUERY_CHARGING_STATUS_EVT:
        /* Query charging command has below return values
        0 - in embedded mode not charging
        1 - in embedded mode and charging
        2 - hadofff completed and in normal mode
        3 - no wipower supported on mtp. so irrepective of charging
        handoff command has to be sent if return values are 0 or 1.
        These change include logic to enable generic BT turn on sequence.*/
        if (rsp[4] < EMBEDDED_MODE_CHECK)
        {
            ALOGI("%s: WiPower Charging in Embedded Mode!!!", __FUNCTION__);
            wipower_handoff_ready = rsp[4];
            wipower_flag = 1;
        }
        break;
    case EDL_WIP_START_HANDOFF_TO_HOST_EVENT:
        /*TODO: rsp code 00 mean no charging
        this is going to change in FW soon*/
        if (rsp[4] == NON_WIPOWER_MODE)
        {
            ALOGE("%s: WiPower Charging hand off not ready!!!", __FUNCTION__);
        }
        break;
    case HCI_VS_GET_ADDON_FEATURES_EVENT:
        tu8 = unified_hci ? rsp[11] : rsp[4];
        if ((tu8 & ADDON_FEATURES_EVT_WIPOWER_MASK))
        {
            ALOGI("%s: WiPower feature supported!!", __FUNCTION__);
            property_set("persist.bluetooth.a4wp", "true");
        }
        break;
    case HCI_VS_STRAY_EVT:
        /* WAR to handle stray Power Apply EVT during patch download */
        ALOGI("%s: Stray HCI VS EVENT", __FUNCTION__);
        if (patch_dnld_pending && dnld_fd != -1)
        {
            unsigned char rsp[HCI_MAX_EVENT_SIZE];
            memset(rsp, 0x00, HCI_MAX_EVENT_SIZE);
            read_vs_hci_event(dnld_fd, rsp, HCI_MAX_EVENT_SIZE);
        }
        else
        {
            ALOGE("%s: Not a valid status!!!", __FUNCTION__);
            err = -1;
        }
        break;
    default:
        ALOGE("%s: Not a valid status!!!", __FUNCTION__);
        err = -1;
        break;
    }

failed:
    return err;
}

/*
 * Read an VS HCI event from the given file descriptor.
 */
int read_vs_hci_event(int fd, unsigned char *buf, int size)
{
    int remain, r;
    int count = 0, i;
    unsigned short int opcode;
    UNUSED(i);

    if (size <= 0)
    {
        ALOGE("Invalid size arguement!");
        return -1;
    }

    ALOGI("%s: Wait for HCI-Vendor Specfic Event from SOC", __FUNCTION__);

    /* The first byte identifies the packet type. For HCI event packets, it
     * should be 0x04, so we read until we get to the 0x04. */
    /* It will keep reading until find 0x04 byte */
    while (1)
    {
        r = read(fd, buf, 1);
        if (r < 0)
        {
            ALOGE("%s, read result is wrong!", __FUNCTION__);
            return -1;
        }
        else if (r == 0)
        {
            ALOGI("%s, read no data, continue", __FUNCTION__);
            usleep(10000);
            continue;
        }
        else
            ALOGI("%s, read buf[0] = 0x%02x", __FUNCTION__, buf[0]);

        if (buf[0] == 0x04) break;
    }
    count++;

    /* The next two bytes are the event code and parameter total length. */
    while (count < 3)
    {
        r = read(fd, buf + count, 3 - count);
        if ((r <= 0) || ((buf[1] != VSEVENT_CODE) && (buf[1] != EVT_CMD_COMPLETE)))
        {
            ALOGE("It is not VS event !! ret: %d, EVT: %d", r, buf[1]);
            return -1;
        }
        count += r;
    }

    /* Now we read the parameters. */
    if (buf[2] < (size - 3))
        remain = buf[2];
    else
        remain = size - 3;

    while ((count - 3) < remain)
    {
        r = read(fd, buf + count, remain - (count - 3));
        if (r <= 0) return -1;
        count += r;
    }

    if (buf[1] == VSEVENT_CODE)
    {
        ALOGV("VSC Event! good");
    }
    else if (buf[1] == EVT_CMD_COMPLETE)
    {
        ALOGI("%s: Expected CC", __func__);
        if (count > UNIFIED_HCI_CC_MIN_LENGTH)
        {
            opcode = (buf[4] | (buf[5] << 8));
            if (((HCI_VS_WIPOWER_CMD_OPCODE == opcode) && (UNIFIED_HCI_CODE == buf[6])) ||
                    ((HCI_VS_GET_VER_CMD_OPCODE == opcode) && (buf[7] == EDL_PATCH_VER_REQ_CMD)))
            {
                unified_hci = 1;
                ALOGI("HCI Unified command interface supported");
            }
        }
        if (unified_hci)
        {
            return (get_vs_hci_event(buf) == HCI_CMD_SUCCESS) ? count : -1;
        }
    }
    else
    {
        ALOGI("%s: Unexpected event! : opcode: %d", __func__, buf[1]);
        count = -1;
        return count;
    }

    /* Check if the set patch command is successful or not */
    if (get_vs_hci_event(buf) != HCI_CMD_SUCCESS) return -1;

    return count;
}

int hci_send_vs_cmd(int fd, unsigned char *cmd, unsigned char *rsp, int size)
{
    int ret = 0;

    /* Send the HCI command packet to UART for transmission */
    ret = do_write(fd, cmd, size);
    if (ret != size)
    {
        ALOGE("%s: Send failed with ret value: %d", __FUNCTION__, ret);
        goto failed;
    }

    if (wait_vsc_evt)
    {
        /* Check for response from the Controller */
        if (read_vs_hci_event(fd, rsp, HCI_MAX_EVENT_SIZE) < 0)
        {
            ret = -ETIMEDOUT;
            ALOGI("%s: Failed to get HCI-VS Event from SOC", __FUNCTION__);
            goto failed;
        }
        ALOGI("%s: Received HCI-Vendor Specific Event from SOC", __FUNCTION__);
    }

failed:
    return ret;
}

void frame_hci_cmd_pkt(unsigned char *cmd, int edl_cmd, unsigned int p_base_addr, int segtNo, int size)
{
    int offset = 0;
    hci_command_hdr *cmd_hdr;

    memset(cmd, 0x0, HCI_MAX_CMD_SIZE);

    cmd_hdr = (void *)(cmd + 1);

    cmd[0] = HCI_COMMAND_PKT;
    cmd_hdr->opcode = cmd_opcode_pack(HCI_VENDOR_CMD_OGF, HCI_PATCH_CMD_OCF);
    cmd_hdr->plen = size;
    cmd[4] = edl_cmd;

    switch (edl_cmd)
    {
    case EDL_PATCH_SET_REQ_CMD:
        /* Copy the patch header info as CMD params */
        memcpy(&cmd[5], phdr_buffer, PATCH_HDR_LEN);
        ALOGD("%s: Sending EDL_PATCH_SET_REQ_CMD", __FUNCTION__);
        ALOGD("HCI-CMD %d:\t0x%x \t0x%x \t0x%x \t0x%x \t0x%x", segtNo, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);
        break;
    case EDL_PATCH_DLD_REQ_CMD:
        offset = ((segtNo - 1) * MAX_DATA_PER_SEGMENT);
        p_base_addr += offset;
        cmd_hdr->plen = (size + 6);
        cmd[5] = (size + 4);
        cmd[6] = EXTRACT_BYTE(p_base_addr, 0);
        cmd[7] = EXTRACT_BYTE(p_base_addr, 1);
        cmd[8] = EXTRACT_BYTE(p_base_addr, 2);
        cmd[9] = EXTRACT_BYTE(p_base_addr, 3);
        memcpy(&cmd[10], (pdata_buffer + offset), size);

        ALOGD("%s: Sending EDL_PATCH_DLD_REQ_CMD: size: %d bytes", __FUNCTION__, size);
        ALOGD(
            "HCI-CMD %d:\t0x%x\t0x%x\t0x%x\t0x%x\t0x%x\t0x%x\t0x%x\t"
            "0x%x\t0x%x\t0x%x\t\n",
            segtNo, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5], cmd[6], cmd[7], cmd[8], cmd[9]);
        break;
    case EDL_PATCH_ATCH_REQ_CMD:
        ALOGD("%s: Sending EDL_PATCH_ATTACH_REQ_CMD", __FUNCTION__);
        ALOGD("HCI-CMD %d:\t0x%x \t0x%x \t0x%x \t0x%x \t0x%x", segtNo, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);
        break;
    case EDL_PATCH_RST_REQ_CMD:
        ALOGD("%s: Sending EDL_PATCH_RESET_REQ_CMD", __FUNCTION__);
        ALOGD("HCI-CMD %d:\t0x%x \t0x%x \t0x%x \t0x%x \t0x%x", segtNo, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);
        break;
    case EDL_PATCH_VER_REQ_CMD:
        ALOGD("%s: Sending EDL_PATCH_VER_REQ_CMD", __FUNCTION__);
        ALOGD("HCI-CMD %d:\t0x%x \t0x%x \t0x%x \t0x%x \t0x%x", segtNo, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);
        break;
    case EDL_PATCH_TLV_REQ_CMD:
        // ALOGD("%s: Sending EDL_PATCH_TLV_REQ_CMD", __FUNCTION__);
        /* Parameter Total Length */
        cmd[3] = size + 2;

        /* TLV Segment Length */
        cmd[5] = size;
        // ALOGD("HCI-CMD %d:\t0x%x \t0x%x \t0x%x \t0x%x \t0x%x \t0x%x",
        // segtNo, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], cmd[5]);
        offset = (segtNo * MAX_SIZE_PER_TLV_SEGMENT);
        memcpy(&cmd[6], (pdata_buffer + offset), size);
        break;
    case EDL_GET_BUILD_INFO:
        ALOGD("%s: Sending EDL_GET_BUILD_INFO", __FUNCTION__);
        ALOGD("HCI-CMD %d:\t0x%x \t0x%x \t0x%x \t0x%x \t0x%x", segtNo, cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);
        break;
    default:
        ALOGE("%s: Unknown EDL CMD !!!", __FUNCTION__);
    }
}

static int check_chip_version(int g_soc_id){

    if(g_soc_id == SOC_VER_QCA6696)  return 0;
    if(g_soc_id == SCO_VER_QCA6698)  return 0;
    if(g_soc_id == SOC_VER_QCA6695)  return 0;
    if(g_soc_id == SOC_VER_QCA206X)  return 0;
    if(g_soc_id == SOC_VER_QCA206X_20)  return 0;
    if(g_soc_id == SOC_VER_QCA206X_G)  return 0;
    if(g_soc_id == SOC_VER_QCC207X)  return 0;
    return -1;
}

void update_nvm_format(tlv_nvm_hdr *nvm_ptr)
{
    uint8_t *nvm_byte_ptr = (uint8_t *) nvm_ptr;

    if (!nvm_ptr)
    {
        return;
    }

    nvm_byte_ptr += sizeof(tlv_nvm_hdr);
    /* Write BD Address */
    if (nvm_ptr->tag_id == TAG_NUM_2)
    {
        // memcpy(nvm_byte_ptr, vnd_local_bd_addr, 6);
        ALOGI("BD Address: %.02x:%.02x:%.02x:%.02x:%.02x:%.02x", *nvm_byte_ptr, *(nvm_byte_ptr + 1),
              *(nvm_byte_ptr + 2), *(nvm_byte_ptr + 3), *(nvm_byte_ptr + 4), *(nvm_byte_ptr + 5));
    }

    if (nvm_ptr->tag_id == TAG_NUM_17)
    {
        uint8_t baudrate = BAUDRATE_3000000;

        ALOGI("%s: baudrate %02x", __func__, baudrate);

        /* Byte#1: UART baudrate */
        *(nvm_byte_ptr + 1) = baudrate;
    }

    if(check_chip_version(g_soc_id) == 0)
    {

        /* Update Tag#27: SIBS Settings */
        if (nvm_ptr->tag_id == TAG_NUM_27)
        {
            if (!g_enable_ibs)
            {
                /* TxP Sleep Mode: Disable */
                *(nvm_byte_ptr + 1) &= ~0x01;
                ALOGI("%s: SIBS Disable", __func__);
            }
            else
            {
                /* TxP Sleep Mode-1:UART_SIBS, 2:USB_SUSPEND, 3: GPIO_OOB, 4: UART_HWIBS
                 */
                *(nvm_byte_ptr + 1) |= 0x01;
                ALOGI("%s: SIBS Enable", __func__);
            }
        }
    }

    if(g_soc_id == SOC_VER_QCA9377)
    {

        if (nvm_ptr->tag_id == TAG_NUM_17)
        {
            if (!g_enable_ibs)
            {
                *(nvm_byte_ptr) &= (~(1 << 7));
                ALOGI("%s: SIBS Disable", __func__);
            }
            else
            {
                *(nvm_byte_ptr) |= (1 << 7);
            }
        }

        /* Update Tag#27: SIBS Settings */
        if (nvm_ptr->tag_id == TAG_NUM_27)
        {
            if (!g_enable_ibs)
            {
                /* TxP Sleep Mode: Disable */
                *(nvm_byte_ptr) &= ~0x01;
                ALOGI("%s: SIBS Disable", __func__);
            }
            else
            {
                /* TxP Sleep Mode-1:UART_SIBS, 2:USB_SUSPEND, 3: GPIO_OOB, 4: UART_HWIBS
                 */
                *(nvm_byte_ptr) |= 0x01;
                ALOGI("%s: SIBS Enable", __func__);
            }
        }
    }
}

int rome_get_tlv_file(char *file_path)
{
    FILE *pFile;
    long fileSize;
    int readSize, err = 0, total_segment, remain_size, nvm_length, nvm_index, i;
    // unsigned short nvm_tag_len;
    tlv_patch_info *ptlv_header;
    tlv_nvm_hdr *nvm_ptr;
    unsigned char data_buf[PRINT_BUF_SIZE] =
    {
        0,
    };
    unsigned char *nvm_byte_ptr;

    UNUSED(err);
    UNUSED(total_segment);
    UNUSED(remain_size);
    ALOGI("File Open (%s)", file_path);
    pFile = fopen(file_path, "r");
    if (pFile == NULL)
    {
        ALOGE("%s File Open Fail", file_path);
        return -1;
    }

    /* Get File Size */
    fseek(pFile, 0, SEEK_END);
    fileSize = ftell(pFile);
    rewind(pFile);

    pdata_buffer = (unsigned char *)malloc(sizeof(char) * fileSize);
    if (pdata_buffer == NULL)
    {
        ALOGE("Allocated Memory failed");
        fclose(pFile);
        return -1;
    }

    /* Copy file into allocated buffer */
    readSize = fread(pdata_buffer, 1, fileSize, pFile);

    /* File Close */
    fclose(pFile);

    if (readSize != fileSize)
    {
        ALOGE("Read file size(%d) not matched with actual file size (%ld bytes)", readSize, fileSize);
        return -1;
    }

    ptlv_header = (tlv_patch_info *)pdata_buffer;

    /* To handle different event between rampatch and NVM */
    gTlv_type = ptlv_header->tlv_type;
    gTlv_dwndCfg = ptlv_header->tlv.patch.dwnd_cfg;

    if (ptlv_header->tlv_type == TLV_TYPE_PATCH)
    {
        ALOGI("====================================================");
        ALOGI("TLV Type\t\t\t : 0x%x", ptlv_header->tlv_type);
        ALOGI("Length\t\t\t : %d bytes",
              (ptlv_header->tlv_length1) | (ptlv_header->tlv_length2 << 8) | (ptlv_header->tlv_length3 << 16));
        ALOGI("Total Length\t\t\t : %d bytes", ptlv_header->tlv.patch.tlv_data_len);
        ALOGI("Patch Data Length\t\t\t : %d bytes", ptlv_header->tlv.patch.tlv_patch_data_len);
        ALOGI("Signing Format Version\t : 0x%x", ptlv_header->tlv.patch.sign_ver);
        ALOGI("Signature Algorithm\t\t : 0x%x", ptlv_header->tlv.patch.sign_algorithm);
        ALOGI("Event Handling\t\t\t : 0x%x", ptlv_header->tlv.patch.dwnd_cfg);
        ALOGI("Reserved\t\t\t : 0x%x", ptlv_header->tlv.patch.reserved1);
        ALOGI("Product ID\t\t\t : 0x%04x\n", ptlv_header->tlv.patch.prod_id);
        ALOGI("Rom Build Version\t\t : 0x%04x\n", ptlv_header->tlv.patch.build_ver);
        ALOGI("Patch Version\t\t : 0x%04x\n", ptlv_header->tlv.patch.patch_ver);
        ALOGI("Reserved\t\t\t : 0x%x\n", ptlv_header->tlv.patch.reserved2);
        ALOGI("Patch Entry Address\t\t : 0x%x\n", (ptlv_header->tlv.patch.patch_entry_addr));
        ALOGI("====================================================");

    }
    else if (ptlv_header->tlv_type == TLV_TYPE_NVM)
    {
        ALOGI("====================================================");
        ALOGI("TLV Type\t\t\t : 0x%x", ptlv_header->tlv_type);
        ALOGI("Length\t\t\t : %d bytes",
              nvm_length = (ptlv_header->tlv_length1) | (ptlv_header->tlv_length2 << 8) | (ptlv_header->tlv_length3 << 16));

        if (nvm_length <= 0) return readSize;

        for (nvm_byte_ptr = (unsigned char *)(nvm_ptr = &(ptlv_header->tlv.nvm)), nvm_index = 0; nvm_index < nvm_length;
                nvm_ptr = (tlv_nvm_hdr *)nvm_byte_ptr)
        {
            // ALOGI("TAG ID\t\t\t : %d", nvm_ptr->tag_id);
            // ALOGI("TAG Length\t\t\t : %d", nvm_tag_len = nvm_ptr->tag_len);
            // ALOGI("TAG Pointer\t\t\t : %d", nvm_ptr->tag_ptr);
            // ALOGI("TAG Extended Flag\t\t : %d", nvm_ptr->tag_ex_flag);

            /* Increase nvm_index to NVM data */
            nvm_index += sizeof(tlv_nvm_hdr);
            nvm_byte_ptr += sizeof(tlv_nvm_hdr);

            update_nvm_format(nvm_ptr);

            for (i = 0; (i < nvm_ptr->tag_len && (i * 3 + 2) < PRINT_BUF_SIZE); i++)
                snprintf((char *)data_buf, PRINT_BUF_SIZE, "%s%.02x ", (char *)data_buf, *(nvm_byte_ptr + i));

            // ALOGI("TAG Data\t\t\t : %s", data_buf);

            /* Clear buffer */
            memset(data_buf, 0x0, PRINT_BUF_SIZE);

            /* increased by tag_len */
            nvm_index += nvm_ptr->tag_len;
            nvm_byte_ptr += nvm_ptr->tag_len;
        }

        ALOGI("====================================================");

    }
    else
    {
        ALOGI("TLV Header type is unknown (%d) ", ptlv_header->tlv_type);
    }

    return readSize;
}

int rome_tlv_dnld_segment(int fd, int index, int seg_size, unsigned char wait_cc_evt)
{
    int size = 0, err = -1;
    unsigned char cmd[HCI_MAX_CMD_SIZE];
    unsigned char rsp[HCI_MAX_EVENT_SIZE];

    // ALOGI("%s: Downloading TLV Patch segment no.%d, size:%d, wait_cc_evt:%d", __FUNCTION__, index, seg_size,
    // wait_cc_evt);

    /* Frame the HCI CMD PKT to be sent to Controller*/
    frame_hci_cmd_pkt(cmd, EDL_PATCH_TLV_REQ_CMD, 0, index, seg_size);

    /* Total length of the packet to be sent to the Controller */
    size = (HCI_CMD_IND + HCI_COMMAND_HDR_SIZE + cmd[PLEN]);

    /* Initialize the RSP packet everytime to 0 */
    memset(rsp, 0x0, HCI_MAX_EVENT_SIZE);

    /* Send HCI Command packet to Controller */
    err = hci_send_vs_cmd(fd, (unsigned char *)cmd, rsp, size);
    if (err != size)
    {
        ALOGE("Failed to send the patch payload to the Controller! 0x%x", err);
        return err;
    }

    if (!unified_hci)
    {
        if (wait_cc_evt)
        {
            /* Initialize the RSP packet everytime to 0 */
            memset(rsp, 0x0, HCI_MAX_EVENT_SIZE);
            err = read_hci_event(fd, rsp, HCI_MAX_EVENT_SIZE);
            if (err < 0)
            {
                ALOGE("%s: Failed to downlaod patch segment: %d!", __FUNCTION__, index);
                return err;
            }
        }
    }

    // ALOGI("%s: Successfully downloaded patch segment: %d", __FUNCTION__, index);
    return err;
}

int rome_tlv_dnld_req(int fd, int tlv_size)
{
    int total_segment, remain_size, i, err = -1;
    unsigned char wait_cc_evt;
    // unsigned int rom = rome_ver >> 16;

    total_segment = tlv_size / MAX_SIZE_PER_TLV_SEGMENT;
    remain_size = (tlv_size < MAX_SIZE_PER_TLV_SEGMENT) ? tlv_size : (tlv_size % MAX_SIZE_PER_TLV_SEGMENT);

    ALOGI("%s: TLV size: %d, Total Seg num: %d, remain size: %d", __FUNCTION__, tlv_size, total_segment, remain_size);

    if (gTlv_type == TLV_TYPE_PATCH)
    {
        /* Prior to Rome version 3.2(including inital few rampatch release of Rome 3.2), the event
         * handling mechanism is ROME_SKIP_EVT_NONE. After few release of rampatch for Rome 3.2, the
         * mechamism is changed to ROME_SKIP_EVT_VSE_CC. Rest of the mechanism is not used for now
         */
        switch (gTlv_dwndCfg)
        {
        case ROME_SKIP_EVT_NONE:
            wait_vsc_evt = TRUE;
            wait_cc_evt = TRUE;
            ALOGI("Event handling type: ROME_SKIP_EVT_NONE");
            break;
        case ROME_SKIP_EVT_VSE_CC:
            wait_vsc_evt = FALSE;
            wait_cc_evt = FALSE;
            ALOGI("Event handling type: ROME_SKIP_EVT_VSE_CC");
            break;
        /* Not handled for now */
        case ROME_SKIP_EVT_VSE:
        case ROME_SKIP_EVT_CC:
        default:
            ALOGE("Unsupported Event handling: %d", gTlv_dwndCfg);
            break;
        }
    }
    else
    {
        wait_vsc_evt = TRUE;
        wait_cc_evt = TRUE;
    }

    for (i = 0; i < total_segment; i++)
    {
        if ((i + 1) == total_segment)
        {
            if (((rome_ver == TUFELLO_VER_1_1) || IS_HASTINGS_SOC(rome_ver)) && (gTlv_type == TLV_TYPE_PATCH))
            {
                if (gTlv_dwndCfg == ROME_SKIP_EVT_NONE)
                {
                    wait_cc_evt = !remain_size ? FALSE : TRUE;
                }
                else if (gTlv_dwndCfg == ROME_SKIP_EVT_VSE_CC)
                {
                    wait_vsc_evt = !remain_size ? TRUE : FALSE;
                }
            }
            else if ((rome_ver >= ROME_VER_1_1) && (rome_ver < ROME_VER_3_2) && (gTlv_type == TLV_TYPE_PATCH))
            {
                /* If the Rome version is from 1.1 to 3.1
                 * 1. No CCE for the last command segment but all other segment
                 * 2. All the command segments get VSE including the last one
                 */
                wait_cc_evt = !remain_size ? FALSE : TRUE;
            }
            else if ((rome_ver >= ROME_VER_3_2) && (gTlv_type == TLV_TYPE_PATCH))
            {
                /* If the Rome version is 3.2
                 * 1. None of the command segments receive CCE
                 * 2. No command segments receive VSE except the last one
                 * 3. If gTlv_dwndCfg is ROME_SKIP_EVT_NONE then the logic is
                 *    same as Rome 2.1, 2.2, 3.0
                 */
                if (gTlv_dwndCfg == ROME_SKIP_EVT_NONE)
                {
                    wait_cc_evt = !remain_size ? FALSE : TRUE;
                }
                else if (gTlv_dwndCfg == ROME_SKIP_EVT_VSE_CC)
                {
                    wait_vsc_evt = !remain_size ? TRUE : FALSE;
                }
            }
        }

        patch_dnld_pending = TRUE;
        if ((err = rome_tlv_dnld_segment(fd, i, MAX_SIZE_PER_TLV_SEGMENT, wait_cc_evt)) < 0) goto error;
        /* FC20/FC21/FC900E use chipset qca1023/9377 need short delay ,cause no delay,uart flow contro is exception in some platform */
        if(g_soc_id == SOC_VER_QCA9377) usleep(5*1000);

        patch_dnld_pending = FALSE;
    }

    if (((rome_ver == TUFELLO_VER_1_1) || IS_HASTINGS_SOC(rome_ver)) && (gTlv_type == TLV_TYPE_PATCH))
    {
        if (gTlv_dwndCfg == ROME_SKIP_EVT_NONE)
        {
            wait_cc_evt = remain_size ? FALSE : TRUE;
        }
        else if (gTlv_dwndCfg == ROME_SKIP_EVT_VSE_CC)
        {
            wait_vsc_evt = remain_size ? TRUE : FALSE;
        }
    }
    else if ((rome_ver >= ROME_VER_1_1) && (rome_ver < ROME_VER_3_2) && (gTlv_type == TLV_TYPE_PATCH))
    {
        /* If the Rome version is from 1.1 to 3.1
         * 1. No CCE for the last command segment but all other segment
         * 2. All the command segments get VSE including the last one
         */
        wait_cc_evt = remain_size ? FALSE : TRUE;
    }
    else if ((rome_ver >= ROME_VER_3_2) && (gTlv_type == TLV_TYPE_PATCH))
    {
        /* If the Rome version is 3.2
         * 1. None of the command segments receive CCE
         * 2. No command segments receive VSE except the last one
         * 3. If gTlv_dwndCfg is ROME_SKIP_EVT_NONE then the logic is
         *    same as Rome 2.1, 2.2, 3.0
         */
        if (gTlv_dwndCfg == ROME_SKIP_EVT_NONE)
        {
            wait_cc_evt = remain_size ? FALSE : TRUE;
        }
        else if (gTlv_dwndCfg == ROME_SKIP_EVT_VSE_CC)
        {
            wait_vsc_evt = remain_size ? TRUE : FALSE;
        }
    }
    patch_dnld_pending = TRUE;
    if (remain_size) err = rome_tlv_dnld_segment(fd, i, remain_size, wait_cc_evt);
    patch_dnld_pending = FALSE;
error:
    if (patch_dnld_pending) patch_dnld_pending = FALSE;
    return err;
}

int rome_download_tlv_file(int fd)
{
    int tlv_size, err = -1;

    /* Rampatch TLV file Downloading */
    pdata_buffer = NULL;
    if ((tlv_size = rome_get_tlv_file(rampatch_file_path)) < 0) goto error;

    if ((err = rome_tlv_dnld_req(fd, tlv_size)) < 0) goto error;

    if (pdata_buffer != NULL)
    {
        free(pdata_buffer);
        pdata_buffer = NULL;
    }
    usleep(100 * 1000);
    /* NVM TLV file Downloading */
    if ((tlv_size = rome_get_tlv_file(nvm_file_path)) < 0) goto error;

    if ((err = rome_tlv_dnld_req(fd, tlv_size)) < 0) goto error;

error:
    if (pdata_buffer != NULL) free(pdata_buffer);

    return err;
}

int rome_patch_ver_req(int fd)
{
    int size, ret = 0;
    unsigned char cmd[HCI_MAX_CMD_SIZE];
    unsigned char rsp[HCI_MAX_EVENT_SIZE];

    /* Frame the HCI CMD to be sent to the Controller */
    frame_hci_cmd_pkt(cmd, EDL_PATCH_VER_REQ_CMD, 0, -1, EDL_PATCH_CMD_LEN);

    /* Total length of the packet to be sent to the Controller */
    size = (HCI_CMD_IND + HCI_COMMAND_HDR_SIZE + EDL_PATCH_CMD_LEN);

    tcflush(fd, TCIOFLUSH);
    usleep(20 * 1000);
    ALOGI("%s,delay 20ms after flush IO queue", __func__);

    /* Send HCI Command packet to Controller */

    /* Send the HCI command packet to UART for transmission */
    ret = do_write(fd, cmd, size);
    if (ret != size)
    {
        ALOGE("%s: Send failed with ret value: %d", __func__, ret);
        goto hci_send_vs_cmd_failed;
    }

    /* Check for response from the Controller */
    if (wait_vsc_evt)
    {
        int remain = 0, n = 1, count = 0, r = 0, err = 0, code = 0, flags = 0;
        unsigned short int opcode;
        fd_set fdread;
        struct timeval st_timeout;

        /* set nonblock mode */
        if ((flags = fcntl(fd, F_GETFL, 0)) < 0)
        {
            ALOGE("%s: fcntl get failed!ret = %d,errno=%d,errstr=%s\n", __func__, flags, errno, strerror(errno));
            err = -1;
            goto read_vs_hci_event_failed;
        }
        flags |= O_NONBLOCK;
        if ((code = fcntl(fd, F_SETFL, flags)) < 0)
        {
            ALOGE("%s: fcntl set nonblock mode failed!ret = %d,errno=%d,errstr=%s\n", __func__, code, errno, strerror(errno));
            err = -1;
            goto read_vs_hci_event_failed;
        }

        ALOGI("%s: Wait for HCI-Vendor Specfic Event from SOC", __FUNCTION__);
        st_timeout.tv_sec = 0;
        st_timeout.tv_usec = 20 * 1000; /* 20ms */
        while (1)
        {
            FD_ZERO(&fdread);
            FD_SET(fd, &fdread);
            do
            {
                code = select(fd + 1, &fdread, NULL, NULL, &st_timeout); /* block */
            }
            while (code == -1 && errno == EINTR);                      /* skip the interrupt signal */
            switch (code)
            {
            case 0:
                ALOGE("%s: select() timeout!ret = %d,errno=%d,errstr=%s\n", __func__, code, errno, strerror(errno));
                err = -1;
                goto read_vs_hci_event_failed;
            case -1:
                ALOGE("%s: select() error!ret = %d,errno=%d,errstr=%s\n", __func__, code, errno, strerror(errno));
                err = -1;
                goto read_vs_hci_event_failed;
            default:
                if (FD_ISSET(fd, &fdread))
                {
                    r = read(fd, rsp + count, n);
                    if (r <= 0)
                    {
                        continue;
                    }
                    else
                    {
                        /* The first byte identifies the packet type. For HCI event packets, it
                         * should be 0x04, so we read until we get to the 0x04. */
                        /* It will keep reading until find 0x04 byte */
                        if (count == 0)
                        {
                            if (rsp[0] != 0x04)
                            {
                                continue;
                            }
                        }
                        count += r;
                        /* The next two bytes are the event code and parameter total length. */
                        if (count <= 3)
                        {
                            if (count < 3)
                            {
                                n = 3 - count;
                            }
                            if (count > 1 && (rsp[1] != VSEVENT_CODE) && (rsp[1] != EVT_CMD_COMPLETE))
                            {
                                ALOGE("%s: It is not VS event!! EVT: %d\n", rsp[1]);
                                err = -1;
                                goto read_vs_hci_event_failed;
                            }
                        }
                        /* Now we read the parameters. */
                        if (count >= 3)
                        {
                            if (count == 3)
                            {
                                if (rsp[2] < (HCI_MAX_EVENT_SIZE - 3))
                                    remain = rsp[2];
                                else
                                    remain = HCI_MAX_EVENT_SIZE - 3;
                            }
                            n = remain - (count - 3);
                            if ((count - 3) >= remain)
                            {
                                goto read_done;
                            }
                        }
                    }
                }
            }
        }
read_done:
        if (rsp[1] == VSEVENT_CODE)
        {
            ALOGV("%s: VSC Event! good", __func__);
        }
        else if (rsp[1] == EVT_CMD_COMPLETE)
        {
            ALOGI("%s: Expected CC", __func__);
            if (count > UNIFIED_HCI_CC_MIN_LENGTH)
            {
                opcode = (rsp[4] | (rsp[5] << 8));
                if (((HCI_VS_WIPOWER_CMD_OPCODE == opcode) && (UNIFIED_HCI_CODE == rsp[6])) ||
                        ((HCI_VS_GET_VER_CMD_OPCODE == opcode) && (rsp[7] == EDL_PATCH_VER_REQ_CMD)))
                {
                    unified_hci = 1;
                    ALOGI("%s: HCI Unified command interface supported", __func__);
                }
            }
            if (unified_hci)
            {
                err = (get_vs_hci_event(rsp) == HCI_CMD_SUCCESS) ? count : -1;
                goto read_vs_hci_event_done;
            }
        }
        else
        {
            ALOGI("%s: Unexpected event! : opcode: %d", __func__, rsp[1]);
            err = -1;
            goto read_vs_hci_event_failed;
        }

        /* Check if the set patch command is successful or not */
        if (get_vs_hci_event(rsp) != HCI_CMD_SUCCESS)
        {
            err = -1;
            goto read_vs_hci_event_failed;
        }

        err = count;

read_vs_hci_event_done:

read_vs_hci_event_failed:
        /* set block mode */
        flags &= ~O_NONBLOCK;
        if ((code = fcntl(fd, F_SETFL, flags)) < 0)
        {
            ALOGE("%s: fcntl set block mode failed!ret = %d,errno=%d,errstr=%s\n", __func__, code, errno, strerror(errno));
        }
        if (err < 0)
        {
            ret = -ETIMEDOUT;
            ALOGI("%s: Failed to get HCI-VS Event from SOC", __func__);
            goto hci_send_vs_cmd_failed;
        }
        ALOGI("%s: Received HCI-Vendor Specific Event from SOC", __func__);
    }

hci_send_vs_cmd_failed:
    if (ret != size)
    {
        ALOGE("%s: Failed to attach the patch payload to the Controller!", __func__);
        return ret;
    }

    /* Read Command Complete Event - This is extra routine for ROME 1.0. From ROM 2.0, it should be removed. */
    if (!unified_hci)
    {
        ret = read_hci_event(fd, rsp, HCI_MAX_EVENT_SIZE);
        if (ret < 0)
        {
            ALOGE("%s: Failed to get patch version(s)", __func__);
            return ret;
        }
    }

    return ret;
}

int rome_get_build_info_req(int fd)
{
    int size, err = 0;
    unsigned char cmd[HCI_MAX_CMD_SIZE];
    unsigned char rsp[HCI_MAX_EVENT_SIZE];

    /* Frame the HCI CMD to be sent to the Controller */
    frame_hci_cmd_pkt(cmd, EDL_GET_BUILD_INFO, 0, -1, EDL_PATCH_CMD_LEN);

    /* Total length of the packet to be sent to the Controller */
    size = (HCI_CMD_IND + HCI_COMMAND_HDR_SIZE + EDL_PATCH_CMD_LEN);

    /* Send HCI Command packet to Controller */
    err = hci_send_vs_cmd(fd, (unsigned char *)cmd, rsp, size);
    if (err != size)
    {
        ALOGE("Failed to send get build info cmd to the SoC!");
        goto error;
    }

    if (!unified_hci)
    {
        err = read_hci_event(fd, rsp, HCI_MAX_EVENT_SIZE);
        if (err < 0)
        {
            ALOGE("%s: Failed to get build info", __FUNCTION__);
            goto error;
        }
    }
error:
    return err;
}

int rome_set_baudrate_req(int fd)
{
    int size, err = 0;
    unsigned char cmd[HCI_MAX_CMD_SIZE];
    unsigned char rsp[HCI_MAX_EVENT_SIZE];
    hci_command_hdr *cmd_hdr;
    int flags;

    memset(cmd, 0x0, HCI_MAX_CMD_SIZE);

    cmd_hdr = (void *)(cmd + 1);
    cmd[0] = HCI_COMMAND_PKT;
    cmd_hdr->opcode = cmd_opcode_pack(HCI_VENDOR_CMD_OGF, EDL_SET_BAUDRATE_CMD_OCF);
    cmd_hdr->plen = VSC_SET_BAUDRATE_REQ_LEN;
    cmd[4] = BAUDRATE_3000000;

    /* Total length of the packet to be sent to the Controller */
    size = (HCI_CMD_IND + HCI_COMMAND_HDR_SIZE + VSC_SET_BAUDRATE_REQ_LEN);

    tcflush(fd, TCIOFLUSH);
    usleep(20 * 1000);
    ALOGI("%s,delay 20ms after flush IO queue", __func__);

    /* Flow off during baudrate change */
    if ((err = userial_vendor_ioctl(USERIAL_OP_FLOW_OFF, &flags)) < 0)
    {
        ALOGE("%s: HW Flow-off error: 0x%x\n", __FUNCTION__, err);
        goto error;
    }

    /* Send the HCI command packet to UART for transmission */
    err = do_write(fd, cmd, size);
    if (err != size)
    {
        ALOGE("%s: Send failed with ret value: %d", __FUNCTION__, err);
        goto error;
    }

    /* Change Local UART baudrate to high speed UART */
    userial_vendor_set_baud(USERIAL_BAUD_3M);

    /* Flow on after changing local uart baudrate */
    if ((err = userial_vendor_ioctl(USERIAL_OP_FLOW_ON, &flags)) < 0)
    {
        ALOGE("%s: HW Flow-on error: 0x%x \n", __FUNCTION__, err);
        return err;
    }

    tcflush(fd, TCIOFLUSH);
    usleep(20 * 1000);
    ALOGI("%s,delay 20ms after flush IO queue", __func__);

    /* Send one more change baud cmd to ensure both module*/
    err = do_write(fd, cmd, size);
    if (err != size)
    {
        ALOGE("%s: 2th Send failed with ret value: %d", __FUNCTION__, err);
        goto error;
    }

    memset(rsp, 0, HCI_MAX_EVENT_SIZE);
    /* Check for response from the Controller */
    if ((err = read_vs_hci_event(fd, rsp, HCI_MAX_EVENT_SIZE)) < 0)
    {
        ALOGE("%s: Failed to get HCI-VS Event from SOC", __FUNCTION__);
        goto error;
    }

    ALOGI("%s: Received HCI-Vendor Specific Event from SOC", __FUNCTION__);

    if (!unified_hci)
    {
        /* Wait for command complete event */
        err = read_hci_event(fd, rsp, HCI_MAX_EVENT_SIZE);
        if (err < 0)
        {
            ALOGE("%s: Failed to set patch info on Controller", __FUNCTION__);
            goto error;
        }
    }

error:
    return err;
}

int rome_hci_reset(int fd)
{
    int size, err = 0;
    unsigned char cmd[HCI_MAX_CMD_SIZE];
    unsigned char rsp[HCI_MAX_EVENT_SIZE];
    hci_command_hdr *cmd_hdr;
    int flags;

    UNUSED(flags);
    ALOGI("%s: HCI RESET ", __FUNCTION__);

    memset(cmd, 0x0, HCI_MAX_CMD_SIZE);

    cmd_hdr = (void *)(cmd + 1);
    cmd[0] = HCI_COMMAND_PKT;
    cmd_hdr->opcode = HCI_RESET;
    cmd_hdr->plen = 0;

    /* Total length of the packet to be sent to the Controller */
    size = (HCI_CMD_IND + HCI_COMMAND_HDR_SIZE);
    err = do_write(fd, cmd, size);
    if (err != size)
    {
        ALOGE("%s: Send failed with ret value: %d", __FUNCTION__, err);
        err = -1;
        goto error;
    }

    /* Wait for command complete event */
    err = read_hci_event(fd, rsp, HCI_MAX_EVENT_SIZE);
    if (err < 0)
    {
        ALOGE("%s: Failed to set patch info on Controller", __FUNCTION__);
        goto error;
    }

error:
    return err;
}

void enable_controller_log(int fd, unsigned char wait_for_evt)
{
    int ret = 0;
    /* VS command to enable controller logging to the HOST. By default it is disabled */
    unsigned char cmd[6] = {0x01, 0x17, 0xFC, 0x02, 0x00, 0x00};
    unsigned char rsp[HCI_MAX_EVENT_SIZE];
    char value[PROPERTY_VALUE_MAX] = {'\0'};

    property_get("persist.service.bdroid.soclog", value, "false");

    // value at cmd[5]: 1 - to enable, 0 - to disable
    ret = (strcmp(value, "true") == 0) ? cmd[5] = 0x01 : 0;
    ALOGI("%s: %d", __func__, ret);
    /* Ignore vsc evt if wait_for_evt is true */
    if (wait_for_evt) wait_vsc_evt = FALSE;

    ret = hci_send_vs_cmd(fd, (unsigned char *)cmd, rsp, 6);
    if (ret != 6)
    {
        ALOGE("%s: command failed", __func__);
    }

    if (!unified_hci)
    {
        /*Ignore hci_event if wait_for_evt is true*/
        if (wait_for_evt) goto end;
        ret = read_hci_event(fd, rsp, HCI_MAX_EVENT_SIZE);
        if (ret < 0)
        {
            ALOGE("%s: Failed to get CC for enable SoC log", __FUNCTION__);
        }
    }
end:
    wait_vsc_evt = TRUE;
    return;
}

static int disable_internal_ldo(int fd)
{
    int ret = 0;
    if (enable_extldo)
    {
        unsigned char cmd[5] = {0x01, 0x0C, 0xFC, 0x01, 0x32};
        unsigned char rsp[HCI_MAX_EVENT_SIZE];

        ALOGI(" %s ", __FUNCTION__);
        ret = do_write(fd, cmd, 5);
        if (ret != 5)
        {
            ALOGE("%s: Send failed with ret value: %d", __FUNCTION__, ret);
            ret = -1;
        }
        else
        {
            /* Wait for command complete event */
            ret = read_hci_event(fd, rsp, HCI_MAX_EVENT_SIZE);
            if (ret < 0)
            {
                ALOGE("%s: Failed to get response from controller", __FUNCTION__);
            }
        }
    }
    return ret;
}

int rome_soc_init(int fd, char *bdaddr)
{
    int err = -1, size = 0;
    dnld_fd = fd;
    ALOGI(" %s ", __FUNCTION__);
    UNUSED(bdaddr);
    /* Get Rome version information */
    if ((err = rome_patch_ver_req(fd)) < 0)
    {
        ALOGI("%s: Fail to get Rome Version (0x%x)", __FUNCTION__, err);
        goto error;
    }

    ALOGI("%s: Chipset Version (0x%08x) g_soc_id(0x%x)", __func__, rome_ver, g_soc_id);

    switch (g_soc_id)
    {
    case SOC_VER_QCA9377:
        rampatch_file_path = QCA9377_RAMPATCH_TLV_1_0_1_PATH;
        nvm_file_path = QCA9377_NVM_TLV_1_0_1_PATH;
        break;
    case SOC_VER_QCA206X_20:
        rampatch_file_path = QCA206X_RAMPATCH_TLV_UART_2_0_PATH;
        nvm_file_path = QCA206X_NVM_TLV_UART_2_0_PATH;
        break;
    case SOC_VER_QCA206X: //SCO_VER_QCA6698
        rampatch_file_path = QCA206X_RAMPATCH_TLV_UART_2_1_PATH;
        nvm_file_path = QCA206X_NVM_TLV_UART_2_1_PATH;
        break;
    case SOC_VER_QCA206X_G:
        rampatch_file_path = QCA206X_RAMPATCH_TLV_UART_2_1_PATH;
        nvm_file_path = QCA206X_NVM_TLV_UART_2_1_G_PATH;
        break;        
    case SOC_VER_QCC207X:
        rampatch_file_path = QCC207X_RAMPATCH_TLV_UART_2_0_PATH;
        nvm_file_path = QCC207X_NVM_TLV_UART_2_0_PATH;
        break;
    default:
        ALOGE("Unknow soc id:0x%x", g_soc_id);
        goto error;
    }

    /* Change baud rate 115.2 kbps to 3Mbps*/
    err = rome_set_baudrate_req(fd);
    if (err < 0)
    {
        ALOGE("%s: Baud rate change failed!", __FUNCTION__);
        goto error;
    }
    ALOGI("%s: Baud rate changed successfully ", __FUNCTION__);
    /* Donwload TLV files (rampatch, NVM) */
    err = rome_download_tlv_file(fd);
    if (err < 0)
    {
        ALOGE("%s: Download TLV file failed!", __FUNCTION__);
        goto error;
    }
    ALOGI("%s: Download TLV file successfully ", __FUNCTION__);

    /* Get SU FM label information */
    if ((err = rome_get_build_info_req(fd)) < 0)
    {
        ALOGI("%s: Fail to get Rome FW SU Build info (0x%x)", __FUNCTION__, err);
        // Ignore the failure of ROME FW SU label information
        err = 0;
    }

    /* Disable internal LDO to use external LDO instead*/
    // err = disable_internal_ldo(fd);

    /* Send HCI Reset */
    err = rome_hci_reset(fd);
    if (err < 0)
    {
        ALOGE("HCI Reset Failed !!");
        goto error;
    }

    ALOGI("HCI Reset is done\n");

error:
    dnld_fd = -1;
    return err;
}
