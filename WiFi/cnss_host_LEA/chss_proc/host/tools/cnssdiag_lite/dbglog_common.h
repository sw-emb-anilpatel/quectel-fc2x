/*
 * Copyright (c) 2015, 2017 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *
 * 2015 Qualcomm Atheros, Inc.
 *
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 */

/*
 * Copyright (c) 2011, 2014-2015 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * This file was originally distributed by Qualcomm Atheros, Inc.
 * under proprietary terms before Copyright ownership was assigned
 * to the Linux Foundation.
 */

#ifndef _DBGLOG_COMMON_H_
#define _DBGLOG_COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "dbglog_id.h"
#include "dbglog.h"

#define MAX_DBG_MSGS 256

#define ATH6KL_FWLOG_PAYLOAD_SIZE              1500

#define DBGLOG_PRINT_PREFIX "FWLOG: "

/* Handy Macros to read data length and type from FW */
#define WLAN_DIAG_0_TYPE_S          0
#define WLAN_DIAG_0_TYPE            0x000000ff
#define WLAN_DIAG_0_TYPE_GET(x)     WMI_F_MS(x, WLAN_DIAG_0_TYPE)
#define WLAN_DIAG_0_TYPE_SET(x, y)  WMI_F_RMW(x, y, WLAN_DIAG_0_TYPE)
/* bits 8-15 reserved */

/* length includes the size of wlan_diag_data */
#define WLAN_DIAG_0_LEN_S           16
#define WLAN_DIAG_0_LEN             0xffff0000
#define WLAN_DIAG_0_LEN_GET(x)      WMI_F_MS(x, WLAN_DIAG_0_LEN)
#define WLAN_DIAG_0_LEN_SET(x, y)   WMI_F_RMW(x, y, WLAN_DIAG_0_LEN)

#define CNSS_DIAG_SLEEP_INTERVAL    5   /* In secs */

#define ATH6KL_FWLOG_MAX_ENTRIES   20
#define ATH6KL_FWLOG_PAYLOAD_SIZE  1500

#define DIAG_WLAN_DRIVER_UNLOADED 6
#define DIAG_WLAN_DRIVER_LOADED   7
#define DIAG_TYPE_LOGS   1
#define DIAG_TYPE_EVENTS 2

typedef enum {
	DBGLOG_PROCESS_DEFAULT = 0,
	DBGLOG_PROCESS_PRINT_RAW,       /* print them in debug view */
	DBGLOG_PROCESS_POOL_RAW,        /* user buffer pool to save them */
	DBGLOG_PROCESS_NET_RAW,         /* user buffer pool to save them */
	DBGLOG_PROCESS_MAX,
} dbglog_process_t;

enum wlan_diag_config_type {
	DIAG_VERSION_INFO,
	DIAG_BASE_TIMESTAMP,
};

enum wlan_diag_frame_type {
	WLAN_DIAG_TYPE_CONFIG,
	WLAN_DIAG_TYPE_EVENT,
	WLAN_DIAG_TYPE_LOG,
	WLAN_DIAG_TYPE_MSG,
	WLAN_DIAG_TYPE_LEGACY_MSG,
	WLAN_DIAG_TYPE_EVENT_V2,
	WLAN_DIAG_TYPE_LOG_V2,
	WLAN_DIAG_TYPE_MSG_V2,
};

/* log/event are always 32-bit aligned. Padding is inserted after
 * optional payload to satisify this requirement */
struct wlan_diag_data {
	unsigned int word0;             /* type, length */
	unsigned int target_time;
	unsigned int code;              /* Diag log or event Code */
	uint8_t payload[0];
};

typedef struct event_report_s {
	unsigned int diag_type;
	unsigned short event_id;
	unsigned short length;
} event_report_t;

/*
 * Custom debug_print handlers
 * Args:
 * radio_id
 * module Id
 * vap id
 * debug msg id
 * Time stamp
 * no of arguments
 * pointer to the buffer holding the args
 */
typedef A_BOOL (*module_dbg_print)(A_UINT32, A_UINT32, A_UINT16, A_UINT32,
				   A_UINT64, A_UINT16, A_UINT32 *);

/** Register module specific dbg print*/
void dbglog_reg_modprint(A_UINT32 mod_id, module_dbg_print printfn);

#ifdef __cplusplus
}
#endif

#endif /* _DBGLOG_COMMON_H_ */
