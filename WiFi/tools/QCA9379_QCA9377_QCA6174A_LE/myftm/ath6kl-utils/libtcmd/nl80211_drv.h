/*
* Copyright (c) 2011-2012 Qualcomm Atheros Inc. All Rights Reserved.
* Qualcomm Atheros Proprietary and Confidential.
*/

#ifndef _NL80211_DRV_H_
#define _NL80211_DRV_H_
#ifdef ANDROID
#include <netlink/genl/genl.h>
#include <netlink/genl/family.h>
#include <netlink/genl/ctrl.h>
#include <netlink/msg.h>
#include <netlink/attr.h>
#include <netlink/socket.h>
#else
#include <libnl3/netlink/genl/genl.h>
#include <libnl3/netlink/genl/family.h>
#include <libnl3/netlink/genl/ctrl.h>
#include <libnl3/netlink/msg.h>
#include <libnl3/netlink/attr.h>
#include <libnl3/netlink/socket.h>
#endif
#include <linux/nl80211.h>

#include "libtcmd.h"

int nl80211_init(struct tcmd_cfg *cfg);
int nl80211_tcmd_tx(struct tcmd_cfg *cfg, void *buf, int len);
int nl80211_tcmd_rx(struct tcmd_cfg *cfg);
#ifndef CONFIG_AR6002_REV6
int nl80211_set_ep(uint32_t *driv_ep, enum tcmd_ep ep);
#endif
#endif /* _NL80211_DRV_H_ */
