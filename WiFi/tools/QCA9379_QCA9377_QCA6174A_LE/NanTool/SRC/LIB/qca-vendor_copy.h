/*
 * Qualcomm Atheros OUI and vendor specific assignments
 * Copyright (c) 2014-2015, Qualcomm Atheros, Inc.
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#ifndef QCA_VENDOR_H
#define QCA_VENDOR_H

/*
 * This file is a registry of identifier assignments from the Qualcomm Atheros
 * OUI 00:13:74 for purposes other than MAC address assignment. New identifiers
 * can be assigned through normal review process for changes to the upstream
 * hostap.git repository.
 */

#define OUI_QCA 0x001374

/**
 * enum qca_radiotap_vendor_ids - QCA radiotap vendor namespace IDs
 */
enum qca_radiotap_vendor_ids {
	QCA_RADIOTAP_VID_WLANTEST = 0,
};

/**
 * enum qca_nl80211_vendor_subcmds - QCA nl80211 vendor command identifiers
 *
 * @QCA_NL80211_VENDOR_SUBCMD_UNSPEC: Reserved value 0
 *
 * @QCA_NL80211_VENDOR_SUBCMD_TEST: Test command/event
 *
 * @QCA_NL80211_VENDOR_SUBCMD_ROAMING: Set roaming policy for drivers that use
 *	internal BSS-selection. This command uses
 *	@QCA_WLAN_VENDOR_ATTR_ROAMING_POLICY to specify the new roaming policy
 *	for the current connection (i.e., changes policy set by the nl80211
 *	Connect command). @QCA_WLAN_VENDOR_ATTR_MAC_ADDR may optionally be
 *	included to indicate which BSS to use in case roaming is disabled.
 *
 * @QCA_NL80211_VENDOR_SUBCMD_AVOID_FREQUENCY: Recommendation of frequency
 *	ranges to avoid to reduce issues due to interference or internal
 *	co-existence information in the driver. The event data structure is
 *	defined in struct qca_avoid_freq_list.
 *
 * @QCA_NL80211_VENDOR_SUBCMD_DFS_CAPABILITY: Command to check driver support
 *	for DFS offloading.
 *
 * @QCA_NL80211_VENDOR_SUBCMD_NAN: NAN command/event which is used to pass
 *	NAN Request/Response and NAN Indication messages. These messages are
 *	interpreted between the framework and the firmware component.
 *
 * @QCA_NL80211_VENDOR_SUBCMD_KEY_MGMT_SET_KEY: Set key operation that can be
 *	used to configure PMK to the driver even when not connected. This can
 *	be used to request offloading of key management operations. Only used
 *	if device supports QCA_WLAN_VENDOR_FEATURE_KEY_MGMT_OFFLOAD.
 *
 * @QCA_NL80211_VENDOR_SUBCMD_KEY_MGMT_ROAM_AUTH: An extended version of
 *	NL80211_CMD_ROAM event with optional attributes including information
 *	from offloaded key management operation. Uses
 *	enum qca_wlan_vendor_attr_roam_auth attributes. Only used
 *	if device supports QCA_WLAN_VENDOR_FEATURE_KEY_MGMT_OFFLOAD.
 *
 * @QCA_NL80211_VENDOR_SUBCMD_DO_ACS: ACS command/event which is used to
 *	invoke the ACS function in device and pass selected channels to
 *	hostapd.
 *
 * @QCA_NL80211_VENDOR_SUBCMD_GET_FEATURES: Command to get the features
 *	supported by the driver. enum qca_wlan_vendor_features defines
 *	the possible features.
 *
 * @QCA_NL80211_VENDOR_SUBCMD_DFS_OFFLOAD_CAC_STARTED: Event used by driver,
 *	which supports DFS offloading, to indicate a channel availability check
 *	start.
 *
 * @QCA_NL80211_VENDOR_SUBCMD_DFS_OFFLOAD_CAC_FINISHED: Event used by driver,
 *	which supports DFS offloading, to indicate a channel availability check
 *	completion.
 *
 * @QCA_NL80211_VENDOR_SUBCMD_DFS_OFFLOAD_CAC_ABORTED: Event used by driver,
 *	which supports DFS offloading, to indicate that the channel availability
 *	check aborted, no change to the channel status.
 *
 * @QCA_NL80211_VENDOR_SUBCMD_DFS_OFFLOAD_CAC_NOP_FINISHED: Event used by
 *	driver, which supports DFS offloading, to indicate that the
 *	Non-Occupancy Period for this channel is over, channel becomes usable.
 *
 * @QCA_NL80211_VENDOR_SUBCMD_DFS_OFFLOAD_RADAR_DETECTED: Event used by driver,
 *	which supports DFS offloading, to indicate a radar pattern has been
 *	detected. The channel is now unusable.
 */
enum qca_nl80211_vendor_subcmds {
	QCA_NL80211_VENDOR_SUBCMD_UNSPEC = 0,
	QCA_NL80211_VENDOR_SUBCMD_TEST = 1,
	/* subcmds 2..8 not yet allocated */
	QCA_NL80211_VENDOR_SUBCMD_ROAMING = 9,
	QCA_NL80211_VENDOR_SUBCMD_AVOID_FREQUENCY = 10,
	QCA_NL80211_VENDOR_SUBCMD_DFS_CAPABILITY =  11,
	QCA_NL80211_VENDOR_SUBCMD_NAN =  12,
	QCA_NL80211_VENDOR_SUBMCD_STATS_EXT = 13,
	QCA_NL80211_VENDOR_SUBCMD_LL_STATS_SET = 14,
	QCA_NL80211_VENDOR_SUBCMD_LL_STATS_GET = 15,
	QCA_NL80211_VENDOR_SUBCMD_LL_STATS_CLR = 16,
	QCA_NL80211_VENDOR_SUBCMD_LL_STATS_RADIO_RESULTS = 17,
	QCA_NL80211_VENDOR_SUBCMD_LL_STATS_IFACE_RESULTS = 18,
	QCA_NL80211_VENDOR_SUBCMD_LL_STATS_PEERS_RESULTS = 19,
	QCA_NL80211_VENDOR_SUBCMD_GSCAN_START = 20,
	QCA_NL80211_VENDOR_SUBCMD_GSCAN_STOP = 21,
	QCA_NL80211_VENDOR_SUBCMD_GSCAN_GET_VALID_CHANNELS = 22,
	QCA_NL80211_VENDOR_SUBCMD_GSCAN_GET_CAPABILITIES = 23,
	QCA_NL80211_VENDOR_SUBCMD_GSCAN_GET_CACHED_RESULTS = 24,
	QCA_NL80211_VENDOR_SUBCMD_GSCAN_SCAN_RESULTS_AVAILABLE = 25,
	QCA_NL80211_VENDOR_SUBCMD_GSCAN_FULL_SCAN_RESULT = 26,
	QCA_NL80211_VENDOR_SUBCMD_GSCAN_SCAN_EVENT = 27,
	QCA_NL80211_VENDOR_SUBCMD_GSCAN_HOTLIST_AP_FOUND = 28,
	QCA_NL80211_VENDOR_SUBCMD_GSCAN_SET_BSSID_HOTLIST = 29,
	QCA_NL80211_VENDOR_SUBCMD_GSCAN_RESET_BSSID_HOTLIST = 30,
	QCA_NL80211_VENDOR_SUBCMD_GSCAN_SIGNIFICANT_CHANGE = 31,
	QCA_NL80211_VENDOR_SUBCMD_GSCAN_SET_SIGNIFICANT_CHANGE = 32,
	QCA_NL80211_VENDOR_SUBCMD_GSCAN_RESET_SIGNIFICANT_CHANGE = 33,
	QCA_NL80211_VENDOR_SUBCMD_TDLS_ENABLE = 34,
	QCA_NL80211_VENDOR_SUBCMD_TDLS_DISABLE = 35,
	QCA_NL80211_VENDOR_SUBCMD_TDLS_GET_STATUS = 36,
	QCA_NL80211_VENDOR_SUBCMD_TDLS_STATE = 37,
	QCA_NL80211_VENDOR_SUBCMD_GET_SUPPORTED_FEATURES = 38,
	QCA_NL80211_VENDOR_SUBCMD_SCANNING_MAC_OUI = 39,
	QCA_NL80211_VENDOR_SUBCMD_NO_DFS_FLAG = 40,
	QCA_NL80211_VENDOR_SUBCMD_GSCAN_HOTLIST_AP_LOST = 41,
	QCA_NL80211_VENDOR_SUBCMD_GET_CONCURRENCY_MATRIX = 42,
	/* 43..49 - reserved for QCA */
	QCA_NL80211_VENDOR_SUBCMD_KEY_MGMT_SET_KEY = 50,
	QCA_NL80211_VENDOR_SUBCMD_KEY_MGMT_ROAM_AUTH = 51,
	QCA_NL80211_VENDOR_SUBCMD_APFIND = 52,
	/* 53 - reserved - was used by QCA, but not in use anymore */
	QCA_NL80211_VENDOR_SUBCMD_DO_ACS = 54,
	QCA_NL80211_VENDOR_SUBCMD_GET_FEATURES = 55,
	QCA_NL80211_VENDOR_SUBCMD_DFS_OFFLOAD_CAC_STARTED = 56,
	QCA_NL80211_VENDOR_SUBCMD_DFS_OFFLOAD_CAC_FINISHED = 57,
	QCA_NL80211_VENDOR_SUBCMD_DFS_OFFLOAD_CAC_ABORTED = 58,
	QCA_NL80211_VENDOR_SUBCMD_DFS_OFFLOAD_CAC_NOP_FINISHED = 59,
	QCA_NL80211_VENDOR_SUBCMD_DFS_OFFLOAD_RADAR_DETECTED = 60,
	/* 61-90 - reserved for QCA */
	QCA_NL80211_VENDOR_SUBCMD_DATA_OFFLOAD = 91,
	QCA_NL80211_VENDOR_SUBCMD_OCB_SET_CONFIG = 92,
	QCA_NL80211_VENDOR_SUBCMD_OCB_SET_UTC_TIME = 93,
	QCA_NL80211_VENDOR_SUBCMD_OCB_START_TIMING_ADVERT = 94,
	QCA_NL80211_VENDOR_SUBCMD_OCB_STOP_TIMING_ADVERT = 95,
	QCA_NL80211_VENDOR_SUBCMD_OCB_GET_TSF_TIMER = 96,
	QCA_NL80211_VENDOR_SUBCMD_DCC_GET_STATS = 97,
	QCA_NL80211_VENDOR_SUBCMD_DCC_CLEAR_STATS = 98,
	QCA_NL80211_VENDOR_SUBCMD_DCC_UPDATE_NDL = 99,
	QCA_NL80211_VENDOR_SUBCMD_DCC_STATS_EVENT = 100,
	QCA_NL80211_VENDOR_SUBCMD_LINK_PROPERTIES = 101,
	QCA_NL80211_VENDOR_SUBCMD_GW_PARAM_CONFIG = 102,
	QCA_NL80211_VENDOR_SUBCMD_GET_PREFERRED_FREQ_LIST = 103,
	QCA_NL80211_VENDOR_SUBCMD_SET_PROBABLE_OPER_CHANNEL = 104,
	QCA_NL80211_VENDOR_SUBCMD_SETBAND = 105,
	QCA_NL80211_VENDOR_SUBCMD_TRIGGER_SCAN = 106,
	QCA_NL80211_VENDOR_SUBCMD_SCAN_DONE = 107,
	QCA_NL80211_VENDOR_SUBCMD_OTA_TEST = 108,
	QCA_NL80211_VENDOR_SUBCMD_SET_TXPOWER_SCALE = 109,
	/* 110..114 - reserved for QCA */
	QCA_NL80211_VENDOR_SUBCMD_SET_TXPOWER_DECR_DB = 115,
	/* 116..118 - reserved for QCA */
};


enum qca_wlan_vendor_attr {
	QCA_WLAN_VENDOR_ATTR_INVALID = 0,
	/* used by QCA_NL80211_VENDOR_SUBCMD_DFS_CAPABILITY */
	QCA_WLAN_VENDOR_ATTR_DFS     = 1,
	/* used by QCA_NL80211_VENDOR_SUBCMD_NAN */
	QCA_WLAN_VENDOR_ATTR_NAN     = 2,
	/* used by QCA_NL80211_VENDOR_SUBCMD_STATS_EXT */
	QCA_WLAN_VENDOR_ATTR_STATS_EXT     = 3,
	/* used by QCA_NL80211_VENDOR_SUBCMD_STATS_EXT */
	QCA_WLAN_VENDOR_ATTR_IFINDEX     = 4,
	/* used by QCA_NL80211_VENDOR_SUBCMD_ROAMING, u32 with values defined
	 * by enum qca_roaming_policy. */
	QCA_WLAN_VENDOR_ATTR_ROAMING_POLICY = 5,
	QCA_WLAN_VENDOR_ATTR_MAC_ADDR = 6,
	/* used by QCA_NL80211_VENDOR_SUBCMD_GET_FEATURES */
	QCA_WLAN_VENDOR_ATTR_FEATURE_FLAGS = 7,
	QCA_WLAN_VENDOR_ATTR_TEST = 8,
	/* used by QCA_NL80211_VENDOR_SUBCMD_GET_FEATURES */
	/* Unsigned 32-bit value. */
	QCA_WLAN_VENDOR_ATTR_CONCURRENCY_CAPA = 9,
	/* Unsigned 32-bit value */
	QCA_WLAN_VENDOR_ATTR_MAX_CONCURRENT_CHANNELS_2_4_BAND = 10,
	/* Unsigned 32-bit value */
	QCA_WLAN_VENDOR_ATTR_MAX_CONCURRENT_CHANNELS_5_0_BAND = 11,
	/* Unsigned 32-bit value from enum qca_set_band. */
	QCA_WLAN_VENDOR_ATTR_SETBAND_VALUE = 12,
	/* keep last */
	QCA_WLAN_VENDOR_ATTR_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_MAX	= QCA_WLAN_VENDOR_ATTR_AFTER_LAST - 1,
};


enum qca_roaming_policy {
	QCA_ROAMING_NOT_ALLOWED,
	QCA_ROAMING_ALLOWED_WITHIN_ESS,
};

enum qca_wlan_vendor_attr_roam_auth {
	QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_INVALID = 0,
	QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_BSSID,
	QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_REQ_IE,
	QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_RESP_IE,
	QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_AUTHORIZED,
	QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_KEY_REPLAY_CTR,
	QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_PTK_KCK,
	QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_PTK_KEK,
	QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_SUBNET_STATUS,
	/* keep last */
	QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_MAX =
	QCA_WLAN_VENDOR_ATTR_ROAM_AUTH_AFTER_LAST - 1
};

enum qca_wlan_vendor_attr_acs_offload {
	QCA_WLAN_VENDOR_ATTR_ACS_CHANNEL_INVALID = 0,
	QCA_WLAN_VENDOR_ATTR_ACS_PRIMARY_CHANNEL,
	QCA_WLAN_VENDOR_ATTR_ACS_SECONDARY_CHANNEL,
	QCA_WLAN_VENDOR_ATTR_ACS_HW_MODE,
	QCA_WLAN_VENDOR_ATTR_ACS_HT_ENABLED,
	QCA_WLAN_VENDOR_ATTR_ACS_HT40_ENABLED,
	QCA_WLAN_VENDOR_ATTR_ACS_VHT_ENABLED,
	QCA_WLAN_VENDOR_ATTR_ACS_CHWIDTH,
	QCA_WLAN_VENDOR_ATTR_ACS_CH_LIST,
	QCA_WLAN_VENDOR_ATTR_ACS_VHT_SEG0_CENTER_CHANNEL,
	QCA_WLAN_VENDOR_ATTR_ACS_VHT_SEG1_CENTER_CHANNEL,
	QCA_WLAN_VENDOR_ATTR_ACS_FREQ_LIST,
	/* keep last */
	QCA_WLAN_VENDOR_ATTR_ACS_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_ACS_MAX =
	QCA_WLAN_VENDOR_ATTR_ACS_AFTER_LAST - 1
};

enum qca_wlan_vendor_acs_hw_mode {
	QCA_ACS_MODE_IEEE80211B,
	QCA_ACS_MODE_IEEE80211G,
	QCA_ACS_MODE_IEEE80211A,
	QCA_ACS_MODE_IEEE80211AD,
	QCA_ACS_MODE_IEEE80211ANY,
};

/**
 * enum qca_wlan_vendor_features - Vendor device/driver feature flags
 *
 * @QCA_WLAN_VENDOR_FEATURE_KEY_MGMT_OFFLOAD: Device supports key
 *	management offload, a mechanism where the station's firmware
 *	does the exchange with the AP to establish the temporal keys
 *	after roaming, rather than having the user space wpa_supplicant do it.
 * @QCA_WLAN_VENDOR_FEATURE_SUPPORT_HW_MODE_ANY: Device supports automatic
 *	band selection based on channel selection results.
 * @QCA_WLAN_VENDOR_FEATURE_OFFCHANNEL_SIMULTANEOUS: Device supports
 * 	simultaneous off-channel operations.
 * @NUM_QCA_WLAN_VENDOR_FEATURES: Number of assigned feature bits
 */
enum qca_wlan_vendor_features {
	QCA_WLAN_VENDOR_FEATURE_KEY_MGMT_OFFLOAD	= 0,
	QCA_WLAN_VENDOR_FEATURE_SUPPORT_HW_MODE_ANY     = 1,
	QCA_WLAN_VENDOR_FEATURE_OFFCHANNEL_SIMULTANEOUS = 2,
	NUM_QCA_WLAN_VENDOR_FEATURES /* keep last */
};

/**
 * enum qca_wlan_vendor_attr_data_offload_ind - Vendor Data Offload Indication
 *
 * @QCA_WLAN_VENDOR_ATTR_DATA_OFFLOAD_IND_SESSION: Session corresponding to
 *	the offloaded data.
 * @QCA_WLAN_VENDOR_ATTR_DATA_OFFLOAD_IND_PROTOCOL: Protocol of the offloaded
 *	data.
 * @QCA_WLAN_VENDOR_ATTR_DATA_OFFLOAD_IND_EVENT: Event type for the data offload
 *	indication.
 */
enum qca_wlan_vendor_attr_data_offload_ind {
	QCA_WLAN_VENDOR_ATTR_DATA_OFFLOAD_IND_INVALID = 0,
	QCA_WLAN_VENDOR_ATTR_DATA_OFFLOAD_IND_SESSION,
	QCA_WLAN_VENDOR_ATTR_DATA_OFFLOAD_IND_PROTOCOL,
	QCA_WLAN_VENDOR_ATTR_DATA_OFFLOAD_IND_EVENT,

	/* keep last */
	QCA_WLAN_VENDOR_ATTR_DATA_OFFLOAD_IND_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_DATA_OFFLOAD_IND_MAX =
	QCA_WLAN_VENDOR_ATTR_DATA_OFFLOAD_IND_AFTER_LAST - 1
};

enum qca_vendor_attr_get_preferred_freq_list {
	QCA_WLAN_VENDOR_ATTR_GET_PREFERRED_FREQ_LIST_INVALID,
	/* A 32-unsigned value; the interface type/mode for which the preferred
	 * frequency list is requested (see enum qca_iface_type for possible
	 * values); used in GET_PREFERRED_FREQ_LIST command from user-space to
	 * kernel and in the kernel response back to user-space.
	 */
	QCA_WLAN_VENDOR_ATTR_GET_PREFERRED_FREQ_LIST_IFACE_TYPE,
	/* An array of 32-unsigned values; values are frequency (MHz); sent
	 * from kernel space to user space.
	 */
	QCA_WLAN_VENDOR_ATTR_GET_PREFERRED_FREQ_LIST,
	/* keep last */
	QCA_WLAN_VENDOR_ATTR_GET_PREFERRED_FREQ_LIST_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_GET_PREFERRED_FREQ_LIST_MAX =
	QCA_WLAN_VENDOR_ATTR_GET_PREFERRED_FREQ_LIST_AFTER_LAST - 1
};

enum qca_vendor_attr_probable_oper_channel {
	QCA_WLAN_VENDOR_ATTR_PROBABLE_OPER_CHANNEL_INVALID,
	/* 32-bit unsigned value; indicates the connection/iface type likely to
	 * come on this channel (see enum qca_iface_type).
	 */
	QCA_WLAN_VENDOR_ATTR_PROBABLE_OPER_CHANNEL_IFACE_TYPE,
	/* 32-bit unsigned value; the frequency (MHz) of the probable channel */
	QCA_WLAN_VENDOR_ATTR_PROBABLE_OPER_CHANNEL_FREQ,
	/* keep last */
	QCA_WLAN_VENDOR_ATTR_PROBABLE_OPER_CHANNEL_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_PROBABLE_OPER_CHANNEL_MAX =
	QCA_WLAN_VENDOR_ATTR_PROBABLE_OPER_CHANNEL_AFTER_LAST - 1
};

enum qca_iface_type {
	QCA_IFACE_TYPE_STA,
	QCA_IFACE_TYPE_AP,
	QCA_IFACE_TYPE_P2P_CLIENT,
	QCA_IFACE_TYPE_P2P_GO,
	QCA_IFACE_TYPE_IBSS,
	QCA_IFACE_TYPE_TDLS,
};

enum qca_set_band {
	QCA_SETBAND_AUTO,
	QCA_SETBAND_5G,
	QCA_SETBAND_2G,
};

/* IEEE 802.11 Vendor Specific elements */

/**
 * enum qca_vendor_element_id - QCA Vendor Specific element types
 *
 * These values are used to identify QCA Vendor Specific elements. The
 * payload of the element starts with the three octet OUI (OUI_QCA) and
 * is followed by a single octet type which is defined by this enum.
 *
 * @QCA_VENDOR_ELEM_P2P_PREF_CHAN_LIST: P2P preferred channel list.
 *	This element can be used to specify preference order for supported
 *	channels. The channels in this list are in preference order (the first
 *	one has the highest preference) and are described as a pair of
 *	(global) Operating Class and Channel Number (each one octet) fields.
 *
 *	This extends the standard P2P functionality by providing option to have
 *	more than one preferred operating channel. When this element is present,
 *	it replaces the preference indicated in the Operating Channel attribute.
 *	For supporting other implementations, the Operating Channel attribute is
 *	expected to be used with the highest preference channel. Similarly, all
 *	the channels included in this Preferred channel list element are
 *	expected to be included in the Channel List attribute.
 *
 *	This vendor element may be included in GO Negotiation Request, P2P
 *	Invitation Request, and Provision Discovery Request frames.
 */
enum qca_vendor_element_id {
	QCA_VENDOR_ELEM_P2P_PREF_CHAN_LIST = 0,
};

/**
 * enum qca_wlan_vendor_attr_scan - Specifies vendor scan attributes
 *
 * @QCA_WLAN_VENDOR_ATTR_SCAN_IE: IEs that should be included as part of scan
 * @QCA_WLAN_VENDOR_ATTR_SCAN_FREQUENCIES: Nested unsigned 32-bit attributes
 *	with frequencies to be scanned (in MHz)
 * @QCA_WLAN_VENDOR_ATTR_SCAN_SSIDS: Nested attribute with SSIDs to be scanned
 * @QCA_WLAN_VENDOR_ATTR_SCAN_SUPP_RATES: Nested array attribute of supported
 *	rates to be included
 * @QCA_WLAN_VENDOR_ATTR_SCAN_TX_NO_CCK_RATE: flag used to send probe requests
 * 	at non CCK rate in 2GHz band
 * @QCA_WLAN_VENDOR_ATTR_SCAN_FLAGS: Unsigned 32-bit scan flags
 * @QCA_WLAN_VENDOR_ATTR_SCAN_COOKIE: Unsigned 64-bit cookie provided by the
 * 	driver for the specific scan request
 * @QCA_WLAN_VENDOR_ATTR_SCAN_STATUS: Unsigned 8-bit status of the scan
 * 	request decoded as in enum scan_status
 * @QCA_WLAN_VENDOR_ATTR_SCAN_MAC: 6-byte MAC address to use when randomisation
 * 	scan flag is set
 * @QCA_WLAN_VENDOR_ATTR_SCAN_MAC_MASK: 6-byte MAC address mask to be used with
 * 	randomisation
 */
enum qca_wlan_vendor_attr_scan {
	QCA_WLAN_VENDOR_ATTR_SCAN_INVALID_PARAM = 0,
	QCA_WLAN_VENDOR_ATTR_SCAN_IE,
	QCA_WLAN_VENDOR_ATTR_SCAN_FREQUENCIES,
	QCA_WLAN_VENDOR_ATTR_SCAN_SSIDS,
	QCA_WLAN_VENDOR_ATTR_SCAN_SUPP_RATES,
	QCA_WLAN_VENDOR_ATTR_SCAN_TX_NO_CCK_RATE,
	QCA_WLAN_VENDOR_ATTR_SCAN_FLAGS,
	QCA_WLAN_VENDOR_ATTR_SCAN_COOKIE,
	QCA_WLAN_VENDOR_ATTR_SCAN_STATUS,
	QCA_WLAN_VENDOR_ATTR_SCAN_MAC,
	QCA_WLAN_VENDOR_ATTR_SCAN_MAC_MASK,
	QCA_WLAN_VENDOR_ATTR_SCAN_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_SCAN_MAX =
	QCA_WLAN_VENDOR_ATTR_SCAN_AFTER_LAST - 1
};

/**
 * enum scan_status - Specifies the valid values the vendor scan attribute
 * 	QCA_WLAN_VENDOR_ATTR_SCAN_STATUS can take
 *
 * @VENDOR_SCAN_STATUS_NEW_RESULTS: implies the vendor scan is successful with
 * 	new scan results
 * @VENDOR_SCAN_STATUS_ABORTED: implies the vendor scan was aborted in-between
 */
enum scan_status {
	VENDOR_SCAN_STATUS_NEW_RESULTS,
	VENDOR_SCAN_STATUS_ABORTED,
	VENDOR_SCAN_STATUS_MAX,
};

/**
 * enum qca_vendor_attr_ota_test - Specifies the values for vendor
 *                       command QCA_NL80211_VENDOR_SUBCMD_OTA_TEST
 * @QCA_WLAN_VENDOR_ATTR_OTA_TEST_ENABLE: enable ota test
 */
enum qca_vendor_attr_ota_test {
	QCA_WLAN_VENDOR_ATTR_OTA_TEST_INVALID,
	/* 8-bit unsigned value to indicate if OTA test is enabled */
	QCA_WLAN_VENDOR_ATTR_OTA_TEST_ENABLE,
	/* keep last */
	QCA_WLAN_VENDOR_ATTR_OTA_TEST_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_OTA_TEST_MAX =
	QCA_WLAN_VENDOR_ATTR_OTA_TEST_AFTER_LAST - 1
};

/**
 * enum qca_vendor_attr_txpower_scale - vendor sub commands index
 *
 * @QCA_WLAN_VENDOR_ATTR_TXPOWER_SCALE: scaling value
 */
enum qca_vendor_attr_txpower_scale {
	QCA_WLAN_VENDOR_ATTR_TXPOWER_SCALE_INVALID,
	/* 8-bit unsigned value to indicate the scaling of tx power */
	QCA_WLAN_VENDOR_ATTR_TXPOWER_SCALE,
	/* keep last */
	QCA_WLAN_VENDOR_ATTR_TXPOWER_SCALE_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_TXPOWER_SCALE_MAX =
	QCA_WLAN_VENDOR_ATTR_TXPOWER_SCALE_AFTER_LAST - 1
};

/**
 * enum qca_vendor_attr_txpower_decr_db - Attributes for TX power decrease
 *
 * These attributes are used with QCA_NL80211_VENDOR_SUBCMD_SET_TXPOWER_DECR_DB.
 */
enum qca_vendor_attr_txpower_decr_db {
	QCA_WLAN_VENDOR_ATTR_TXPOWER_DECR_DB_INVALID,
	/* 8-bit unsigned value to indicate the reduction of TX power in dB for
	 * a virtual interface. */
	QCA_WLAN_VENDOR_ATTR_TXPOWER_DECR_DB,
	/* keep last */
	QCA_WLAN_VENDOR_ATTR_TXPOWER_DECR_DB_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_TXPOWER_DECR_DB_MAX =
	QCA_WLAN_VENDOR_ATTR_TXPOWER_DECR_DB_AFTER_LAST - 1
};

/* Nan Data Path */
#define QCA_NL80211_VENDOR_SUBCMD_NDP 81
enum qca_wlan_vendor_attr_ndp_params
{
    QCA_WLAN_VENDOR_ATTR_NDP_PARAM_INVALID = 0,
    /* enum of sub commands */
    QCA_WLAN_VENDOR_ATTR_NDP_SUBCMD,
    /* Unsigned 16-bit value */
    QCA_WLAN_VENDOR_ATTR_NDP_TRANSACTION_ID,
    /* NL attributes for data used NDP SUB cmds */
    /* Unsigned 32-bit value indicating a service info */
    QCA_WLAN_VENDOR_ATTR_NDP_SERVICE_INSTANCE_ID,
    /* Unsigned 32-bit value; channel frequency */
    QCA_WLAN_VENDOR_ATTR_NDP_CHANNEL,
    /* Interface Discovery MAC address. An array of 6 Unsigned int8 */
    QCA_WLAN_VENDOR_ATTR_NDP_PEER_DISCOVERY_MAC_ADDR,
    /* Interface name on which NDP is being created */
    QCA_WLAN_VENDOR_ATTR_NDP_IFACE_STR,
    /* Unsigned 32-bit value for security */
    QCA_WLAN_VENDOR_ATTR_NDP_CONFIG_SECURITY,
    /* Unsigned 32-bit value for Qos */
    QCA_WLAN_VENDOR_ATTR_NDP_CONFIG_QOS,
    /* Array of u8: len = QCA_WLAN_VENDOR_ATTR_NAN_DP_APP_INFO_LEN */
    QCA_WLAN_VENDOR_ATTR_NDP_APP_INFO,
    /* Unsigned 32-bit value for NDP instance Id */
    QCA_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID,
    /* Array of instance Ids */
    QCA_WLAN_VENDOR_ATTR_NDP_INSTANCE_ID_ARRAY,
    /* Unsigned 32-bit value for initiator/responder ndp response code accept/reject */
    QCA_WLAN_VENDOR_ATTR_NDP_RESPONSE_CODE,
    /* NDI MAC address. An array of 6 Unsigned int8 */
    QCA_WLAN_VENDOR_ATTR_NDP_NDI_MAC_ADDR,
    /* Unsigned 32-bit value errors types returned by driver */
    QCA_WLAN_VENDOR_ATTR_NDP_DRV_RESPONSE_STATUS_TYPE,
    /* Unsigned 32-bit value error values returned by driver */
    QCA_WLAN_VENDOR_ATTR_NDP_DRV_RETURN_VALUE,
    /* Unsigned 32-bit value for Channel setup configuration */
    QCA_WLAN_VENDOR_ATTR_NDP_CHANNEL_CONFIG,
    /* Unsigned 32-bit value for Cipher Suite Shared Key Type */
    QCA_WLAN_VENDOR_ATTR_NDP_CSID,
    /* Array of u8: len = NAN_PMK_INFO_LEN */
    QCA_WLAN_VENDOR_ATTR_NDP_PMK,
    QCA_WLAN_VENDOR_ATTR_NDP_SCID,
    /* Array of u8: len = NAN_SECURITY_MAX_PASSPHRASE_LEN*/
    QCA_WLAN_VENDOR_ATTR_NDP_PASSPHRASE,
    /* Array of u8: len = NAN_MAX_SERVICE_NAME_LEN */
    QCA_WLAN_VENDOR_ATTR_NDP_SERVICE_NAME,


    /* KEEP LAST */
    QCA_WLAN_VENDOR_ATTR_NDP_AFTER_LAST,
    QCA_WLAN_VENDOR_ATTR_NDP_MAX =
        QCA_WLAN_VENDOR_ATTR_NDP_AFTER_LAST - 1,
};

enum qca_wlan_vendor_attr_ndp_cfg_security
{
   /* Security info will be added when proposed in the specification */
   QCA_WLAN_VENDOR_ATTR_NDP_SECURITY_TYPE = 1,

};

enum qca_wlan_vendor_attr_ndp_qos
{
   /* Qos info will be added when proposed in the specification */
   QCA_WLAN_VENDOR_ATTR_NDP_QOS_CONFIG = 1,

};

/*
 * QCA_NL80211_VENDOR_SUBCMD_NDP sub commands.
 */
enum qca_wlan_vendor_attr_ndp_sub_cmd_value
{
   QCA_WLAN_VENDOR_ATTR_NDP_INTERFACE_CREATE = 1,
   QCA_WLAN_VENDOR_ATTR_NDP_INTERFACE_DELETE = 2,
   QCA_WLAN_VENDOR_ATTR_NDP_INITIATOR_REQUEST = 3,
   QCA_WLAN_VENDOR_ATTR_NDP_INITIATOR_RESPONSE = 4,
   QCA_WLAN_VENDOR_ATTR_NDP_RESPONDER_REQUEST = 5,
   QCA_WLAN_VENDOR_ATTR_NDP_RESPONDER_RESPONSE = 6,
   QCA_WLAN_VENDOR_ATTR_NDP_END_REQUEST = 7,
   QCA_WLAN_VENDOR_ATTR_NDP_END_RESPONSE = 8,
   QCA_WLAN_VENDOR_ATTR_NDP_DATA_REQUEST_IND = 9,
   QCA_WLAN_VENDOR_ATTR_NDP_CONFIRM_IND = 10,
   QCA_WLAN_VENDOR_ATTR_NDP_END_IND = 11
};

#endif /* QCA_VENDOR_H */
