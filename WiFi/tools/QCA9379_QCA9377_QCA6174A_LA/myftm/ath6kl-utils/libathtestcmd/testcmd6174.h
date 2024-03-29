/*
 * Copyright (c) 2014-2015 Qualcomm Atheros, Inc.
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 */

/*
// <summary>
// Wifi driver for AR6002
// </summary>
//
 */
#ifndef _TESTCMD6174_H
#define _TESTCMD6174_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ATH_MAC_LEN
#define ATH_MAC_LEN 6
#endif

 typedef struct _cmdParmeter{
	int isTxStart;
	int isRxStart;

	int freq;
	int bandWidth;
	int rate;
	uint8_t bssid[ATH_MAC_LEN];

	int16_t txPwr;
	int32_t tpcm;
	int32_t paConfig;
	int32_t gainIdx;
	int32_t dacGain;
	int32_t numPkt;
	int32_t agg;
	int32_t pktLen0;
	uint32_t broadcast;
	uint8_t txStation[ATH_MAC_LEN];
	uint8_t rxStation[ATH_MAC_LEN];
	int aifs;

	uint32_t rxPkt;
	int32_t  rxRssi;
	uint32_t rxCrcError;
	uint32_t rxSecError;
	uint32_t expectedPkts;
	uint8_t addr[ATH_MAC_LEN];

	int longpreamble;
	int shortguard;
	uint32_t antenna;
	uint32_t chain;
	uint32_t ani;

	int errCode;
	char errString[256];

	int pwr_backoff;
	int stbc;
	int ldpc;
	int wmode;
	int scpc_cal;

	/* NART Command */
	/* buffer to save NART response data */
	uint8_t nart_rsp_buf[2048];
}_CMD_PARM;


int qca6174ApiInit();
void qca6174init();
void qca6174ApiCleanup(void);
void qca6174ChannelSet(int channel);
void qca6174FreqSet(uint32_t freq);
void qca6174RateSet(int rate);
void qca6174TxPowerSet(int txpwr);

/** @breif Enable long preamble */
void qca6174SetLongPreamble(int enable);

/** @breif Set the interval between frames in aifs number
 *  @param slot aifs slot 0->SIFS, 1->PIFS, 2->DIFS, ... 253 */
void qca6174SetAifsNum(int slot);
void qca6174SetAntenna(int antenna);
void qca6174SetChain(int chain);
void qca6174SetBssid(char *mac);
void qca6174SetTxStation(char *mac);
void qca6174SetRxStation(char *mac);
void qca6174SetAddr(char *mac);
void qca6174TxPcmSet(int txpwr);
void qca6174SetPaCfg(int val);
void qca6174SetDacGain(int val);
void qca6174SetGainIdx(int val);
void qca6174SetNumPkt(int val);
void qca6174SetAgg(int val);
void qca6174SetLdpc(int val);
void qca6174SetStbc(int val);
void qca6174SetWlanMode(int val);
void qca6174SetLPreamble();

void qca6174TxPacketSizeSet(int size);
void qca6174ShortGuardSet(int enable);
void qca6174SetBandWidth(int width);

int qca6174TxSineStart(void);
int qca6174Tx99Start(void);
int qca6174TxFrameStart(void);
int qca6174TxCWStart(void);
int qca6174TxStop(void);
int qca6174RxPacketStart(void);
int qca6174RxPacketStop(void);
uint32_t qca6174RxGetErrorFrameNum(void);
uint32_t qca6174RxGetGoodFrameNum(void);
const char *qca6174GetErrorString(void);
void qca6174_compute_checksum(uint8_t *ptr_6320_eeprom);
int qca6174_clear_scpc_done();
void qca6174_enable_scpc_cal(int val);
int qca6174_get_scpc_cal();
int qca6174_eeprom_block_read(void* buf, uint32_t offset, uint32_t length);
int qca6174_eeprom_write_item(void* buf, uint32_t offset, uint8_t length);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif

