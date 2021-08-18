CFLAGS += -fPIC -DLinux -D_HOST_SIM_TESTING -D_FOR_HOST_TESTING -Wno-pointer-to-int-cast
LOCAL_PATH := $(shell pwd)

TARGET_LIB = ./libtlv2.a
SRCS = \
		cmdEncoder/tlvCmdEncoder.c \
		cmdParser/cmdAdcCaptureHandler.c \
		cmdParser/cmdBasicRspHandler.c \
		cmdParser/cmdCalCalTimeHandler.c \
		cmdParser/cmdCommitOtpStream.c \
		cmdParser/cmdConfigHandler.c \
		cmdParser/cmdCustOtpSpaceGetSize.c \
		cmdParser/cmdCustOtpSpaceRead.c \
		cmdParser/cmdCustOtpSpaceWrite.c \
		cmdParser/cmdDPDAgc2Pwr.c \
		cmdParser/cmdDPDComplete.c \
		cmdParser/cmdDPDLoopbackAttn.c \
		cmdParser/cmdDPDLoopbackTiming.c \
		cmdParser/cmdDPDTrainingQuality.c \
		cmdParser/cmdEnableDfeHandler.c \
		cmdParser/cmdGenWaveformHandler.c \
		cmdParser/cmdGetTgtPwr.c \
		cmdParser/cmdHandlerTbl.c \
		cmdParser/cmdHwCalHandler.c \
		cmdParser/cmdLMChannelListAndRspHandler.c \
		cmdParser/cmdLMGoHandler.c \
		cmdParser/cmdLMHWCALHandler.c \
		cmdParser/cmdLMHWCALRSPHandler.c \
		cmdParser/cmdLMQueryHandler.c \
		cmdParser/cmdLMRxHandler.c \
		cmdParser/cmdLMTxHandler.c \
		cmdParser/cmdLMTxInitHandler.c \
		cmdParser/cmdMemReadHandler.c \
		cmdParser/cmdMemWriteHandler.c \
		cmdParser/cmdMoreSegmentHandler.c \
		cmdParser/cmdRegReadHandler.c \
		cmdParser/cmdRegWriteHandler.c \
		cmdParser/cmdRxDcoHandler.c \
		cmdParser/cmdRxGainCalHandler.c \
		cmdParser/cmdRxHandler.c \
		cmdParser/cmdRxStatusHandler.c \
		cmdParser/cmdSARHandler.c \
		cmdParser/cmdSetPCIEConfigParam.c \
		cmdParser/cmdSetPhyRfMode.c \
		cmdParser/cmdSetRegDmn.c \
		cmdParser/cmdTPCCALHandler.c \
		cmdParser/cmdTPCCALPWRHandler.c \
		cmdParser/cmdTxHandler.c \
		cmdParser/cmdTxStatusHandler.c \
		cmdParser/cmdXtalCalHandler.c \
		cmdParser/parseBinCmdStream_v2.c \
		cmdParser/sysCmdGeneric.c \
		cmdParser/sysCmdHandlerTbl.c \
		cmdParser/sysCmdPHYDBGdump.c \
		cmdParser/sysCmdSetChannel.c \
		cmdParser/sysCmdTestData.c \
		cmdParser/sysCmdtlvCaptureCtrl.c \
		cmdParser/cmdDPDLoopbackInfo.c \
		cmdParser/cmdRSTcalHandler.c \
		cmdParser/cmdOfdmaTonePlan.c \
		cmdParser/cmdOfdmaULTxConfig.c \
		cmdParser/cmdLowPowerMode.c \
		common/cmdRspParmsDict.c \
		common/cmdRspParmsInternal.c \
		common/sysCmdRspParmsDict.c \
		tlvcmdrsp/tlvCmdRsp.c \
		util/testUtil.c
#OBJS = $(SRCS:.c=.o)
OBJS = \
		tlvCmdEncoder.o \
		cmdAdcCaptureHandler.o \
		cmdBasicRspHandler.o \
		cmdCalCalTimeHandler.o \
		cmdCommitOtpStream.o \
		cmdConfigHandler.o \
		cmdCustOtpSpaceGetSize.o \
		cmdCustOtpSpaceRead.o \
		cmdCustOtpSpaceWrite.o \
		cmdDPDAgc2Pwr.o \
		cmdDPDComplete.o \
		cmdDPDLoopbackAttn.o \
		cmdDPDLoopbackTiming.o \
		cmdDPDTrainingQuality.o \
		cmdEnableDfeHandler.o \
		cmdGenWaveformHandler.o \
		cmdGetTgtPwr.o \
		cmdHandlerTbl.o \
		cmdHwCalHandler.o \
		cmdLMChannelListAndRspHandler.o \
		cmdLMGoHandler.o \
		cmdLMHWCALHandler.o \
		cmdLMHWCALRSPHandler.o \
		cmdLMQueryHandler.o \
		cmdLMRxHandler.o \
		cmdLMTxHandler.o \
		cmdLMTxInitHandler.o \
		cmdMemReadHandler.o \
		cmdMemWriteHandler.o \
		cmdMoreSegmentHandler.o \
		cmdRegReadHandler.o \
		cmdRegWriteHandler.o \
		cmdRxDcoHandler.o \
		cmdRxGainCalHandler.o \
		cmdRxHandler.o \
		cmdRxStatusHandler.o \
		cmdSARHandler.o \
		cmdSetPCIEConfigParam.o \
		cmdSetPhyRfMode.o \
		cmdSetRegDmn.o \
		cmdTPCCALHandler.o \
		cmdTPCCALPWRHandler.o \
		cmdTxHandler.o \
		cmdTxStatusHandler.o \
		cmdXtalCalHandler.o \
		parseBinCmdStream_v2.o \
		sysCmdGeneric.o \
		sysCmdHandlerTbl.o \
		sysCmdPHYDBGdump.o \
		sysCmdSetChannel.o \
		sysCmdTestData.o \
		sysCmdtlvCaptureCtrl.o \
		cmdDPDLoopbackInfo.o \
		cmdRspParmsDict.o \
		cmdRspParmsInternal.o \
		sysCmdRspParmsDict.o \
		tlvCmdRsp.o \
		testUtil.o \
		cmdRSTcalHandler.o \
		cmdOfdmaTonePlan.o \
		cmdOfdmaULTxConfig.o \
		cmdLowPowerMode.o \

INCLUDES := -I$(LOCAL_PATH)/include\
		-I$(LOCAL_PATH)/cmdEncoder\
		-I$(LOCAL_PATH)/common\
		-I$(LOCAL_PATH)/cmdParser\
		-I$(LOCAL_PATH)/tlvcmdrsp\
		-I$(LOCAL_PATH)/../include

all: $(TARGET_LIB)
$(OBJS) : $(SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) -c $(SRCS)
$(TARGET_LIB): $(OBJS)
	$(AR) -cr $@ $^

clean:
	rm -f ${TARGET_LIB} ${OBJS}
