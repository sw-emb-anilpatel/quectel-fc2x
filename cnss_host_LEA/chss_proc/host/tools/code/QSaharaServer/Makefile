# -----------------------   Simple Build Guide -----------------------------------
#
# For x86/x86_64 platform, comment out TOOLPREFIX and SYSROOT
# For cross-compile, Please modify TOOLPREFIX  and SYSROOT to the correct value
#
# --------------------------------------------------------------------------------

export TOOLPREFIX=/opt/fsl-imx-xwayland/4.14-sumo/sysroots/x86_64-pokysdk-linux/usr/bin/aarch64-poky-linux/aarch64-poky-linux-
export SYSROOT=/work/zhonglin/QCN7606.LEA.1.0.C1/imx-yocto-bsp/build-xwayland/tmp/work/aarch64-poky-linux/hostapd/2.6-r0/recipe-sysroot
export CC=$(TOOLPREFIX)gcc --sysroot=$(SYSROOT)

CFLAGS += -g -DDEBUG -Ishared -Os -fPIC
#LDLIBS= -lpthread -lrt -lm -rdynamic -ldl

OBJS=   comm.o \
    external_utils.o \
	kickstart_log.o \
    kickstart.o \
    kickstart_utils.o \
    sahara_protocol.o
	
.PHONY: all clean

qsahara: $(OBJS)
	$(CC) $(OBJS) $(LDLIBS) $(LDFLAGS) -o qsahara_srv

comm.o: comm.c common_protocol_defs.h comm.h kickstart_log.h \
         kickstart_utils.h
external_utils.o: external_utils.c external_utils.h
kickstart_log.o: kickstart_log.c common_protocol_defs.h kickstart_log.h
kickstart.o : common_protocol_defs.h comm.h sahara_protocol.h \
         kickstart_utils.h kickstart_log.h
kickstart_utils.o: kickstart_utils.c common_protocol_defs.h \
         kickstart_utils.h kickstart_log.h
sahara_protocol.o: sahara_protocol.c common_protocol_defs.h comm.h \
         sahara_protocol.h kickstart_log.h kickstart_utils.h

ifeq ($(WINDOWSPC),1)
windows_utils.o: windows_utils.c windows_utils.h
OBJS += windows_utils.o
CFLAGS += -DWINDOWSPC
endif

clean:
	-$(RM) *.o
	-$(RM) qsahara_srv

