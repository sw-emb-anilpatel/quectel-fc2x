LOCAL_PATH=$(shell pwd)
SRC = myftm_wlan.c  myftm_main.c myftm_qtip.c

OBJ = $(SRC:.c=.o)

OUT = myftm

default: $(OUT)
	#cp -f $(LOCAL_PATH)/$(OUT) ../
	@echo myftm is ready to use.

INCLUDES += -I$(LOCAL_PATH)/../libtcmd\
		-I$(LOCAL_PATH)/../libathtestcmd\
		-I$(LOCAL_PATH)/../libathtestcmd/include\
		-I$(LOCAL_PATH)/../libtlv2/include\
		-I$(LOCAL_PATH)/../libtlv2\
		-I$(LOCAL_PATH)/../libtlvutil\
		-I$(LOCAL_PATH)/../libtlvutil/common

CFLAGS+= -DCONFIG_FTM_WLAN_AUTOLOAD\
	-DCONFIG_FTM_WLAN\
	-DBOARD_HAS_ATH_WLAN_AR6320\
	-DLINUX_FSL\
	-DDEBUG\
	-DCLD3\

LIBS+= ../libathtestcmd/libathtestcmd.a\
	../libtcmd/libtcmd.a\
	../libathtestcmd/libtlvutil/libtlvutil.a\
	../libathtestcmd/libtlv2/libtlv2.a\
	../libnl-3.2.25/lib/libnl-3.a\
	../libnl-3.2.25/lib/libnl-genl-3.a\
	../zlib-1.2.11/lib/libz.a

.c.o:
	$(CC) $(INCLUDES) $(CFLAGS) -c $< -o $@

$(OUT): $(OBJ)
	$(CC) $(OBJ) $(LIBS) -lpthread -lm -lrt -o $@

clean:
	rm -f $(OBJ) $(OUT)

