AM_CPPFLAGS = -O2 \
              -D_GNU_SOURCE

AM_CFLAGS = -Wall \
        -Wundef \
        -Wstrict-prototypes \
        -Wno-trigraphs \
        -std=c99

ACLOCAL_AMFLAGS = -I m4

AM_CPPFLAGS += -I${WORKSPACE}/qcom-opensource/bt/bt-app \
               -I${WORKSPACE}/qcom-opensource/bt/bt-app/utils/include \
               -I${WORKSPACE}/qcom-opensource/bt/bt-app/audio_manager/include \
               -I${WORKSPACE}/qcom-opensource/bt/bt-app/a2dp_sink/include \
               -I${WORKSPACE}/qcom-opensource/bt/bt-app/osi/include \
               -I${WORKSPACE}/qcom-opensource/bt/bt-app/gap/include \
               -I${WORKSPACE}/qcom-opensource/bt/bt-app/pan/include \
               -I${WORKSPACE}/qcom-opensource/bt/bt-app/gatt/include \
               -I${WORKSPACE}/qcom-opensource/bt/bt-app/main/include \
               -I${WORKSPACE}/qcom-opensource/bt/bt-app/hfp_client/include \
               -I${WORKSPACE}/qcom-opensource/bt/bt-app/include \
               -I${WORKSPACE}/system/media/audio/include \
               -I${WORKSPACE}/hardware/libhardware/include

c_sources = ../utils/src/utils.c \
            src/Main.cpp \
            src/ipc.c \
            ../gap/src/Gap.cpp \
            ../gap/src/AdapterProperties.cpp \
            ../gap/src/RemoteDevices.cpp \
            ../audio_manager/src/Audio_Manager.cpp \
            ../a2dp_sink/src/A2dp_Sink.cpp \
            ../hfp_client/src/HfpClient.cpp \
            ../pan/src/Pan.cpp \
            ../gatt/src/Gatt.cpp \
            ../gatt/src/Rsp.cpp

requiredlibs = ../osi/libbtapposi.la -lpthread -lm -ldl -lrt -lhardware

AM_LDFLAGS := $(LDFLAFGS)

bin_PROGRAMS = btapp
btapp_CC = @CC@
btapp_SOURCES = $(c_sources)
btapp_LDADD = $(requiredlibs)
pkgconfigdir = $(libdir)/pkgconfig
