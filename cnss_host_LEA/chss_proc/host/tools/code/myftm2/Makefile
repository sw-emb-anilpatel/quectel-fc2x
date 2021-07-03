#
# Please modify TOOLCHAIN path to meet your environment
#
#export TOOLCHAIN=/<your_toolchain_path>
#export CROSS_COMPILE=${TOOLCHAIN}/armv7l-tizen-linux-gnueabi-
#export KERNELARCH=arm
#export TOOLPREFIX=${CROSS_COMPILE}

#export MAKEARCH=make ARCH=$(KERNELARCH) CROSS_COMPILE=$(TOOLPREFIX)
export MAKEARCH=make

#export CC=$(TOOLPREFIX)gcc
#export AR=$(TOOLPREFIX)ar
#export LD=$(TOOLPREFIX)ld

export MYFTM_TOPDIR=$(shell pwd)/
export LIBTCMD_TOPDIR=$(MYFTM_TOPDIR)/libtcmd
export LIBTLVUTIL_TOPDIR=$(MYFTM_TOPDIR)/libathtestcmd/libtlvutil
export LIBTLV2_TOPDIR=$(MYFTM_TOPDIR)/libathtestcmd/libtlv2
export LIBATHTESTCMD_TOPDIR=$(MYFTM_TOPDIR)/libathtestcmd

export MYFTM_MYFTM=$(MYFTM_TOPDIR)/myftm

export CFLAGS= -I$(MYFTM_TOPDIR)/libnl-3.2.25/include/libnl3 -I$(MYFTM_TOPDIR)/zlib-1.2.11/include

# Use local libnl library or not

default: myftm

clean: myftm_clean  libtlvutil_clean libtlv2_clean libtcmd_clean libathtestcmd_clean libtlvutil_clean

myftm: myftm_clean libtcmd libtlvutil libtlv2 libathtestcmd
	@echo Build myftm
	cd $(MYFTM_MYFTM) && ${MAKEARCH}

myftm_clean:
	@echo clean myftm
	cd $(MYFTM_MYFTM) && ${MAKEARCH} clean

libtlvutil: libtlvutil_clean
	@echo Build libtlvutil
	cd $(LIBTLVUTIL_TOPDIR) && ${MAKEARCH}

libtlvutil_clean:
	@echo clean libtlvutil
	cd $(LIBTLVUTIL_TOPDIR) && ${MAKEARCH} clean

libtlv2: libtlv2_clean
	@echo Build libtlv2
	cd $(LIBTLV2_TOPDIR) && ${MAKEARCH}

libtlv2_clean:
	@echo clean libtlv2
	cd $(LIBTLV2_TOPDIR) && ${MAKEARCH} clean

libtcmd: libtcmd_clean
	@echo Build libtcmd
	cd $(LIBTCMD_TOPDIR) && ${MAKEARCH}

libtcmd_clean:
	@echo clean libtcmd
	cd $(LIBTCMD_TOPDIR) && ${MAKEARCH} clean


libathtestcmd: libathtestcmd_clean
	@echo Build libathtestcmd
	cd $(LIBATHTESTCMD_TOPDIR) && ${MAKEARCH}

libathtestcmd_clean:
	@echo clean libathtestcmd
	cd $(LIBATHTESTCMD_TOPDIR) && ${MAKEARCH} clean
