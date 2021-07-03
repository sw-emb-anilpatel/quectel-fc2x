#!/bin/bash
#backports.sh A script to download backports source code, caf patches, build and install backports componets for bluetooth on x86 Linux
#v1.0

CAF_BT_URL="https://source.codeaurora.org/external/thundersoft/kernel/msm-3.10/patch/net/bluetooth/hci_core.c?id="

function get_source_patch
{
    BACKPORTS_PATH="http://www.kernel.org/pub/linux/kernel/projects/backports/stable/v3.12.8/backports-3.12.8-1.tar.xz"
    echo "===================================="
    echo " Downloading backports..."
    echo "===================================="
    wget ${BACKPORTS_PATH}
    tar -xf backports-3.12.8-1.tar.xz
    
    
    echo "===================================="
    echo " patch backports..."
    echo "===================================="

    rm -rf linux
    mkdir linux
    cd linux

    wget ${CAF_BT_URL}f38c173fe1d78dee7a9f6dd3aad661d42304150f -O 0001-Bluetooth-x86-Rome-driver-support.patch
    wget ${CAF_BT_URL}65dc80a6d9311f82a423a0d54d597ec08099b24c -O 0002-Bluetooth-Enable-auto-suspend-in-bt-usb-client-drive.patch
    wget ${CAF_BT_URL}9701a092dbba4907bebe582a9d060297cdefdbd3 -O 0003-bluetooth-Fix-SCO-connection-synchronization.patch
    wget ${CAF_BT_URL}8b88cdecaf81d50f1209df4c7a8981a79ba09446 -O 0004-bluetooth-Check-FW-status-before-downloading-the-fir.patch
    wget ${CAF_BT_URL}eb1bf87592f707f08a853bd31693638e5628e3bf -O 0005-bluetooth-Rename-BT-Firmware-files.patch
    wget ${CAF_BT_URL}eb448d5bb8c1592e6c1d5fed3a611a2516528843 -O 0006-bluetooth-Send-HCI-RESET-during-BT-OFF.patch
    wget ${CAF_BT_URL}194612b82a4fd770ae7a3d54a3bf6b8140b96483 -O 0007-Bluetooth-add-Qualcomm-In-Band-Sleep-support-to-hci_.patch
    wget ${CAF_BT_URL}ad71719ab5c47ec9c228faa1971726c73f9293f8 -O 0008-bluetooth-hci_ibs-Vote-on-off-UART-clocks-in-non-ato.patch
    wget ${CAF_BT_URL}fc7d4ab76583c4baaec72c2837c48e4c54cc2764 -O 0009-bluetooth-use-corect-struct-member-and-number-of-pro.patch
    wget ${CAF_BT_URL}a4d23c5f6e20ae73220fbdf9b06df806ddebfe6d -O 0010-bluetooth-hci_ibs-disable-irqs-when-spinlock-is-acqu.patch
    wget ${CAF_BT_URL}a3c6464f9c1cd29467b44ded1a8df5c2320348ca -O 0011-bluetooth-Configure-the-Tx-idle-timeout-to-1sec.patch
    wget ${CAF_BT_URL}3a5291901a59b7de7780a2a1f1164764c244f38c -O 0012-bluetooth-Type-cast-the-received-data-before-parsing.patch
    wget ${CAF_BT_URL}c8c6259b62964e35d8faad586d683e601162b088 -O 0013-bluetooth-Add-support-for-ROME-3.0.patch
    wget ${CAF_BT_URL}e2452ddd14b4f1d4e31fe1ea93df284ab6bb5e79 -O 0014-bluetooth-Configure-DUT-as-MASTER-for-outgoing-conne.patch
    wget ${CAF_BT_URL}52f33ca808d2d229bb10d3b47238aac2ff78996f -O 0015-bluetooth-Add-HCI-device-to-device-list-after-initia.patch
    wget ${CAF_BT_URL}33c5f3595a11d6510cd6719b9db5dfd2e309921c -O 0016-bluetooth-Check-for-NULL-pointers-in-input-args.patch
    wget ${CAF_BT_URL}bc3522c532147ba1258edec52f8c9f0ce1656149 -O 0017-bluetooth-Notify-connection-deletion-only-for-SCO-ES.patch
    wget ${CAF_BT_URL}d119b066225d8f86ace922626de35e5ad2243ef0 -O 0018-Bluetooth-Fix-ERTM-L2CAP-resend-packet.patch
    wget ${CAF_BT_URL}672feced02a0d645065e08e945ba2174675fb892 -O 0019-Bluetooth-Fix-crash-in-l2cap_chan_send-after-l2cap_c.patch
    wget ${CAF_BT_URL}6bc14767620b9ef79140de5fe73cd25006e2da5f -O 0020-Bluetooth-ath3k-don-t-use-stack-memory-for-DMA.patch
    wget ${CAF_BT_URL}209369c27258dcd1172af111c6a6c65179a09ef0 -O 0021-bluetooth-return-on-incorrect-version-or-firmware-do.patch
    wget ${CAF_BT_URL}460646929b23ff95827c404a43d2c2a9dd66f358 -O 0022-bluetooth-Add-support-for-Tufello-1.1.patch
    wget ${CAF_BT_URL}6de13bdbdeb971b7dc377dd508d2461b5728fb93 -O 0023-Bluetooth-kill-all-the-anchored-URBs-on-USB-device-d.patch
    wget ${CAF_BT_URL}f5c3608c67bccf4d7b6156506ce9ea2192fb21e8 -O 0024-Bluetooth-Avoid-Spurious-AMP-node-access-if-not-crea.patch
    wget ${CAF_BT_URL}c9b040a6fabdfcf2ae1c3334c192d94effa692d8 -O 0025-Bluetooth-change-cancel_delayed_work-to-cancel_delay.patch

    #because the macro is different between linux kernel and backport, remove the lines
    #in the patch files, and handle these by scripts
    sed -i '/Makefile$/,+9d' 0007-Bluetooth-add-Qualcomm-In-Band-Sleep-support-to-hci_.patch
    sed -i 's/CONFIG_BT_HCIUART_IBS/CPTCFG_BT_HCIUART_IBS/g' 0007-Bluetooth-add-Qualcomm-In-Band-Sleep-support-to-hci_.patch
    sed -i '/ath_init/,+11d' 0009-bluetooth-use-corect-struct-member-and-number-of-pro.patch
    
    cd ../backports-3.12.8-1
    patch -s -p1 <../linux/0001-Bluetooth-x86-Rome-driver-support.patch
    patch -s -p1 <../linux/0002-Bluetooth-Enable-auto-suspend-in-bt-usb-client-drive.patch
    patch -s -p1 <../linux/0003-bluetooth-Fix-SCO-connection-synchronization.patch
    patch -s -p1 <../linux/0004-bluetooth-Check-FW-status-before-downloading-the-fir.patch
    patch -s -p1 <../linux/0005-bluetooth-Rename-BT-Firmware-files.patch
    patch -s -p1 <../linux/0006-bluetooth-Send-HCI-RESET-during-BT-OFF.patch
    patch -s -p1 <../linux/0007-Bluetooth-add-Qualcomm-In-Band-Sleep-support-to-hci_.patch
    patch -s -p1 <../linux/0008-bluetooth-hci_ibs-Vote-on-off-UART-clocks-in-non-ato.patch
    patch -s -p1 <../linux/0009-bluetooth-use-corect-struct-member-and-number-of-pro.patch
    patch -s -p1 <../linux/0010-bluetooth-hci_ibs-disable-irqs-when-spinlock-is-acqu.patch
    patch -s -p1 <../linux/0011-bluetooth-Configure-the-Tx-idle-timeout-to-1sec.patch
    patch -s -p1 <../linux/0012-bluetooth-Type-cast-the-received-data-before-parsing.patch
    patch -s -p1 <../linux/0013-bluetooth-Add-support-for-ROME-3.0.patch
    patch -s -p1 <../linux/0014-bluetooth-Configure-DUT-as-MASTER-for-outgoing-conne.patch

#backports-3.12 >3.10, so 0015 and 0020 patches are not needed    
#patch -s -p1 <../linux/0015-bluetooth-Add-HCI-device-to-device-list-after-initia.patch
    patch -s -p1 <../linux/0016-bluetooth-Check-for-NULL-pointers-in-input-args.patch
    patch -s -p1 <../linux/0017-bluetooth-Notify-connection-deletion-only-for-SCO-ES.patch
    patch -s -p1 <../linux/0018-Bluetooth-Fix-ERTM-L2CAP-resend-packet.patch
    patch -s -p1 <../linux/0019-Bluetooth-Fix-crash-in-l2cap_chan_send-after-l2cap_c.patch
#patch -s -p1 <../linux/0020-Bluetooth-ath3k-don-t-use-stack-memory-for-DMA.patch
    patch -s -p1 <../linux/0021-bluetooth-return-on-incorrect-version-or-firmware-do.patch
    patch -s -p1 <../linux/0022-bluetooth-Add-support-for-Tufello-1.1.patch
    patch -s -p1 <../linux/0023-Bluetooth-kill-all-the-anchored-URBs-on-USB-device-d.patch
    patch -s -p1 <../linux/0024-Bluetooth-Avoid-Spurious-AMP-node-access-if-not-crea.patch
    patch -s -p1 <../linux/0025-Bluetooth-change-cancel_delayed_work-to-cancel_delay.patch

    
    sed -i '/hci_uart-objs/ i\hci_uart-$(CPTCFG_BT_HCIUART_IBS)  += hci_ibs.o' drivers/bluetooth/Makefile
    
} #end of get_source_patch

function build_install
{
    echo "===================================="
    echo "compile backports..."
    echo "===================================="
    
    cd backports-3.12.8-1
    
    #to make sure BT_HCIUART_IBS will befinded in autoconf.h
    sed -i '/BT_HCIUART=/i \BT_HCIUART_IBS' .local-symbols
    
    if [ ! -e .config ]; then
        echo "No .config; please prepare one or do make menuconfig before makeing backports"
        exit 1
    fi
    
    make
    
    
    
    echo "===================================="
    echo " install backports for bluetooth..."
    echo "===================================="
    sudo /etc/init.d/bluetooth stop 
    sudo rmmod bnep
    sudo rmmod btusb
    sudo rmmod ath3k
    sudo rmmod rfcomm
    sudo rmmod hci_uart
    sudo rmmod btsdio
    sudo rmmod bluetooth
    #if compat is installed
    sudo rmmod compat
    
    sudo insmod compat/compat.ko
    sudo insmod net/bluetooth/bluetooth.ko
    sudo insmod net/bluetooth/rfcomm/rfcomm.ko
    sudo insmod net/bluetooth/bnep/bnep.ko
    sudo insmod drivers/bluetooth/hci_uart.ko
    sudo insmod drivers/bluetooth/ath3k.ko
    sudo insmod drivers/bluetooth/btusb.ko
    sudo insmod drivers/bluetooth/btsdio.ko
    
} #end of build_install

function usage
{
    echo "============================================================================"
    echo "usage: ./backports.sh [ [-h] | [-g | -get] | [-i | -install] ]"
    echo "                      -h            to show this usage"
    echo "                      -g,-get       to download backports source and patches"
    echo "                      -i,-install   to build and install backports"
    echo "============================================================================"
    
}

##### Main

case $1 in
    -h | -help )           usage
                           exit
                           ;;
    -g | -get )            get_source_patch
                           exit
                           ;;
    -i | -install )        build_install
                           exit
                           ;;
    * )                    usage
                           exit 1
esac
