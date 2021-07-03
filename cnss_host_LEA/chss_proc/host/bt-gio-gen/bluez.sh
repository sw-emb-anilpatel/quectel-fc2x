#!/bin/bash
#bluez.sh A script to download bluez source code, caf patches, build and install bluez on x86 Linux
#v1.0

function get_source_patch
{
    echo ==========================
    echo Download bluez source code 
    echo ==========================
    #sudo apt-get install git -y
    rm -rf ./bluez
    git clone git://git.kernel.org/pub/scm/bluetooth/bluez.git
    if [ "$?" != "0" ]; then
            echo  NOT able to clone the Bluez Code
            exit 1
    fi
    cd bluez
    if [ "$?" != "0" ]; then
            echo  Not able to create bluez Source Code.
            exit 1
    fi
    git checkout tags/5.19
    if [ "$?" != "0" ]; then
            echo  Not able to pointing to Bluez5.19
            exit 1
    fi
    
    git remote add caf git://codeaurora.org/quic/le/platform/external/bluetooth/bluez
    if [ "$?" != "0" ]; then
            echo  Not able to add caf remote
            exit 1
    fi
    
    git remote update
    if [ "$?" != "0" ]; then
            echo  Not able to update caf remote 
            exit 1
    fi

    echo ==============================
    echo Download and apply caf patches
    echo ==============================
    git cherry-pick bb96f3b759e0b99db70014302ca12929fb42f554
    git cherry-pick 6d13cb1ce518b13ee413f58ed7eaa0b57952f902
    git cherry-pick 0410fb881dafca2f98356ecd4f328f0c6b7d978f
    git cherry-pick 377fc8d9c6581ada5630556463501179bda9a21b
    git cherry-pick 3690828e27818525f91508452cf3880e8d616e31
    git cherry-pick 4f4ae148dd494de4fbdfe381c67807c83c4ca9bb
    git cherry-pick d61a530efd1af188675735b193ddfc669e0d8278
    git cherry-pick 6bf24205c90dc7ec29b39745037879bfba51bc83
    git cherry-pick 028a95c3100039f182826811d91c9dde90dbe7dc
    git cherry-pick 24864a7661013826711c234f742e0267f1e0036a
    git cherry-pick 1e603f836e795767ebc1a9bb3867ecda906a3abe
    git cherry-pick f17e6f316c3e3466686df1b34d10da02da12cd90
    git cherry-pick 8bbd9ed42847fc8f301fc3302b248979b9729bee
    git cherry-pick a67b0480a2c45b0d11e4db4142fc95fb36287dda
    
    echo ====================
    echo Generate patch files
    echo ====================
    git format-patch -14
} #end of get_source_patch
    
function build_install
{
    echo ===================================
    echo Installing Dependency Libraries....
    echo ===================================
    
    cd bluez

    sudo apt-get install libgtk2.0-dev -y
    if [ "$?" != "0" ]; then
    	echo  Failed to install libgtk2.0-dev
    	exit 1
    fi
    sudo apt-get install dbus-*dev -y
    if [ "$?" != "0" ]; then
            echo  Failed to install dbus-*dev
            exit 1
    fi
    sudo apt-get install libical-dev -y
    if [ "$?" != "0" ]; then
            echo  Failed to install libical-dev
            exit 1
    fi
    sudo apt-get install libreadline-dev -y
    if [ "$?"!= "0" ]; then
            echo Failed to install libreadline-dev
    	exit 1
    fi
    sudo apt-get install libtool -y
    if [ "$?" != "0" ]; then
            echo  Failed to install libtool
            exit 1
    fi
    sudo apt-get install automake1.11 -y
    if [ "$?" != "0" ]; then
            echo  Failed to install automake1.11
            exit 1
    fi
    sudo apt-get install libudev-dev -y
    if [ "$?" != "0" ]; then
            echo Failed to install libudev-dev
    	exit 1
    fi
    
    libtoolize --force 
    if [ "$?" != "0" ]; then
            echo  Failed to do libtoolize
            exit 1
    fi
    aclocal 
    if [ "$?" != "0" ]; then
            echo  Failed to do aclocal
            exit 1
    fi
    autoheader 
    if [ "$?" != "0" ]; then
            echo  Failed to autoheader
            exit 1
    fi
    automake --force-missing --add-missing 
    if [ "$?" != "0" ]; then
            echo  Failed to automake
            exit 1
    fi
    autoconf 
    if [ "$?" != "0" ]; then
            echo  Failed to do automake
            exit 1
    fi

    echo ======================
    echo make and install bluez
    echo ======================
    ./configure --prefix=/usr --mandir=/usr/share/man --sysconfdir=/etc --localstatedir=/var --enable-experimental --with-systemdsystemunitdir=/lib/systemd/system --with-systemduserunitdir=/usr/lib/systemd 
    if [ "$?" != "0" ]; then
            echo  Failed to do ./configure
            exit 1
    fi
    sudo make 
    if [ "$?" != "0" ]; then
            echo  Failed to do make
            exit 1
    fi
    
    
    sudo make install
    if [ "$?" != "0" ]; then
            echo Failed to do make install
            exit 1
    fi
    
    sudo rm -rf /usr/sbin/bluetoothd
    if [ "$?" != "0" ]; then
            echo  Unable to remove Previous Bluetoothd file.
            exit 1
    fi
    sudo cp /usr/libexec/bluetooth/bluetoothd /usr/sbin/
    if [ "$?" = "0" ]; then
    	echo INSTALLATION is done.
    else
            echo  Failed to Failed to copy /usr/libexec/bluetooth/bluetoothd to /usr/sbin/
            exit 1
    fi
} #end of build_install

function usage
{
    echo "=================================================================="
    echo "usage: ./bluez.sh [ [-h] | [-g | -get] | [-i | -install] ]"
    echo "                  -h            to show this usage"
    echo "                  -g,-get       to download bluez source and patches"
    echo "                  -i,-install   to build and install bluez"
    echo "=================================================================="
} #end of usage

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


