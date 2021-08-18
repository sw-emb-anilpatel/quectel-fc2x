Steps to build the Nan Demo Tool

For Android:
===========

1) copy Nantool SRC to vendor/qcom/proprietary/wlan/utils/

2) setup build environment for Andorid. for e.g. for MSM8996 it is
        a. export AUTO_TREE_ID=msm8996
        b. source build/envsetup.sh
        c. lunch

3) Goto vendor/qcom/proprietary/wlan/utils/utils/Nantool/SRC/LIB folder

4) Run mm to build "libnanwifihal.so"

5) Copy "libnanwifihal.so.toc" and "libwifihal.so" to out/target/product/<platform>/obj/lib/ folder.
       a. The 64 bit version of the "libwifihal.so" will be in
          out\target\product\<platform>\vendor\lib64 folder.
       b. "libnanwifihal.so.toc" file will be located in
          out\target\product\<platform>\obj_arm\SHARED_LIBRARIES\libnanwifihal_intermediates

6) Goto vendor/qcom/proprietary/wlan/utils/utils/Nantool/SRC/ folder

7) Run mma to build nantool executable for Android

8) While executing nantool, make sure "libwifihal.so" is in /system/lib64/ folder


FOR LINUX:
=========

1) Make sure libnl-3-dev and libbsd-dev packages
 are installed in the pc

2) Build the shared LIB (libwifi-hal-qcom.so)
       
     cd LIB
       
     make

3) Export the shared lib path by doing
     
    export LD_LIBRARY_PATH=<YOUR_NAN_TOOLPATH>/SRC/LIB/
                    
4) Build the NanDemoTool
 
    cd ..

    make

NOTE: if the shared library path is not set in the shell, Nan Demo tool 
will not execute.
