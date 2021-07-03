1.The code can also be cloned by following two steps:

•git clone ssh://git.quicinc.com:29418/platform/vendor/qcom-opensource/bluetooth -b CNSS.LEA_NPL.1.0 ./bluetooth/
•git fetch ssh://review-android.quicinc.com:29418/platform/vendor/qcom-opensource/bluetooth refs/changes/28/2262928/4 && git cherry-pick FETCH_HEAD



For Linux/UART:
 
1. Do a 'make' in the root folder
2. Do chmod +x run_UART_Btdiag.sh (edit this file for changing the QDARTIOType, default is 'ethernet')
3. Run ./run_UART_Btdiag.sh

For Android/UART:
 
1. Do source build/envsetup.sh
2. lunch android_x86_64-userdebug (for 64-bit)  or lunch android_x86-userdebug (for 32-bit)
3. export JACK_SERVER_VM_ARGUMENTS="-Xmx4g -Dfile.encoding=UTF-8 -XX:+TieredCompilation"
4. Do mmm vendor/qcom/opensource/bluetooth/BtDiag on the root folder in the Windows machine which generates a Btdiag file in the out folder. (For now it is already included in the folder)
5. Copy Btdiag file into system/bin/ in the Android machine.
6. Do not turn on the Bluetooth from the system UI before running the tool. (It messes up the board preventing it from downloading the packet).
7. Run "Btdiag UDT=yes PORT=2390 IOType=SERIAL QDARTIOType=ethernet BT-DEVICE=/dev/ttyUSB0 BT-BAUDRATE=115200" (edit for changing the QDARTIOType, default is 'ethernet')