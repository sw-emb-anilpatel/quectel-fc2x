# Qualcomm Linux Bluetooth



# 1 Bluez hciattach



## 1.1 合入patch

0001-hciattach-add-support-for-qualcomm-chip.patch



## 1.2 编译bluez

//TO-DO 补充编译注意事项



## 1.3 hciattach

```
sudo hciattach /dev/ttyUSB0 qca -t120 3000000 flow
```



## 1.4 hciconfig 确认蓝牙的port

```
sudo hciconfig -a 

hci1:   Type: Primary  Bus: UART
        BD Address: 00:00:00:00:5A:AD  ACL MTU: 1024:7  SCO MTU: 240:8
        UP RUNNING PSCAN ISCAN
        RX bytes:1244 acl:0 sco:0 events:79 errors:0
        TX bytes:3348 acl:0 sco:0 commands:78 errors:0
        Features: 0xff 0xfe 0x8f 0xfe 0xd8 0x3f 0x7b 0x87
        Packet type: DM1 DM3 DM5 DH1 DH3 DH5 HV1 HV2 HV3
        Link policy: RSWITCH HOLD SNIFF
        Link mode: PERIPHERAL ACCEPT
        Name: ''
        Class: 0x000000
        Service Classes: Unspecified
        Device Class: Miscellaneous,
        HCI Version:  (0xc)  Revision: 0x0
        LMP Version:  (0xc)  Subversion: 0x3a98
        Manufacturer: Qualcomm (29)

```



# 2 启动btdiag

## 2.1 编译

```
BT/linux/qcom/tool/btdiag/src/# make
```



## 2.1 btdiag使用

*注意修改BT-DEVICE*

```
sudo ./Btdiag UDT=yes PORT=2390 IOType=USB QDARTIOType=ethernet BT-DEVICE=hci1

Run in UDT mode
Connected to SoC DUT!

Thread created successfully
Socket created
bind done
Waiting for incoming connections...
```

# 3 xtt文件



| 文件                       | 作用                                           |
| -------------------------- | ---------------------------------------------- |
| QCA206x_WLAN_TX_RX_FTM.xtt | 用于射频测试，wifi，蓝牙地址和board id的读写， |
|                            |                                                |
|                            |                                                |

