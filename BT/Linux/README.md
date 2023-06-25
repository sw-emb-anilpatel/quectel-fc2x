# Qualcomm Linux Bluetooth



# 1 Bluez hciattach



## 1.1 ����patch

0001-hciattach-add-support-for-qualcomm-chip.patch



## 1.2 ����bluez

//TO-DO �������ע������



## 1.3 hciattach

```
sudo hciattach /dev/ttyUSB0 qca -t120 3000000 flow
```



## 1.4 hciconfig ȷ��������port

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



# 2 ����btdiag

## 2.1 ����

```
BT/linux/qcom/tool/btdiag/src/# make
```



## 2.1 btdiagʹ��

*ע���޸�BT-DEVICE*

```
sudo ./Btdiag UDT=yes PORT=2390 IOType=USB QDARTIOType=ethernet BT-DEVICE=hci1

Run in UDT mode
Connected to SoC DUT!

Thread created successfully
Socket created
bind done
Waiting for incoming connections...
```

# 3 xtt�ļ�



| �ļ�                       | ����                                           |
| -------------------------- | ---------------------------------------------- |
| QCA206x_WLAN_TX_RX_FTM.xtt | ������Ƶ���ԣ�wifi��������ַ��board id�Ķ�д�� |
|                            |                                                |
|                            |                                                |

