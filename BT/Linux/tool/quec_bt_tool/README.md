# **btconfig**



## **1️⃣ 编译方法:**

```
cd quec_bt_tool
make
```



## **2️⃣ 目录结构：**

| 目录文件夹 | 子目录文件夹 | 说明                   |
| ---------- | ------------ | ---------------------- |
| src        |              | btconfig源码           |
| firmware   |              | 蓝牙固件               |
| doc        |              | btconfig相关的说明文档 |
|            |              |                        |



## **3️⃣  项目配置**

| 文件       | 修改                                    |
| ---------- | --------------------------------------- |
| hci_uart.h | 修改当前使用的tty设备 BT_HS_UART_DEVICE |
|            |                                         |
|            |                                         |



## **4️⃣  支持模块：**

| 模块      | 芯片            | 固件                        | 状态 |
| --------- | --------------- | --------------------------- | ---- |
| FC20/FC21 | QCA1023/QCA9377 | tfbtfw11.tlv & tfbtnv11.bin | ✅    |
| FC50V     | QCA1062         | hpbtfw20.tlv & htnv20.bin   | ✅    |
| AF50T     | QCA6696         | hpbtfw20.tlv & htnv20.bin   | ✅    |
|           |                 |                             |      |
|           |                 |                             |      |
|           |                 |                             |      |



## 5️⃣ 支持平台

| 模块     | 状态 | 备注                                                         |
| -------- | ---- | ------------------------------------------------------------ |
| Ubuntu   | ✅    | /data # ./btconfig download<br/>/data # ./btconfig reset     |
| EC20平台 | ✅    | /data # echo 0  >/sys/class/rfkill/rfkill0/state<br/>/data # echo 1  >/sys/class/rfkill/rfkill0/state<br/>/data # ./btconfig download<br/>/data # ./btconfig reset |
|          |      |                                                              |



## 6️⃣ 命令详解

| 命令 | 使用 | 描述 |
| ---- | ---- | ---- |
|      |      |      |
|      |      |      |
|      |      |      |
|      |      |      |
|      |      |      |
|      |      |      |

