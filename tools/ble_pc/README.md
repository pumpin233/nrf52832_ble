# 电脑通过自身蓝牙连接开发板

1. 用 Keil 编译并下载本工程的新固件，复位开发板。
2. 打开电脑蓝牙，断开手机与开发板的连接。
3. 双击 `start.bat`，浏览器会打开本地收发窗口。
4. 点击“连接开发板”，连接成功后输入内容并点击“发送”。
5. 开发板收到数据后原样通知回传，接收框应显示同样的内容。

新设备名为 `BLE_PC_Link`，无需在 Windows 设置里先配对，也不使用 COM14。
窗口由 Python 在本机运行，不需要浏览器支持 Web Bluetooth。
当前电脑的 Python 已安装 bleak；换电脑后使用 Python 3.10+ 安装
`python -m pip install -r requirements.txt`，再运行 `python main.py`。
`start.bat` 中的 Python 路径是本机路径，换电脑时需要修改。

固件使用 NUS 服务 `6e400001-b5a3-f393-e0a9-e50e24dcca9e`。
RX 特征末段编号为 0002（电脑写入），TX 为 0003（电脑接收通知）。
电脑程序自动开启 TX 通知，并按20字节拆包；收到的文本会连续拼接。
回传保持原始字节，不增加回车，不再执行 pum 控灯或实体 UART 回显。
触发板子的 `BSP_EVENT_KEY_3` 按键事件，也会发送 `Board button\r\n`。

开发板的发送入口是 `ble_pc_send(data, length)`，当前在蓝牙和按键回调里调用。
它返回 false 表示通知未开启或1023字节可用缓存已满；缓存满会写 RTT 警告。
此例用于低速交互验证，不承诺持续高速发送时无丢包。
原主程序备份位于 `app_template` 同级实际源码目录里的 `main.c.before_ble_pc.bak`。

蓝牙API依据：https://bleak.readthedocs.io/en/latest/api/client.html
