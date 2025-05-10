HID BTSSP NFC例程说明：
1. 该例程通过模拟NFC FORUM TYPE2的NFC卡片，存储蓝牙信息，支持的设备通过刷卡即可连接指定设备。
2. 例程使用了优化的ECC算法，默认代码运行在flash中，如果对速度有要求，可以将代码放入highcode中，详见Enc/ble_sm_alg.c。
3. 如果使能了HAL_SLEEP，则需要在SLEEP.c的CH58x_LowPower函数中，在睡眠之前调用nfca_picc_stop，在唤醒后调用nfca_picc_start。
