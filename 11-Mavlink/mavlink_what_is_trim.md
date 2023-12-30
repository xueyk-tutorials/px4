# 截断功能

## 说明

​		对于payload是变长的消息来说（例如gps_inject_data，其payload长度为113），有时其payload只有少量字节，如果每次还按照最大长度传输，就非常占用带宽。

​		mavlin v2提供了payload截断功能，在序列化之前，将把payload所有以0x00的尾部部分删除，并重新计算包长度。

## 源码分析

​		获取消息结构体之后，使用mavlink_msg_to_send_buffer函数进行序列化

![image-20230910140128702](imgs\image-20230910140128702.png)



​		通过查看`_mav_trim_payload()`函数可知，其确实将payload结尾是0x00的空字节去掉了。

```c
/**
 * @brief Trim payload of any trailing zero-populated bytes (MAVLink 2 only).
 *
 * @param payload Serialised payload buffer.
 * @param length Length of full-width payload buffer.
 * @return Length of payload after zero-filled bytes are trimmed.
 */
MAVLINK_HELPER uint8_t _mav_trim_payload(const char *payload, uint8_t length)
{
	while (length > 1 && payload[length-1] == 0) {
		length--;
	}
	return length;
}
```

## 使用注意

​		**每次调用消息的pack函数前，将payload的buffer清空置0**，避免如果上次发送的是较长的payload，造成残留尾部对后面消息序列号截断判断产生影响。

## 测试

```c
void mavlink_test() {
    uint8_t buf_rtca[180];
    uint8_t buffer_send1[256];
    uint8_t buffer_send2[256];

    mavlink_message_t msg_send1{}, msg_send2{}, msg_recv1{};
    rt_memset(&msg_send1, 0x00, sizeof(msg_send1));
    rt_memset(&msg_send2, 0x00, sizeof(msg_send2));

    mavlink_gps_inject_data_t data_send1{}, data_send2{}, data_recv{};
    rt_memset(data_send1.data, 0x00, sizeof(data_send1.data));
    rt_memset(data_send2.data, 0x00, sizeof(data_send2.data));

    for (int i = 0; i < 8;++i) {
        data_send1.data[i] = i;
        buf_rtca[i]       = i;
    }
    mavlink_msg_gps_inject_data_encode_chan(1, 1, 0, &msg_send1, &data_send1);
    msg_send1.magic = MAVLINK_STX;                         /// 强制使用mavlink v2
    mavlink_msg_to_send_buffer(buffer_send1, &msg_send1);  /// 序列化后buffer[1]=payload_len=11

    mavlink_msg_gps_inject_data_pack_chan(1, 1, 0, &msg_send2, 10, 10, 8, buf_rtca);
    msg_send2.magic = MAVLINK_STX_MAVLINK1;                /// 强制使用mavlink v1
    mavlink_msg_to_send_buffer(buffer_send2, &msg_send2);  /// 序列化后buffer[1]=payload_len=113
    
}
```

