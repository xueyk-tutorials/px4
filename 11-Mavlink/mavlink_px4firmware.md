# PX4飞控Mavlink模块

以V1.12.3为例子。

## 头文件结构

### mavlink v2.0库

以下是mavlink/v2.0文件夹内头文件的包含关系：

```c++
/* 标准mavlink头文件 */
|-- "standard/mavlink.h"
    |-- "version.h"
    |-- "standard.h"
        |-- "../protocol.h"
        |-- "../common/common.h"
        |-- "../mavlink_get_info.h"

|-- "common/common.h"
    |-- "../protocol.h"
    |-- // enum定义如MAV_MODE、MAV_CMD等
    |-- "./mavlink_msg_xxx.h"     //所有message定义头文件（common文件夹内）
    |-- "../minimal/minimal.h"    //最基础message部分（minimal文件夹内）
    	|-- "./mavlink_msg_heartbeat.h"
    	|-- "./mavlink_msg_protocol_version.h"
    |-- #define MAVLINK_MESSAGE_INFO   // 数组，包含所有消息ID宏
    |-- #define MAVLINK_MESSAGE_NAMES  // 数组，包含所有消息对应的名称与结构体大小
    |-- "../mavlink_get_info.h"
    
|-- "protocol.h"
	|-- "mavlink_types.h"
    |-- "mavlink_helpers.h"
    	|-- "checksum.h"
    	|-- "mavlink_types.h"
    	|-- "mavlink_conversions.h" // 四元数、DCM、Euler互相转换
    	|-- "mavlink_sha256.h"
```

### MAVLink模块

#### 引入mavlink库

通过头文件`mavlink_bridge_header.h`包含mavlink库，其包含的头文件如下：

```c
|-- "mavlink_bridge_header.h"
	|-- "mavlink_types.h"
    |-- "standard/mavlink.h"
```

#### 模块内



## 代码

代码所在文件夹：src/modules/mavlink

mavlink_main.cpp

## 基本框架

### 操作设备发送mavlink消息

#### mavlink发送数据过程

mavlink V2.0库中提供了消息打包和发送函数，函数格式为：`mavlink_msg_<message type>_send_struct`，如姿态消息发送定义如下：

```c
static inline void mavlink_msg_attitude_send_struct(mavlink_channel_t chan, const mavlink_attitude_t* attitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_attitude_send(chan, attitude->time_boot_ms, attitude->roll, attitude->pitch, attitude->yaw, attitude->rollspeed, attitude->pitchspeed, attitude->yawspeed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE, (const char *)attitude, MAVLINK_MSG_ID_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_LEN, MAVLINK_MSG_ID_ATTITUDE_CRC);
#endif
}
```

其中，`_mav_finalize_message_chan_send`函数调用了`_mavlink_send_uart`，

```c++
MAVLINK_HELPER void _mav_finalize_message_chan_send(mavlink_channel_t chan, uint32_t msgid,
                                                    const char *packet, 
						    uint8_t min_length, uint8_t length, uint8_t crc_extra)
{
    /* 准备数据*/
    // ...
    // ...
    
    /* 发送数据*/
    // 开始发送
    MAVLINK_START_UART_SEND(chan, header_len + 3 + (uint16_t)length + (uint16_t)signature_len);
    // 发送数据
	_mavlink_send_uart(chan, (const char *)buf, header_len+1);
	_mavlink_send_uart(chan, packet, length);
	_mavlink_send_uart(chan, (const char *)ck, 2);
	if (signature_len != 0) {
		_mavlink_send_uart(chan, (const char *)signature, signature_len);
	}
    // 结束发送
	MAVLINK_END_UART_SEND(chan, header_len + 3 + (uint16_t)length + (uint16_t)signature_len);
}
```

可以看到，发送的过程涉及到两个宏和一个函数：MAVLINK_START_UART_SEND、_mavlink_send_uart、MAVLINK_END_UART_SEND。

而`_mavlink_send_uart`函数调用了宏`MAVLINK_SEND_UART_BYTES`，代码如下：

```c++
MAVLINK_HELPER void _mavlink_send_uart(mavlink_channel_t chan, const char *buf, uint16_t len)
{
#ifdef MAVLINK_SEND_UART_BYTES
	/* this is the more efficient approach, if the platform
	   defines it */
	MAVLINK_SEND_UART_BYTES(chan, (const uint8_t *)buf, len);
#else
	/* fallback to one byte at a time */
	uint16_t i;
	for (i = 0; i < len; i++) {
		comm_send_ch(chan, (uint8_t)buf[i]);
	}
#endif
}
```

所以，MAVLINK_SEND_UART_BYTES、MAVLINK_START_UART_SEND、MAVLINK_END_UART_SEND三个宏就是发送的关键！！！

#### 用户自定义发送

三个宏其实是由**用户**定义的函数！

> 那如何解决用户定义宏并由mavlink库使用呢？
>
> 由于mavlink库都是头文件，可以通过头文件包含顺序实现用户“提前”定义宏，由mavlink库引用。
>
> 在用户定义的头文件mavlink_bridge_header.h中：
>
> ```
> // 定义宏（在前）
> #define MAVLINK_SEND_UART_BYTES mavlink_send_uart_bytes
> 
> // 包含mavlink头文件（在后，这样mavlink头文件就可以使用宏了）
> #include <v2.0/standard/mavlink.h>
> ```



在mavlink_bridge_header.h中代码如下：

```c++
/* 定义宏，绑定函数 */
#define MAVLINK_SEND_UART_BYTES mavlink_send_uart_bytes

#define MAVLINK_START_UART_SEND mavlink_start_uart_send
#define MAVLINK_END_UART_SEND mavlink_end_uart_send

// 通过include包含mavlink库
#include <v2.0/mavlink_types.h>

/**
 * .......................
 */

/* 声明函数 */
void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length);
void mavlink_start_uart_send(mavlink_channel_t chan, int length);
void mavlink_end_uart_send(mavlink_channel_t chan, int length);

extern mavlink_status_t *mavlink_get_channel_status(uint8_t chan);
extern mavlink_message_t *mavlink_get_channel_buffer(uint8_t chan);

// 通过include包含mavlink库
#include <v2.0/standard/mavlink.h>
```



函数的具体定义与实现在mavlink_main.cpp中，代码如下：

```c++
#include "mavlink_main.h"   // 包含了#include "mavlink_command_sender.h"，再包含#include "mavlink_bridge_header.h"

void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length) { mavlink_module_instances[chan]->send_bytes(ch, length); }

void mavlink_start_uart_send(mavlink_channel_t chan, int length) { mavlink_module_instances[chan]->send_start(length); }

void mavlink_end_uart_send(mavlink_channel_t chan, int length) { mavlink_module_instances[chan]->send_finish(); }
```

以上三个函数又分别调用了send_bytes、send_start、send_finish！！！

- send_bytes()函数，进行数据拷贝和处理

```c++
void Mavlink::send_bytes(const uint8_t *buf, unsigned packet_len)
{
	if (!_tx_buffer_low) {
		if (_buf_fill + packet_len < sizeof(_buf)) {
			memcpy(&_buf[_buf_fill], buf, packet_len);
			_buf_fill += packet_len;

		} else {
			perf_count(_send_byte_error_perf);
		}
	}
}
```

- Mavlink::send_start(int length)，开始发送数据的准备

- Mavlink::send_finish()函数，**包括了串口与网口发送！！！**



### 接收



## MavlinkStream类

### StreamListItem

定义了三个成员：

- new_instance函数指针
- name
- id

## Mavlink类

### configure_stream
配置单个消息流


### configure_streams_to_default
配置默认的消息流，根据MAVLINK_TYPE，将需要用到的消息流添加进来。
- MAVLINK_TYPE== MAVLINK_MODE_NORMAL
    包含的消息流有：ADSB_VEHICLE、ALTITUDE、ATTITUDE等。
- MAVLINK_TYPE==MAVLINK_MODE_ONBOARD
    包含的消息流有：TIMESYNC、CAMERA_TRIGGER、HIGHRES_IMU。