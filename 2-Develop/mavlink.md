# mavlink

## 功能需求



## 介绍

代码所在文件夹：src/modules/mavlink，主要的几个文件如下：

- mavlink_main.cpp
- mavlink_receiver.cpp

mavlink通过shell脚本中添加命令进行启动，如果脚本中添加多条启动命令，将启动多个mavlink实例，每一个mavlink实例都对应一路硬件（如UDP、串口）。

每个malink实例包括两个线程，一个主线程，一个接收线程。

## 启动过程

### 入口函数

```c++
extern "C" __EXPORT int mavlink_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		return Mavlink::start(argc, argv);

	} else if (!strcmp(argv[1], "stop")) {
		PX4_WARN("mavlink stop is deprecated, use stop-all instead");
		usage();
		return 1;

	} else if (!strcmp(argv[1], "stop-all")) {
		return Mavlink::destroy_all_instances();

	} else if (!strcmp(argv[1], "status")) {
		bool show_streams_status = argc > 2 && strcmp(argv[2], "streams") == 0;
		return Mavlink::get_status_all_instances(show_streams_status);

	} else if (!strcmp(argv[1], "stream")) {
		return Mavlink::stream_command(argc, argv);

	} else if (!strcmp(argv[1], "boot_complete")) {
		Mavlink::set_boot_complete();
		return 0;

	} else {
		usage();
		return 1;
	}

	return 0;
}
```

### 仿真脚本

build/px4_sitl_default/etc/init.d-posix/rcS

```shell
#user defined mavlink streams for instances can be in PATH
. px4-rc.mavlink
```

build/px4_sitl_default/etc/init.d-posix/px4-rc.mavlink

通过mavlink模块提供的启动参数进行。

```shell
#!/bin/sh
# shellcheck disable=SC2154

echo ">>>>> Alex: start mavlink"

udp_offboard_port_local=$((14580+px4_instance))
udp_offboard_port_remote=$((14540+px4_instance))
[ $px4_instance -gt 9 ] && udp_offboard_port_remote=14549 # use the same ports for more than 10 instances to avoid port overlaps
udp_onboard_payload_port_local=$((14280+px4_instance))
udp_onboard_payload_port_remote=$((14030+px4_instance))
udp_onboard_gimbal_port_local=$((13030+px4_instance))
udp_onboard_gimbal_port_remote=$((13280+px4_instance))
udp_gcs_port_local=$((18570+px4_instance))

# GCS link
mavlink start -x -u $udp_gcs_port_local -r 4000000 -f
mavlink stream -r 50 -s POSITION_TARGET_LOCAL_NED -u $udp_gcs_port_local
mavlink stream -r 50 -s LOCAL_POSITION_NED -u $udp_gcs_port_local
mavlink stream -r 50 -s GLOBAL_POSITION_INT -u $udp_gcs_port_local
mavlink stream -r 50 -s ATTITUDE -u $udp_gcs_port_local
mavlink stream -r 50 -s ATTITUDE_QUATERNION -u $udp_gcs_port_local
mavlink stream -r 50 -s ATTITUDE_TARGET -u $udp_gcs_port_local
mavlink stream -r 50 -s SERVO_OUTPUT_RAW_0 -u $udp_gcs_port_local
mavlink stream -r 20 -s RC_CHANNELS -u $udp_gcs_port_local
mavlink stream -r 10 -s OPTICAL_FLOW_RAD -u $udp_gcs_port_local

# API/Offboard link
mavlink start -x -u $udp_offboard_port_local -r 4000000 -f -m onboard -o $udp_offboard_port_remote

# Onboard link to camera
mavlink start -x -u $udp_onboard_payload_port_local -r 4000 -f -m onboard -o $udp_onboard_payload_port_remote

# Onboard link to gimbal
mavlink start -x -u $udp_onboard_gimbal_port_local -r 400000 -m gimbal -o $udp_onboard_gimbal_port_remote

```

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

## 基本框架

### 发送

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

在mavlink初始化完成前，创建接收线程。代码文件为mavlink_receiver.cpp。





### 转发

mavlink模块可以创建多个实例，每个实例可以认为是一路消息处理通道，也就是要打开一个串口或网口用于消息通信。为了实现不同通道直接的数据交互，malvink模块提供了转发机制，这样能够实现消息的跨实例通信（进而实现跨通道通信）。

那它有什么用呢？

比如有飞控连接了地面站和一个吊舱，一个mavlink实例通过UDP连接地面站，一个mavlink实例通过串口连接吊舱，如果我们希望地面站与吊舱的通信该怎么办呢？由于二种没有物理连接通道，这个时候就可以通过转发机制来实现了。

#### 基本机制

实例自身接收消息后，进行转发处理，遍历除自身外所有的mavlink实例，将要转发的消息**写到**对方的缓冲数组中。

实例自身发送消息时，检查是否需要转发，如果需要转发则从自身的缓冲数组中**读取**数据，然后发送。

假如有两个mavlink实例A和B开启了转发，则消息转发流程如下图所示：

![](imgs\mavlink_forwarding1.png)

其中接收消息由mavlink_receiver线程实现，发送消息由mavlink主线程实现。

**在实际使用中，必须同时开启两个实例的转发功能才能实现双向转发，如果只有一个实例开启了转发，那么消息只从这一个实例单向流向其他实例！**

#### 缓冲数组

转发数据需要一个ringbuffer，定义缓冲数组结构如下：

```c
    struct mavlink_message_buffer {
        int   write_ptr;
        int   read_ptr;
        int   size;
        char *data;
    };
```

各成员变量如下：

- write_ptr：写数据位置，也就是上次写完数据的末尾位置；
- read_ptr：读数据位置，也就是从该位置开始取数据，一直取到write_ptr位置结束；

缓冲数组初始化

- 初始化读写指针为0
- 开辟大小为buffer size=2 * sizeof(mavlink_message_t) + 1)的缓冲数组

**写数据**

写数据时根据当前write_ptr位置与待写入的字节数大小data size，分两种情况：1）要写入的数据需要分首尾两部分分块写入；2）要写入的数据一整块写入。

- 分块写入

  要写入的字节大小以及当前write_ptr指向位置表示如下：

  ![](imgs\mavlink_forwarding_write1.png)

  则由于`buffer_size-write_ptr < data_size`，故需要拆开两块来写，第一块填满末尾，剩下的第二块从头写入：

  ![](imgs\mavlink_forwarding_write2.png)

  第一块写入的字节数为n=buffer_size-write_ptr，第二块写入的字节数为p=data_size-n。

- 整块写入

  要写入的字节大小以及当前write_ptr指向位置表示如下：

  ![](imgs\mavlink_forwarding_write3.png)

  可以看到`buffer_size-write_ptr > data_size`，剩余空间足够，一次性写入

  ![](imgs\mavlink_forwarding_write4.png)



**读数据**

读数据也分两种情况：1）缓冲数组存放的数据连续一整块，2）不连续，分为了两块。

- 一整块

  这种情况下write_ptr在前，read_ptr在后

  ![](imgs\mavlink_forwarding_read1.png)

  可以判断出要读取的字节数

  ![](imgs\mavlink_forwarding_read2.png)

  读取完后，移动read_ptr至读取的结尾。

- 两块

  情况如下：

  ![](imgs\mavlink_forwarding_read3.png)

  首先第一次读到结尾

  ![](imgs\mavlink_forwarding_read4.png)

  然后再读一次

  ![](imgs\mavlink_forwarding_read5.png)



> 注意:
>
> 在实际代码中，写入数据不论那种情况，都是调用一次写函数完成！而读数据时，如果数据分块存放，则需要调用两次读函数完成数据读取操作。

## 配置

### datarate

最大速率

```c
#define MAX_DATA_RATE                  10000000
```



#### UDP网络通信宏定义

```c++
#if defined(CONFIG_NET) || defined(__PX4_POSIX)
# define MAVLINK_UDP
# define DEFAULT_REMOTE_PORT_UDP 14550 ///< GCS port per MAVLink spec
#endif // CONFIG_NET || __PX4_POSIX
```



## 与orb相关

mavlink_log_info()：打印字符串并通过uorb发送至mavlink模块，进而转发至地面站。

```c++
#define mavlink_log_info(_pub, _text, ...)                             \
    do {                                                               \
        mavlink_vasprintf(_MSG_PRIO_INFO, _pub, _text, ##__VA_ARGS__); \
        LOG_I(_text, ##__VA_ARGS__);                                   \
    } while (0);
```

mavlink_vasprintf()：通过ORB_ID(mavlink_log)发布。

## 源码

### MavlinkStream类

#### StreamListItem

定义了三个成员：

- new_instance函数指针
- name
- id

### Mavlink类

#### configure_stream

配置单个消息流


#### configure_streams_to_default

配置默认的消息流，根据MAVLINK_TYPE，将需要用到的消息流添加进来。

- MAVLINK_TYPE== MAVLINK_MODE_NORMAL
  包含的消息流有：ADSB_VEHICLE、ALTITUDE、ATTITUDE等。
- MAVLINK_TYPE==MAVLINK_MODE_ONBOARD
  包含的消息流有：TIMESYNC、CAMERA_TRIGGER、HIGHRES_IMU。



## 附录

### MAVLink协议

![image-20230309141927403](imgs\image-20230309141927403.png)



#### 消息长度

以HEARTBEAT为例，其长度包括：

|起始位|帧头长度|payload长度|校验长度|签名长度|=1+9+payload+2+13

例如代码中：

```c
MAVLINK_START_UART_SEND(chan, header_len + 3 + (uint16_t)length + (uint16_t)signature_len);
```



### 串口设备

| UART   | Device     | Port                  |
| ------ | ---------- | --------------------- |
| UART1  | /dev/ttyS0 | GPS                   |
| USART2 | /dev/ttyS1 | TELEM1 (flow control) |
| USART3 | /dev/ttyS2 | TELEM2 (flow control) |
| UART4  | /dev/ttyS3 | TELEM4                |
| USART6 | /dev/ttyS4 | RC SBUS               |
| UART7  | /dev/ttyS5 | Debug Console         |
| UART8  | /dev/ttyS6 | PX4IO                 |

