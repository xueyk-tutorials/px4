# 日志

## ULog格式

官方文档：https://docs.px4.io/main/en/dev_log/ulog_file_format.html

### 文件结构

日志文件包括三个部分：

```shell
----------------------
|       Header       |
----------------------
|    Definitions     |
----------------------
|        Data        |
----------------------
```

### 文件内容

#### Header-文件头段

Header为固定长度（16个字节）

```shell
----------------------------------------------------------------------
| 0x55 0x4c 0x6f 0x67 0x01 0x12 0x35 | 0x01         | uint64_t       |
| File magic (7B)                    | Version (1B) | Timestamp (8B) |
----------------------------------------------------------------------
```

#### 定义段

不定长，包括了**版本信息**，**格式定义**，**初始参数值**。

该段包含了一系列消息，每个消息都包括了消息头，定义如下：

```c
struct message_header_s {
	uint16_t msg_size;
	uint8_t msg_type
};
```

- msg_size:

消息长度为消息字节长度，不包括上面3字节的消息头。

- msg_type:

消息类型定义了消息内容，类型通过单字符表示，主要有：'B', 'F', 'I', 'M', 'P', 'Q'

定义段在第一个 `message_add_logged_s` 或 `message_logging_s`消息前结束。

#### 数据段

### 典型示例

日志文件会保存无人机的基本信息，例如软硬件版本等，这些对应**'I'信息**。

![image-20230327194024494](imgs\image-20230327194024494.png)



### 定义段内的消息类型

#### 'I': information message信息

一般用于记录总体描述信息。

```c++
struct message_info_s {
	struct message_header_s header;
	uint8_t key_len;
	char key[key_len];
	char value[header.msg_size-1-key_len]
};
```

key对应的信息一般只会出现一次。

示例：

| key                            | Description                  | Example for value |
| ------------------------------ | ---------------------------- | ----------------- |
| char[value_len] sys_name       | Name of the system           | "PX4"             |
| char[value_len] ver_hw         | Hardware version (board)     | "PX4FMU_V4"       |
| char[value_len] ver_hw_subtype | Board subversion (variation) | "V2"              |
| char[value_len] ver_sw         | Software version (git tag)   | "7f65e01"         |

#### 'P': parameter参数

用于记录飞控参数。对于改动的参数会保存在数据段。格式与'I'一样。

```c++
struct message_info_s {
	struct message_header_s header;
	uint8_t key_len;
	char key[key_len];
	char value[header.msg_size-1-key_len]
};
```



#### 'F': Format数据格式

记录所有订阅消息的结构体数据格式。

```c
struct message_format_s {
	struct message_header_s header;
	char format[header.msg_size];
};
```

format是不定长的纯文本数组，格式为：`message_name:field0;field1;`。

其中message_name是消息名称，一个消息包含一个或多个字段（feild），字段格式为`type field_name`，或者`type[array_length] field_name`，type表示消息成员变量的类型，field表示变量名。

例如，对于消息vehicle_attitude来说，其结构体为：

```c++
#ifdef __cplusplus
struct __EXPORT vehicle_attitude_s {
#else
struct vehicle_attitude_s {
#endif
    uint64_t timestamp;
    uint64_t timestamp_sample;
    float    q[4];
    float    delta_q_reset[4];
    uint8_t  quat_reset_counter;
    uint8_t  _padding0[7]; // required for logger
#ifdef __cplusplus
#endif
};
```

对应的format为：

```c
'vehicle_attitude:uint64_t timestamp;uint64_t timestamp_sample;float[4] q;float[4] delta_q_reset;uint8_t  quat_reset_counter;uint8_t[7] _padding0;'
```



### 数据段内的消息类型

#### 'A': AddLogged添加消息

表示添加一个订阅消息的信息，为每个订阅消息的名称给定一个ID。这样后面的**'D'**保存每一次订阅消息时就只使用ID就可以知道其名称了。

```c++
struct message_add_logged_s {
	struct message_header_s header;
	uint8_t multi_id;
	uint16_t msg_id;
	char message_name[header.msg_size-3];
};
```

#### 'D': Data消息数据

记录具体的数据内容，各种不同格式的数据持续产生，以如下格式进行记录：

```c++
struct message_data_s {
	struct message_header_s header;
	uint16_t msg_id;
	uint8_t data[header.msg_size-2];
};
```

数据的格式获取：通过msg_id进行匹配，通过`A，message_add_logged_s`查找message_name，进而在定义段`F，message_format_s`中根据message_name获取数据格式。

####  'L': Logged String Message

记录各模块发布的字符串信息，并且给定消息的等级（错误、警告、调试等）。

其他模块进行字符打印与记录时调用PX4_INFO、PX4_WARN、PX4_PANIC、PX4_ERR函数 --> px4_log --> ORB_ID(log_message) --> logger模块

> 同时mavlink模块也会订阅ORB_ID(log_message)消息

## 日志分析工具

### pyulog

- ulog_info

  命令格式：ulog_info [-h] [-v] file.ulg

- ulog_messages

  命令格式：ulog_info [-h] file.ulg

- ulog_params

  命令格式：`ulog_params [-h] [-d DELIMITER] [-i] [-o] file.ulg [params.txt]`

- ulog2csv

  命令格式：`ulog2csv [-h] [-m MESSAGES] [-d DELIMITER] [-o DIR] file.ulg`

## logger模块

### 帮助命令

- logger status

```shell
msh />logger status
        Running in mode: file
        Number of subscriptions: 166 (6640 bytes)
          Full File Logging Running:
                Log file: /log/2018-01-01/00_00_01.ulg
                Wrote 1.57 MiB (avg 110.92 KiB/s)
                Since last status: dropouts: 0 (max len: 0.000 s), max used buffer: 11027 / 12288 B
```



## 软件设计

### 功能需求分析

- 保存各模块的orb消息

- 保存mission

- 将各模块的状态信息发送至mavlink模块进而发送至地面站

### 数据缓存设计

为了节省时间，不应该频繁进行SD写操作，故需要将写的数据一点点的累积并存放在缓存数组中，等待时机，一次性将需要写的数据写入文件中。

对缓存的操作涉及到两个线程，一个放数据的线程，一个取数据的线程。

- main thread：产生数据，并将数据放到缓存数组中，一次只放一点；
- write thread：取数据并写入文件，根据情况取出数据，一次取的数据会很大；

![image-20230330171252011](imgs\image-20230330171252011.png)

> 这个缓存数组就像一个蓄水池，平常一点点的积累水，用的时候开闸放水，一次放出满足要求的水，一般会放出很多水。

**定义几个名词：**

buff_size：缓存数组长度（字节大小），也就是能够放多少个字节；

count：要写入文件的字节大小，也就是需要取出多少个字节并写入文件中；

head：下一次放数据的位置，存数据线程main thread会把数据从该位置存放，取数据线程write thread会从该位置取出数据，取出的字节大小为count；

#### 如何存

由于head的位置随着存数据和取数据会不断变化，那么会有两个情况：1）head位置距离缓存数组末尾还很远，足够一次性放下当次需要存放的数据；2）head位置接近缓存数据末尾了，无法一次性放下。

**第一种情况：整块放**

对于第一种情况很好解决，直接将数据拷贝至head位置。

![image-20230330172418255](imgs\image-20230330172418255.png)

拷贝完成并且更新head位置。

![image-20230330180327233](imgs\image-20230330180327233.png)



**第二种情况：分两块放**

通过判断，无法一次性放下

![image-20230330180430257](imgs\image-20230330180430257.png)

对要存放的数据分成两个部分

![image-20230330180520571](imgs\image-20230330180520571.png)

先将末尾填满

![image-20230330180549694](imgs\image-20230330180549694.png)

然后将剩余的放在开始位置

![image-20230330180657671](imgs\image-20230330180657671.png)

**这种情况下，数据分为两块放在首尾！**

#### 如何取

对应两种存放情况，取也有两种情况：

**第一种情况：整块取**

直接根据head所在位置前面count大小的数据取出来即可。

**第二种情况：分两块取**

先取首

## 源码

### logger.cpp

#### init()



#### run()

run函数包括两个部分：初始配置部分和循环迭代部分。

1. 初始配置

2. 循环迭代
   - 循环一次用时约4ms；
   - 循环迭代的推出条件是全局函数should_exit()决定；
   - 迭代更新由两种驱动方式：1）定时器驱动，2）依赖某个消息更新驱动，如果需要使用这种方式，则在类实例化的时候指定消息名称并拿到消息对应的meta_data存放至_polling_topic_meta变量，后续创建针对该消息的订阅polling_topic_sub，只要订阅到该消息，则进入下一轮循环。



##### timer

hrt_call_every()



#### writer

创建线程



### log_writer.cpp

##### 确定使用的后端

类初始化时，根据传入参数（backend）是使用文件还是mavlink作为后端



### log_writer_file.cpp

##### 类LogWriterFile

创建写线程，定周期将放在缓存中的数据写入SD卡，当前周期为1s。





### log_writer_mavlink.cpp

##### 类LogWriterMavlink

将日志通过MAVLink发送至地面站。logger模块与mavlink模块通过orb消息传输日志，然后再发送至地面站。

logger模块 -- ORB_ID(ulog_stream) -->mavlink模块

logger模块 <-- ORB_ID(ulog_stream_ack) -->mavlink模块



mavlink_log_info()函数的作用是通过uorb发布消息，并且在终端打印。其使用示例如下：

```c++

mavlink_log_info(&_mavlink_log_pub, "Takeoff detected");
mavlink_log_critical(&_mavlink_log_pub, "Mission start denied! No valid mission");
```



## 相关MSG和CMD

LOGGING_DATA ( #266 )

MAV_CMD_LOGGING_START (2510)







## 参数

| 参数          | 默认值 | 解释                                                         |
| ------------- | ------ | ------------------------------------------------------------ |
| SDLOG_PROFILE | 0      | 用于设置记录哪些消息以及记录消息的频率<br>具体各种配置对应记录的消息有哪些可以查看函数<br>`void LoggedTopics::initialize_configured_topics(SDLogProfileMask profile)` |
|               |        |                                                              |
|               |        |                                                              |



















