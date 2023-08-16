# 遥控数据处理流程

​		本章节讲解遥控数据的处理流程。

​		由于遥控器厂商不同、飞手操作习惯不同，为了适配各厂商遥控器的差异，方便飞手进行个性化配置，飞控需要对从遥控器信号输入到产生飞控控制信号制定灵活的处理流程。

## 遥控消息解析流程

​		为了方便对整个处理流程进行表示，我们对各个阶段的数据使用其对应的orb消息结构体类型名进行说明。

在不同阶段，处理的数据主要有：

- 遥控原始数据

  也就是直接根据厂商提供的协议从接收机接收并解析后的数据，这里面的数据根据厂商不同，千差万别。

- input_rc_s

  将原始数据进行重新整理，保存了各通道原始值、通道数、接收计算统计、是否丢失等数据。

- rc_channels_s 

  定义了18个通道，每个通道取值[1000, 2000]，保存了通道功能定义等。

- manual_control_setpoint_s

  包括了x,y,z,r，作为控制器的期望控制输入。

数据处理流程如下图：

sbus frame => input_rc_s=> rc_channels_s => manual_control_setpoint_s manual_control_switches_s



### 解析遥控信号

实现代码：在驱动层，如futaba遥控解析代码文件为drv_sbus.c

功能：根据遥控器厂商定义的协议，将接收机输入的二进制数据进行解析，这部分功能一般由辅控芯片完成，然后将解析后的数据通过串口发送给主控。

> 由于使用的遥控器不同，故通道数不同，通道取值范围也不同。

### 获取input_rc_s

功能：将sbus frame转成input_rc_s，主要获取遥控器18通道的数据，取值范围为[1000, 2000]。

input_rc_s消息还包括了其他关键信息，例如遥控信号接收计数、遥控是否丢失标志等。

由于遥控器厂商不同，遥控通道取值也不同，非常有必要进行转换，保证输入到飞控的数据统一。故这里需要对遥控数据，进行取值范围转换，例如对于sbus数据来说，输入数据范围是200~1800，需要转换至1000~2000，代码示例如下：

```c
#define SBUS_RANGE_MIN 200.0f      // 定义输入范围
#define SBUS_RANGE_MAX 1800.0f

#define SBUS_TARGET_MIN 1000.0f    // 定义输出范围
#define SBUS_TARGET_MAX 2000.0f

/* pre-calculate the floating point stuff as far as possible at compile time */
#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))

/**
 * 对所有通道进行转换
 */
for (uint8_t i = 0; i < 18; i++) {
	// 将sbus原始值转换到1000-2000
    rc_val.values[i] = (raw_data[i] * SBUS_SCALE_FACTOR + 0.5f) + SBUS_SCALE_OFFSET;
}
```

### 获取rc_channel_s

实现代码：rc_update.c

功能：将input_rc_s转成rc_channel_s，根据参数（RC?_MIN, TRIM, MAX, REV），进行通道映射。

也就是输入为18通道1000-2000，输出值为 -1~+1 。

通过校准后，获取18个通道的校准参数，进行归一化操作后，油门通道值[0 1]，其他通道都是[-1 1]

#### 校准

默认参数值为：

RCx_MIN=1000

RCx_TRIM=1500

RCx_MAX=2000

RCx_REV=1

RCx_DZ=10

经过校准后，对于油门通道来说有两种情况：

- 油门处于最低位置（例如futaba遥控器），校准后RC3_TRIM=RC3_MIN

- 油门处于中间位置（例如大疆遥控器），校准后RC3_TRIM>RC3_MIN

#### 归一化基本逻辑

根据RCx_MIN、RCx_TRIM、RCx_MAX三个参数进行归一化处理。

trim是min与max之间的值，即$trim \in [min, max]$，然后将输入值rc_val进行归一化操作：

- 如果输入值在中间值之上，则根据根据[trim, max]区间进行归一化，得到在[0, 1]内的值

- 如果输入值在中间值之下，则根据根据[min, trim]区间进行归一化，得到在[-1, 0]内的值

如果$min<trim<max$，则对输入值进行归一化处理后，其取值范围是[-1, 1]。但有两个特殊情况需要考虑：

- 如果center(trim)==min，归一化至[0, 1]

- 如果center(trim)==max，归一化至[-1, 0]

伪代码如下：

```c
// rc_val：rc输入值
if (rc_val > trim){
    ch_val = (rc_val - trim) / (max - trim)
}
else if(rc_val < trim){
    ch_val = (rc_val - trim) / (trim - min)
}
```

#### 反向处理

根据参数RCx_REV进行反向处理。

伪代码如下：

```c
// 反向
if (rev < 0){
    v= v*-1;
}
```



#### 通道映射配置

由于飞手的使用习惯不同，我们需要决定通道与无人机操控的映射关系。例如希望通道1控制无人机横滚，通道3控制无人机油门等。

飞控逻辑中主要通过RC_MAP_x参数实现这个映射关系，每次参数更改都需要将rc_channel_s内的**function**成员进行更新，例如：

```c
void RCUpdate::update_rc_functions() {
    // 摇杆通道映射部分设置
    _rc.function[rc_channels_s::FUNCTION_THROTTLE] = _param_rc_map_throttle.get() - 1;
    _rc.function[rc_channels_s::FUNCTION_ROLL]     = _param_rc_map_roll.get() - 1;
    _rc.function[rc_channels_s::FUNCTION_PITCH]    = _param_rc_map_pitch.get() - 1;
    _rc.function[rc_channels_s::FUNCTION_YAW]      = _param_rc_map_yaw.get() - 1;
}
```

除了最基本的摇杆通道，有时还希望将一些摇杆通道设置为特定功能，如模式切换控制、返回控制等

```c
void RCUpdate::update_rc_functions() {
    // 开关通道映射部分设置
    _rc.function[rc_channels_s::FUNCTION_MODE]       = _param_rc_map_mode_sw.get() - 1;
    _rc.function[rc_channels_s::FUNCTION_RETURN]     = _param_rc_map_return_sw.get() - 1;
    // 后面的通道省略
}
```

由于RC_MAP_x默认值都是0，故必须通过地面站进行遥控器校准后，RC_MAP才会生效。

> 这也是当遥控器进行校准前，都要由飞手选择遥控器是中国手还是日本手、美国手的原因。
>
> 对于日本手配置参数值：RC_MAP_ ROLL=1，PITCH=channel2, THROTTLE=3, YAW=4

### 获取manual_control

实现代码：rc_update.c

功能：根据RC_MAP表示的通道映射关系，将rc_channel_s内各通道转换为manual_control_setpoint_s和manual_swtiches_s

manual_control_setpoint_s最关键包括 (x,y,z,r)  

飞控中设置的取值来源对应关系为：

- x:  取横滚控制摇杆对应的通道值
- y:  取俯仰控制摇杆对应的通道值
- z:  取油门控制摇杆对应的通道值
- r:  取方向控制摇杆对应的通道值

对应代码为：

```c
void RCUpdate::UpdateManualSetpoint(){
    manual_control_setpoint.y = get_rc_value(rc_channels_s::FUNCTION_ROLL, -1.f, 1.f);
    manual_control_setpoint.x = get_rc_value(rc_channels_s::FUNCTION_PITCH, -1.f, 1.f);
    manual_control_setpoint.r = get_rc_value(rc_channels_s::FUNCTION_YAW, -1.f, 1.f);
    manual_control_setpoint.z = get_rc_value(rc_channels_s::FUNCTION_THROTTLE, -1.f, 1.f);
}
```

> 注意，这里我们称呼的横滚摇杆、俯仰摇杆等，仅仅是对各摇杆进行约定熟成的命名方便理解和记忆，并不是说改摇杆一定控制横滚或者俯仰，与飞控所处的飞行模式有关系！当姿态模式下，横滚摇杆控制的是横滚角，而位置模式下，横滚摇杆控制的是x方向的速度。

#### x,y,z,r的含义

这里为什么用x,y,z,r表示呢？

因为无人机有不同的飞行模式，在不同飞行模式下遥控器控制的物理量不一样，在不同模式下，xyzr的含义也不一样。例如在手动模式下，控制的是姿态角，则x表示期望横滚角，而在位置模式下x表示机体坐标系下的x方向期望速度。

## 遥控输入

### 校准

#### 遥控器校准

设置param

RCxx_MIN/TRIM/MAX/REV

RC_MAP????



#### joystick校准

（1）不会自动做任何param配置

（2）joystick产生manual_control_setpoint_s，通过mavlink数据链发给飞控，mavlink_manual_control.c，需要配置RC_MAP_PITHC=1,ROLL=2,YAW=3,THROTTLE=4, 

（3）joystick不产生manual_swtiches_s，按钮直接通过command_long



param set RC_MAP_PITCH 1

param show RC_MAP*



### 虚拟摇杆

QGC能够接入虚拟摇杆（例如游戏手柄），并且生成`MAVLINK_MSG_ID_MANUAL_CONTROL`消息，飞控mavlink接收到该消息后，可以发布`ORB_ID(manual_control_setpoint)`主题。

如果希望通过manual_control_setpoint生成input_rc，可以设置参数`COM_RC_IN_MODE=2`，这样接入虚拟摇杆后，飞控可以发布`ORB_ID(input_rc)`。由于QGC只发送前四个通道（俯仰、横滚、油门、偏航），故转成了input_rc后也只有前面四个通道有效。



mavlink_main.cpp的主函数会订阅vehicle_status，并对其rc_input_mode进行判断，如果为RC_IN_MODE_GENERATED则会通过manual_control_setpoint生成input_rc

```c
if (_vehicle_status_sub.updated()) {
    
set_generate_virtual_rc_input(vehicle_status.rc_input_mode == vehicle_status_s::RC_IN_MODE_GENERATED);
}
```



## 相关参数

| 参数             | 说明                                   | 取值                                                         |
| ---------------- | -------------------------------------- | ------------------------------------------------------------ |
| COM_FLTMODE1~6   | 设置飞行模式通道开关位置对应的飞行模式 | 1~16，该值减1就对应commander_state_s::MAIN_STATE_xxx<br>参考：<br>Commander.cpp/_flight_mode_slots<br>commander_state.h<br> |
| RC_MAP_FLTMODE   | 设置飞行模式对应的通道                 | 根据遥控器通道数决定，一般为1~18                             |
| RC_MAP_OFFB_SW   | 设置offboard模式对应的通道             | 根据遥控器通道数决定，一般为1~18                             |
| RC_MAP_RETURN_SW | 设置返航对应的通道                     | 根据遥控器通道数决定，一般为1~18                             |



## 附录

### sbus2协议

#### 协议格式

```c

|startbyte | data1 | data2 | ... | data22 | flags | endbyte|
|  1Byte   |             22Bytes          | 1Byte | 1Byte  |
```

- startbyte:

  帧头起始字节，由1个字节表示，默认为0x0F;

- data:

  数据内容，包括22个字节，data1…data22对应16个通道（ch1-ch16），每个通道11bit（228=1611=176）;

- flags:

  一个字节，每个比特位表示如下

  - bit0: 表示第17通道的值；
  - bit1: 表示第18通道的值；
  - bit2: 表示frame lost；
  - bit3: 表示failsafe；

- endbyte:

  整包的结尾字节，表示下一包数据的内容是什么。取值0x00，0x04，0x14，0x24，0x34，各代表的下一包内容含义如下：
  
  - 0x00: S.Bus 1
  - 0x04: 接收机电压
  - 0x14: GPS或气压数据
  - 0x24: 未知的SBus2数据
  - 0x34: 未知的SBus2数据







#### 典型SBus数据包示例

Futaba RS3008S接收机

```shell
[ 2023-04-17 14:25:10.983 ] :0ff6b31f5c020809b040021200042000010840000210800014
[ 2023-04-17 14:25:10.983 ] :0ff6b31f5c020809b040021200042000010840000210800024
[ 2023-04-17 14:25:10.983 ] :0ff6b31f5c020809b040021200042000010840000210800034
[ 2023-04-17 14:25:10.983 ] :0ff6b31f5c020809b040021200042000010840000210800004
[ 2023-04-17 14:25:10.983 ] :03c400
[ 2023-04-17 14:25:10.983 ] :0ff6b31f5c020809b040021200042000010840000210800014
[ 2023-04-17 14:25:10.983 ] :0ff6b31f5c020809b040021200042000010840000210800024
[ 2023-04-17 14:25:10.983 ] :0ff6b31f5c020809b040021200042000010840000210800034
[ 2023-04-17 14:25:10.983 ] :0ff6b31f5c020809b040021200042000010840000210800004
[ 2023-04-17 14:25:10.983 ] :03c033
[ 2023-04-17 14:25:10.983 ] :0ff6b31f5c020809b040021200042000010840000210800014
[ 2023-04-17 14:25:10.983 ] :0ff6b31f5c020809b040021200042000010840000210800024
[ 2023-04-17 14:25:10.983 ] :0ff6b31f5c020809b040021200042000010840000210800034
[ 2023-04-17 14:25:10.983 ] :0ff6b31f5c020809b040021200042000010840000210800004
[ 2023-04-17 14:25:10.983 ] :03c400
[ 2023-04-17 14:25:11.184 ] :0ff6b31f5c020809b040021200042000010840000210800014
[ 2023-04-17 14:25:11.184 ] :0ff6b31f5c020809b040021200042000010840000210800024
[ 2023-04-17 14:25:11.184 ] :0ff6b31f5c020809b040021200042000010840000210800034
[ 2023-04-17 14:25:11.184 ] :0ff6b31f5c020809b040021200042000010840000210800004
[ 2023-04-17 14:25:11.184 ] :03c033
```









