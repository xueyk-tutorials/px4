# 混控器mixer

## 概述

将混控逻辑与实际的姿态控制器分离开来大大提高了程序的可复用性。

## 参考

https://docs.px4.io/v1.12/zh/concept/mixing.html

## 基本概念

### 控制组

### 定义

混控器定义文件路径为：ROMFS\px4fmu_common\mixers，另外仿真对应的混控文件路径为：ROMFS\px4fmu_common\mixers-sitl。

mixer由纯文本文件描述，文件后缀名为.mix。对于xxx.main.mix来说，其对应的输出是MAIN通道8路PWM，而xxx.aux.mix来说，其对应的输出是AUX通道8路PWM。

#### 语法

一共有4中类型的混控器，分别为：

混控器类型定义格式为：`<tag>: <arguments>`。其中tag表示如下：

- `R`: [Multirotor mixer](https://docs.px4.io/v1.12/en/concept/mixing.html#multirotor_mixer)
- `H`: [Helicopter mixer](https://docs.px4.io/v1.12/en/concept/mixing.html#helicopter_mixer)
- `M`: [Summing mixer](https://docs.px4.io/v1.12/en/concept/mixing.html#summing_mixer)
- `Z`: [Null mixer](https://docs.px4.io/v1.12/en/concept/mixing.html#null_mixer)

##### 加法混控器-R

- M: 

  地点

  ```c
  M: <control count>
  O: <-ve scale> <+ve scale> <offset> <lower limit> <upper limit> <traversal time>
  ```

  加法混控器是0个或多个控制输入混合为单个执行器输出。如果控制输入是0个，则输出将是一个定值，也就是处于限幅下的offset。如果不会0个，则将多个输入进行混合累加。

- S: 

  格式为：

  ```c
  S: <group> <index> <-ve scale> <+ve scale> <offset> <lower limit> <upper limit>
  ```

  - group：表示输入控制组
  - index：表示输入控制组内的通道序号
  - -ve scale：如果输入为负，则对输入根据该值进行比例缩放
  - ve scale：如果输入为正，则对输入根据该值进行比例缩放
  - offset：通过缩放计算后，叠加该偏移量
  - lower limit：限幅，最小值
  - upper limit：限幅，最大值

##### 空-Z 

空占位

### 示例

#### fw_generic_wing.main.mix

```shell
Generic wing mixer
===========================

This file defines mixers suitable for controlling a delta wing aircraft.
The configuration assumes the elevon servos are connected to servo
outputs 0 and 1 and the motor speed control to output 3. Output 2 is
assumed to be unused.

Inputs to the mixer come from channel group 0 (vehicle attitude), channels 0
(roll), 1 (pitch) and 3 (thrust).

See the README for more information on the scaler format.
### 输出通道0和1：用于控制左副翼、右副翼
### 
Elevon mixers
-------------
Three scalers total (output, roll, pitch).
# 默认两个舵机安装方向相反，也就是舵机输入的PWM一样时，一个舵机向上拉而另外一个舵机向下拉
On the assumption that the two elevon servos are physically reversed, the pitch
input is inverted between the two servos.

The scaling factor for roll inputs is adjusted to implement differential travel
for the elevons.
## 输出通道0的混控，由group_0的输入0(roll)和输入1(pitch)进行相加
## 当roll=0.1 pitch=0.3时，输出为：
## pwm = (-8000*0.1+0) + (6000*0.3+0) = 1000
M: 2
S: 0 0  -8000  -8000      0 -10000  10000
S: 0 1   6000   6000      0 -10000  10000
## 输出通道1的混控，由group_0的输入0(roll)和输入1(pitch)进行相加
## 当roll=0.1 pitch=0.3时，输出为：
## pwm = (-8000*0.1+0) + (-6000*0.3+0) = -2600
M: 2
S: 0 0  -8000  -8000      0 -10000  10000
S: 0 1  -6000  -6000      0 -10000  10000

### 输出通道2：空
Output 2
--------
This mixer is empty.

Z:

### 输出通道3：用于控制推力电机，由group_0的输入3(thrust)
Motor speed mixer
-----------------
Two scalers total (output, thrust).

This mixer generates a full-range output (-1 to 1) from an input in the (0 - 1)
range.  Inputs below zero are treated as zero.

M: 1
S: 0 3      0  20000 -10000 -10000  10000

```



#### pass.aux.mix

```shell
# Manual pass through mixer for servo outputs 1-4

# AUX1 channel (select RC channel with RC_MAP_AUX1 param)
M: 1
S: 3 5  10000  10000      0 -10000  10000

# AUX2 channel (select RC channel with RC_MAP_AUX2 param)
M: 1
S: 3 6  10000  10000      0 -10000  10000

# AUX3 channel (select RC channel with RC_MAP_AUX3 param)
M: 1
S: 3 7  10000  10000      0 -10000  10000

# FLAPS channel (select RC channel with RC_MAP_FLAPS param)
M: 1
S: 3 4  10000  10000      0 -10000  10000
```





## 加载混控器

机架

ROMFS\px4fmu_common\init.d\airframes

例如对于四旋翼机架来说，其启动文件为4001_quad_x：

```shell
#!/bin/sh
#
# @name Generic Quadcopter
#
# @type Quadrotor x
# @class Copter
#
# @output MAIN1 motor 1
# @output MAIN2 motor 2
# @output MAIN3 motor 3
# @output MAIN4 motor 4
# @output MAIN5 feed-through of RC AUX1 channel
# @output MAIN6 feed-through of RC AUX2 channel
#
# @output AUX1 feed-through of RC AUX1 channel
# @output AUX2 feed-through of RC AUX2 channel
# @output AUX3 feed-through of RC AUX3 channel
# @output AUX4 feed-through of RC FLAPS channel
#
# @maintainer Lorenz Meier <lorenz@px4.io>
#

. ${R}etc/init.d/rc.mc_defaults
### 设置mixer，对应的是quad_x.main.mix
set MIXER quad_x

set PWM_OUT 1234
```







### 常用混控器

#### 车辆

- rover_generic.main.mix

  对应的机架有50000_generic_ground_vehicle

- rover_diff_and_servo.main.mix

  对应的机架有50004_nxpcup_car_dfrobot_gpx

