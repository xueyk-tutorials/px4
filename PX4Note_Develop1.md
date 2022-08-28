PX4开发笔记——二次开发（）

# PX4开发环境搭建

**环境系统**：ubuntu18.04

**px4版本**：1.11

**参考官网**：https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html



### 烧写固件 (Flashing the board)

在相应的编译命令后添加**upload**即可。

```shell
$ make px4_fmu-v5_default upload
```

如下是烧写过程中的输出：

```bash
alex@alex_laptop:~/Desktop/PX4-Autopilot$ make px4_fmu-v5_default upload
[0/13] Performing build step for 'px4io_firmware'
ninja: no work to do.
[1/2] uploading px4
==========================================================================================================
WARNING: You should uninstall ModemManager as it conflicts with any non-modem serial device (like Pixhawk)
==========================================================================================================
Loaded firmware for board id: 50,0 size: 1889441 bytes (91.53%), waiting for the bootloader...


Found board id: 50,0 bootloader version: 5 on /dev/serial/by-id/usb-3D_Robotics_PX4_BL_FMU_v5.x_0-if00
sn: 002d001a3138510238393738
chip: 10016451
family: b'STM32F7[6|7]x'
revision: b'Z'
flash: 2064384 bytes
Windowed mode: False

Erase  : [====================] 100.0%
Program: [====================] 100.0%
Verify : [====================] 100.0%
Rebooting. Elapsed Time 15.884

```

### SD目录

这里飞控型号是雷迅的V5+，使用的PX4固件版本是1.11.3，SD卡目录如下：

```javascript
/
	|--dev/
    	|--baro0
		|--console
		|--led0
		|--mag0
		|--mag1
		|--px4fmu
		|--px4io
		|--tone_alarm0
		|--ttyS0
		|...
    |--etc/
        |--extras/
    		|--px4_io-v2_default.bin
        |--mixers/
        |--init.d/    //存放启动文件，这个就是烧写固件时拷贝后的文件夹，从PX4-Autopilot/build/px4_fmu-xx_default/genromfs/init.d/拷贝
    |--fs/
    	|--microsd/
    	|--mtd_params
		|--mtd_waypoints
    |--obj/
    |--proc/
```



## 调试

### 打开Shell

为了验证程序运行结果，需要启动PX4的交互终端，进行程序启动、输出打印等等，这时就需要启动PX4的Shell，有两种方式，一种是通过QGC地面站，一种是通过运行`mavlink_shell.py`打开调试串口。

#### QGroundControl MAVLink Console

open QGC, **Analyze View > Mavlink Console**

#### mavlink_shell.py

You can also access the shell in a terminal using the **mavlink_shell.py** script:

1. Shut down *QGroundControl*.

2. Install dependencies:

   ```sh
   sudo pip3 install pymavlink pyserial
   ```

3. Open terminal (in PX4-Autopilot directory) and start the shell:

   ```sh
   # For serial port
   ./Tools/mavlink_shell.py /dev/ttyACM0
   ```

   ```sh
   # For Wifi connection
   ./Tools/mavlink_shell.py 0.0.0.0:14550
   ```

Use `mavlink_shell.py -h` to get a description of all available arguments.



# PX4系统架构

## 启动脚本-ROMFS

#### 脚本解析

在PX4中，使用rc文件控制程序的启动，类似与Linux中的shell脚本，其中主脚本为rcS。实际上同一个脚本你会在不同路径下都看到，那么他们有什么关系呢？

| 脚本路径                                  | 说明                                                         |      |
| ----------------------------------------- | ------------------------------------------------------------ | ---- |
| ROMFS/px4fmu_common/init.d                | 用于最原始的编辑，里面有很多注释用于解释每行脚本的意义，编辑后使用`make`命令进行编译 |      |
| build/px4_fmu-xx_default/genromfs/init.d/ | 编译后生成的脚本，无注释，另外会多出一些脚本如：`rc.autostart` |      |
| /etc/init.d/                              | SD卡中的目录，固件烧写时从`build/px4_fmu-xx_default/genromfs/init.d/`拷贝过来，实际飞控运行时时会从该目录下启动 |      |

对于主脚本`rcS`进行解析如下：

```bash
### PX4 V1.11.3
### --------------------------------------
### 通过set设置脚本中默认变量值
set FRC /fs/microsd/etc/rc.txt                 //在SD卡相应目录下查找rc.txt文件
### ...

### --------------------------------------
### 挂载SD卡至目录/fs/microsd，在shell中用`ls /fs`命令可看到目录/fs下的所有文件夹如下：
### nsh> ls fs/
### /fs/bbr0, /fs/bbr1, fs/bbr2, /fs/bbr3, /fs/microsd/, /fs/mtd_params, /fs/mtd_waypoints
if mount -t vfat /dev/mmcsd0 /fs/microsd

### --------------------------------------
### 由于`/fs/microsd/etc/rc.txt`文件不存在，故通过autostart进行启动，否则将根据rc.txt进行启动。
#
# Look for an init script on the microSD card.
# Disable autostart if the script found.
#
if [ -f $FRC ]
then
	. $FRC
else
	### --------------------------------------
	### 挂载指定参数文件为：/fs/mtd_params
	if mtd start
	then
		set PARAM_FILE /fs/mtd_params
	fi
	### ...
	
	### --------------------------------------
	### 加载参数文件并将所有参数保持至变量`param`中，这些参数都可以通过QGC查看并设置。
	param select $PARAM_FILE
	### ...
	
	### --------------------------------------
	### Set AUTOCNF flag to use it in AUTOSTART scripts.
	### `SYS_AUTOSTART=0(keep params)`（可通过QGC查看）,不执行AUTOCNF
	if param greater SYS_AUTOCONFIG 0
	then
		###...
		set AUTOCNF yes
	fi
	
	### --------------------------------------
	### 由于rc.board_defaults文件存在则执行
	### 【rc.board_defaults文件内容如下】
	### set VEHICLE_TYPE rover
	### if [ $AUTOCNF = yes ]
	### then
	### ...
	### fi
	### set PWM_OUT 1234
	### set PWM_RATE 50
	### set MIXER_AUX pass      //gimbal pass mixer
	### set PWM_AUX_OUT 1234
	set BOARD_RC_DEFAULTS /etc/init.d/rc.board_defaults
	if [ -f $BOARD_RC_DEFAULTS ]
	then
		echo "Board defaults: ${BOARD_RC_DEFAULTS}"
		sh $BOARD_RC_DEFAULTS
	fi
	unset BOARD_RC_DEFAULTS
	
	### --------------------------------------
	###
	dataman start $DATAMAN_OPT    //Waypoint storage.(set DATAMAN_OPT "")
	send_event start              //Start the socket communication send_event handler.
	load_mon start                //Start the resource load monitor.
	rgbled start -X               //Start system state indicator.
	rgbled_ncp5623c start -X
	
	### --------------------------------------
	### 加载完参数后，启动音乐
	tone_alarm start
	### ...
	
	### --------------------------------------
	### 设置AIRFRAME
	### 设置MIXER、PWM_OUT
	### 设置VEHICLE_TYPE
	### 设置MIXER_AUX、PWM_AUX_OUT
	### 由于`SYS_AUTOSTART=4001`（可通过QGC查看），故调用rc.autostart，这个文件在
	### 编译后的文件夹中，PX4-Autopilot/build/px4_fmu-xx_default/genromfs/init.d/rc.autostart
	### 1）【rc.autostart文件内容如下】
	### // 根据SYS_AUTOSTART设置机体类型！
	### if param compare SYS_AUTOSTART 4001
    ### then
    ### set AIRFRAME 4001_quad_x
	### fi
	### // 再根据机体类型调用相应机体的启动文件
	### if [ ${AIRFRAME} != none ]
	### then
	### sh /etc/init.d/airframes/${AIRFRAME}    //调用SD卡文件/etc/init.d/airframes/4001_quad_x
	### 2）【4001_quad_x文件内容如下】
	### sh /etc/init.d/rc.mc_defaults           //调用SD卡文件/etc/init.d/rc.mc_defaults
	### set MIXER quad_x
	### set PWM_OUT 1234
	
	### 3）【rc.mc_defaults文件内容如下】
	### set VEHICLE_TYPE mc
	### if [ $AUTOCNF = yes ]                    //默认为 no
	### then
    ### 	param set-default NAV_ACC_RAD 2
    ### 	param set-default RTL_RETURN_ALT 30
    ### 	param set-default RTL_DESCEND_ALT 10
    ### 	param set RTL_LAND_DELAY 0
    ### 	param set-default PWM_MAIN_MAX 1950
    ### 	param set-default PWM_MAIN_MIN 1075
    ### 	param set-default PWM_MAIN_RATE 400
    ### 	param set-default GPS_UBX_DYNMODEL 6
    ### fi
    ### set MIXER_AUX pass                       // gimble pass mixer
    ### set PWM_AUX_OUT 1234
    
	if ! param compare SYS_AUTOSTART 0		      //SYS_AUTOSTART的值根据选定的机型来定，例如四旋翼x机型为4001
	then
		sh etc/init.d/rc.autostart
	fi
	### ...
	
	### --------------------------------------
	### 由于rcS开头定义set IOFW "/etc/extras/px4_io-v2_default.bin"
	### 且bin文件存在，故执行
	if [ -f $IOFW ]
	then
		...
	fi
	
	### --------------------------------------
	### RC update (map raw RC input to calibrate manual control)
	### 启动遥控器进程
	rc_update start
	### ...
	
	### --------------------------------------
	### 启动sensor，SYS_HITL默认为0
	if param greater SYS_HITL 0
	then
		### 硬件在环启动
	else
		set BOARD_RC_SENSORS /etc/init.d/rc.board_sensors
        if [ -f $BOARD_RC_SENSORS ]
            then
                echo "Board sensors: ${BOARD_RC_SENSORS}"
                sh $BOARD_RC_SENSORS
            fi
            unset BOARD_RC_SENSORS
            ### Sensors System (start before Commander so Preflight checks are properly run)  
            ### 启动各类传感器并进行检查
            sh /etc/init.d/rc.sensors
            
            ### 设置电池信息来源
            ### 默认BAT1_SOURCE=0(power module)
            ### power_module=0: 使用电源模块获取电池信息
            ### External=1: 根据mavlink消息获取电池信息
            ### ESCS=2: 使用电调获取，这时候电调必须具备电压和电流测量功能
            if param compare -s BAT1_SOURCE 2
            then
                esc_battery start
            fi

            if ! param compare BAT1_SOURCE 1
            then
                battery_status start
            fi
            
            ### 启动commander，类似于任务管理器
            commander start
    fi
    
    ### ...
    
	### --------------------------------------
	### 启动mavlink
	### 【rc.board_mavlink文件内容如下】
	### mavlink start -d /dev/ttyACM0
	set BOARD_RC_MAVLINK /etc/init.d/rc.board_mavlink
	if [ -f $BOARD_RC_MAVLINK ]
	then
		echo "Board extras: ${BOARD_RC_MAVLINK}"
		sh $BOARD_RC_MAVLINK
	fi
	unset BOARD_RC_MAVLINK
	
	### --------------------------------------
	### Start UART/Serial device drivers.
	### Note: rc.serial is auto-generated from Tools/serial/generate_config.py
	sh /etc/init.d/rc.serial
	
	### --------------------------------------
	### 【rc.vehicle_setup文件内容如下】
	### 在文件rc.mc_defaults中，会定义VEHICLE_TYPE
	### if [ $VEHICLE_TYPE = mc ]
	### then
	### 	...
    ### 	if [ $MAV_TYPE = none ]
    ### 	then
	###			set MAV_TYPE 2
	###     //根据MIXER设置MAV_TYPE
	###		param set MAV_TYPE ${MAV_TYPE}
	### 	//根据MIXER配置输出
	### 	sh /etc/init.d/rc.interface
	###
	### 	//启动多旋翼相关进程
	###  	//mc_rate_control start
	### 	//mc_hover_thrust_estimator start
	###		//mc_pos_control start
	### 	sh /etc/init.d/rc.mc_apps
	### fi
	#
	# Configure vehicle type specific parameters.
	# Note: rc.vehicle_setup is the entry point for rc.interface,
	#       rc.fw_apps, rc.mc_apps, rc.rover_apps, and rc.vtol_apps.
	#
	sh /etc/init.d/rc.vehicle_setup
	
	### --------------------------------------
	navigator start                //Start the navigator.
	sh /etc/init.d/rc.thermal_cal  //Start a thermal calibration if required.
	### ...
	
	sh /etc/init.d/rc.logging      //Start the logger.
#
# End of autostart.
#
fi

### --------------------------------------
### 清除变量，节约RAM
unset R
unset AUTOCNF

unset FRC
```

#### 脚本语法说明

1、日志记录

在编译过程中，使用`echo`命令可以将信息打印在终端，并且保持在SD卡中。

```shell
echo "ALEX: SYS_AUTOSTART=${SYS_AUTOSTART}" >> $LOG_FILE
```

## mavlink

mavlink作为一个模块（MODULES）被编译。

### 启动

```shell
### 【rcS文件内容】
	#
	# Optional board mavlink streams: rc.board_mavlink
	#
	set BOARD_RC_MAVLINK /etc/init.d/rc.board_mavlink
	if [ -f $BOARD_RC_MAVLINK ]
	then
		echo "ALEX: sh rc.board_mavlink" >> $LOG_FILE
		echo "Board extras: ${BOARD_RC_MAVLINK}"
		sh $BOARD_RC_MAVLINK
	fi
	unset BOARD_RC_MAVLINK
mavlink boot_complete

### 【rc.board_mavlink文件相关内容】
mavlink start -d /dev/ttyACM0
```

### APP

mavlink模块代码路径为：`src/modules/mavlink`。

文件`mavlink_main.cpp`





## 混控器-MIXER

参考：https://docs.px4.io/master/en/concept/mixing.html

混控器定义文件所在目录为：`ROMFS/px4fmu_common/mixers`。A mixer file must be named **XXXX.main.mix** if it is responsible for the mixing of MAIN outputs or **XXXX.aux.mix** if it mixes AUX outputs.

通常MAIN and AUX outputs correspond to MAIN and AUX PWM outputs。

### 控制组

首先需要明白混控器有输入控制组的概念，这个分组是固定的，例如**#0**用于进行飞行控制，**#2**用于进行云台控制，**#3**用于进行遥控器透传，且每个组有多个序号通道。具体如下所示

```python
Control Group #0 (Flight Control)
•	0: roll (-1..1)
•	1: pitch (-1..1)
•	2: yaw (-1..1)
•	3: throttle (0..1 normal range, -1..1 for variable pitch / thrust reversers)
•	4: flaps (-1..1)
•	5: spoilers (-1..1)
•	6: airbrakes (-1..1)
•	7: landing gear (-1..1)
Control Group #1 (Flight Control VTOL/Alternate)
•	0: roll ALT (-1..1)
•	1: pitch ALT (-1..1)
•	2: yaw ALT (-1..1)
•	3: throttle ALT (0..1 normal range, -1..1 for variable pitch / thrust reversers)
•	4: reserved / aux0
•	5: reserved / aux1
•	6: reserved / aux2
•	7: reserved / aux3
Control Group #2 (Gimbal)
•	0: gimbal roll
•	1: gimbal pitch
•	2: gimbal yaw
•	3: gimbal shutter
•	4: reserved
•	5: reserved
•	6: reserved
•	7: reserved (parachute, -1..1)
Control Group #3 (Manual Passthrough)
•	0: RC roll
•	1: RC pitch
•	2: RC yaw
•	3: RC throttle
	4: RC mode switch (Passthrough of RC channel mapped by RC_MAP_FLAPS)
	5: RC aux1 (Passthrough of RC channel mapped by RC_MAP_AUX1)
	6: RC aux2 (Passthrough of RC channel mapped by RC_MAP_AUX2)
	7: RC aux3 (Passthrough of RC channel mapped by RC_MAP_AUX3)
Control Group #6 (First Payload)
•	0: function 0 (default: parachute)
•	1: function 1
•	2: function 2
•	3: function 3
•	4: function 4
•	5: function 5
•	6: function 6
•	7: function 7
```

### 语法格式

```python
R: Multirotor mixer  // R对应多旋翼混控器
H: Helicopter mixer
M: Summing mixer     // M对应简单求和混控器
Z: Null mixer
```

#### 关键字M

M: <control count>

<control count>：表示输出控制的结果数量。

#### 关键字S

S: <group>  <index>  <-ve scale>  <+ve scale>  <offset>  <lower limit>  <upper limit>

<group>：表示控制组编号；

<index>：表示输入组内的序号通道；

 <-ve scale>和<+ve scale>：表示负值和正值的缩放比例；

<offset>：表示偏移；

<lower limit>和<upper limit>：表示输出所限定的最大值和最小值范围。

### 四旋翼混控器

对于四旋翼机体为例，在文件`rc.autostart`中调用. /etc/init.d/airframes/${AIRFRAME}，其中AIRFRAME=4001_quad_x，文件`4001_quad_x`内容如下：

```c
. ${R}etc/init.d/rc.mc_defaults
set MIXER quad_x                //MAIN mixer
set PWM_OUT 1234
```

而文件`rc.mc_defaults`内容如下：

```c
set VEHICLE_TYPE mc
param set-default IMU_GYRO_RATEMAX 800
param set-default NAV_ACC_RAD 2
param set-default RTL_RETURN_ALT 30
param set-default RTL_DESCEND_ALT 10
param set-default PWM_MAIN_MAX 1950
param set-default PWM_MAIN_MIN 1075
param set-default PWM_MAIN_RATE 400
param set-default GPS_UBX_DYNMODEL 6
set MIXER_AUX pass                   // AUX mixer
set PWM_AUX_OUT 1234
```

混控文件`mixer/quad_x.main.mix`内容如下：

```
R: 4x

AUX1 Passthrough
M: 1
S: 3 5  10000  10000      0 -10000  10000

AUX2 Passthrough
M: 1
S: 3 6  10000  10000      0 -10000  10000

Failsafe outputs
The following outputs are set to their disarmed value
during normal operation and to their failsafe falue in case
of flight termination.
Z:
Z:
```

