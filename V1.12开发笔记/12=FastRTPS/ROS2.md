# ROS2

## 概述

ROS2与PX4通过[microRTPS Bridge](https://docs.px4.io/main/en/middleware/micrortps.html)进行通信。

![](../imgs/ros2_microRTPS_bridge.png)



## 环境配置-agent

本节讲解如何配置agent(机载计算机)的环境。

### 依赖安装

- 安装基本包

```shell
$ sudo apt-get update 
$ sudo apt-get install --yes --no-install-recommends \
    git \
    build-essential \
    cmake \
    libssl-dev \
    libasio-dev \
    libtinyxml2-dev
```

- 安装java

```shell
$ sudo apt-get update
$ sudo apt-get install --yes --no-install-recommends \
    openjdk-8-jre-headless
$ sudo apt install openjdk-8-jdk
```

- 查看java版本

```
$ java -version
```

### 安装ROS2 Foxy

- 安装Foxy

  安装过程请参考官网，这里不再重复。

- 安装以下相关依赖

```shell
$ sudo apt install python3-colcon-common-extensions
$ sudo apt install ros-foxy-eigen3-cmake-module

$ sudo apt-get install pip
$ sudo pip3 install -U empy pyros-genmsg setuptools
$ sudo pip3 install launchpadlib
```

### 安装Fast DDS

如果使用ROS2 Dashing或Foxy进行开发，其默认已经安装了Fast DDS，便不用重新安装Fast DDS了。

通过如下命令查看已经安装的DDS包：

```shell
$ ros2 pkg list
rmw_dds_common
rmw_fastrtps_cpp
rmw_fastrtps_shared_cpp
```

> Fast DDS原本名称为FastRTPS，在版本v2.0.0之后才更名为Fast DDS。

### Fast-RTPS-Gen 

Fast-RTPS-Gen是一个java程序，它根据IDL定义的数据类型生成Fast RTPS（DDS）相关的代码，也就是说Fast-RTPS-Gen是IDL代码生成工具。

注意只能安装Fast-RTPS-Gen 1.0.4版本。

- 下载仓库

```shell
$ git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 ~/Fast-RTPS-Gen 
$ cd ~/Fast-RTPS-Gen/gradle/wrapper
```

- 修改gradle-wrapper.properties文件：

```shell
$ cd Fast-DDS-Gen/gradle/wrapper
$ vi gradle-wrapper.properties
```

将版本改为6.8.3即可，修改如下：

```shell
distributionUrl=https\://services.gradle.org/distributions/gradle-5.6.2-bin.zip
### 修改为
distributionUrl=https\://services.gradle.org/distributions/gradle-6.8.3-bin.zip
```

- 安装

使用以下命令，进行编译安装。

```shell
$ cd Fast-DDS-Gen
$ sudo chmod u+x gradlew
$ dos2unix gradlew
$ ./gradlew assemble && sudo env "PATH=$PATH" ./gradlew install
```

过程如下：

> Deprecated Gradle features were used in this build, making it incompatible with Gradle 7.0.
> Use '--warning-mode all' to show the individual deprecation warnings.
> See https://docs.gradle.org/6.8.3/userguide/command_line_interface.html#sec:command_line_warnings
>
> BUILD SUCCESSFUL in 28s
> 6 actionable tasks: 4 executed, 2 up-to-date
>
> 6 actionable tasks: 3 executed, 3 up-to-date
> Downloading https://services.gradle.org/distributions/gradle-6.8.3-bin.zip
> ......................................................................................................
>
> Welcome to Gradle 6.8.3!
>
> Here are the highlights of this release:
>  - Faster Kotlin DSL script compilation
>  - Vendor selection for Java toolchains
>  - Convenient execution of tasks in composite builds
>  - Consistent dependency resolution
>
> For more details see https://docs.gradle.org/6.8.3/release-notes.html
>
> Starting a Gradle Daemon (subsequent builds will be faster)
>
> Deprecated Gradle features were used in this build, making it incompatible with Gradle 7.0.
> Use '--warning-mode all' to show the individual deprecation warnings.
> See https://docs.gradle.org/6.8.3/userguide/command_line_interface.html#sec:command_line_warnings
>
> BUILD SUCCESSFUL in 1m 56s
> 1 actionable task: 1 executed
>
> 安装过程可能出现因为网络连接问题出现的异常，可以多试几次。

- 检测下fastrtpsgen是否成功安装

```shell
$ which fastrtpsgen
/usr/local/bin/fastrtpsgen
```

### px4_ros_com 和px4_msgs

- px4_msgs

  这里定义了uORB与DDS通信的所有消息，都是标准的`.msg`文件，通过`rosidl_generate_interfaces`将这些`.msg`文件生成c++头文件、idl文件。

- px4_ros_com

  包括了**micrortps_agent**

- 下载

```shell
$ git clone https://github.com/PX4/px4_ros_com.git ~/px4_ros_com_ros2/src/px4_ros_com
$ git clone https://github.com/PX4/px4_msgs.git ~/px4_ros_com_ros2/src/px4_msgs
```

> 如果github网络连接不稳定可能导致无法下载。可以通过在gitee上创建并拷贝github的仓库，然后再下载。

### 创建工程

- 首先建立工作空间

```shell
$ mkdir -p ~/px4_ros_com_ros2/src
```

- 将`px4_ros_com` 和`px4_msgs` 拷贝至 px4_ros_com_ros2/src

现在工程目录如下：

```shell
|-- px4_ros_com_ros2/
    |-- src/
        |-- px4_msgs/
        |-- px4_ros_com/
```

- 编译

通过`build_ros2_workspace.bash` 脚本编译ROS 2工作空间 (包括`px4_ros_com` 和`px4_msgs`).

```shell
$ cd ~/px4_ros_com_ros2/src/px4_ros_com/scripts
$ source build_ros2_workspace.bash
```

注意，如果编译过程有错误，可以转换一下格式

```shell
# 转换一下build_ros2_workspace.bash的文件格式
$ dos2unix build_ros2_workspace.bash
$ sudo chmod u+x /usr/local/bin/fastrtpsgen
$ dos2unix /usr/local/bin/fastrtpsgen
```

**编译过程可能较久，耐心等待。**

- 问题和解决

> 注意：有时候下载下来的文件格式是Windows的话，会报错：
>
> ```shell
> ubuntu@ubuntu:~/px4_fastRTPS/px4_ros_com_ros2_test/src/px4_ros_com/scripts$ source build_ros2_workspace.bash 
> : invalid option
> set: usage: set [-abefhkmnptuvxBCHP] [-o option-name] [--] [arg ...]
> : command not found
> -bash: build_ros2_workspace.bash: line 60: syntax error near unexpected token `$'in\r''
> 'bash: build_ros2_workspace.bash: line 60: `  case "$(lsb_release -s -c)" in
> ```
>
> 需要使用`dos2unix`转换一下，例如
>
> $ dos2unix build_ros2_workspace.bash

- 检测micrortps_agent

```shell
$ which micrortps_agent
/home/ubuntu/ROS2/px4_ros_com_ros2/install/px4_ros_com/bin/micrortps_agent
```



### 启动测试节点

在px4_ros_com包中包含了一些测试节点，这些节点程序路径为：`px4_ros_com/src/examples`，主要有三个文件夹：advertisers, listeners, offboard

首先添加一下ROS2工程环境变量：

```shell
$ source ~/px4_ros_com_ros2/install/setup.bash
```

然后可以通过`ros2 pkg list`命令，就可以看到px4_msgs、px4_ros_com已经安装。

然后通过`ros2 pkg executables px4_ros_com`命令可以看到编译好的节点：

```shell
$ ros2 pkg executables px4_ros_com
px4_ros_com debug_vect_advertiser
px4_ros_com offboard_control
px4_ros_com sensor_combined_listener
px4_ros_com vehicle_gps_position_listener
```

- 启动传感器消息订阅节点

```shell
$ ros2 launch px4_ros_com sensor_combined_listener.launch.py
```

- 启动offboard节点



## 飞控连接

参考：

https://docs.px4.io/main/en/ros/ros2_offboard_control.html

https://docs.px4.io/main/en/middleware/micrortps.html



### microRTPS

microRTPS包括了两个部分，一个是**运行在PX4固件中的client**，一个是**运行在机载计算机中的agent**。

#### client

```shell
> micrortps_client start|stop|status [options]
  -t <transport>          [UART|UDP] Default UART
  -d <device>             UART device. Default /dev/ttyACM0
  -l <loops>              How many iterations will this program have. -1 for infinite. Default -1.
  -w <sleep_time_ms>      Time in ms for which each iteration sleep. Default 1ms
  -b <baudrate>           UART device baudrate. Default 460800
  -p <poll_ms>            Time in ms to poll over UART. Default 1ms
  -r <reception port>     UDP port for receiving. Default 2019
  -s <sending port>       UDP port for sending. Default 2020
  -i <ip_address>         Select IP address (remote) values: <x.x.x.x>. Default: 127.0.0.1
```



#### agent

```shell
$ ./micrortps_agent [options]
  -t <transport>          [UART|UDP] Default UART.
  -d <device>             UART device. Default /dev/ttyACM0.
  -w <sleep_time_us>      Time in us for which each iteration sleep. Default 1ms.
  -b <baudrate>           UART device baudrate. Default 460800.
  -p <poll_ms>            Time in ms to poll over UART. Default 1ms.
  -r <reception port>     UDP port for receiving. Default 2019.
  -s <sending port>       UDP port for sending. Default 2020.
  -n <set namespace>      Set a namespace for the micrortps_agent.
```



### PX4仿真连接

1、下载PX4源码后，打开一个终端进入源码根目录进行编译

```shell
$ make px4_sitl_rtps gazebo
```

2、添加ROS2工作空间至环境变量，启动microRTPS

```shell
$ source ~/px4_ros_com_ros2/install/setup.bash
$ micrortps_agent -t UDP
--- MicroRTPS Agent ---
[   micrortps_agent   ] Starting link...
[   micrortps_agent   ] UDP transport: ip address: 127.0.0.1; recv port: 2020; send port: 2019; sleep: 1us
[ micrortps_transport ] UDP transport: Trying to connect...
[ micrortps_transport ] UDP transport: Connected to server!

---   Subscribers   ---
- DebugArray subscriber started
- DebugKeyValue subscriber started
- DebugValue subscriber started
- DebugVect subscriber started
- OffboardControlMode subscriber started
- OpticalFlow subscriber started
- PositionSetpoint subscriber started
- PositionSetpointTriplet subscriber started
- TelemetryStatus subscriber started
- Timesync subscriber started
- VehicleCommand subscriber started
- VehicleLocalPositionSetpoint subscriber started
- VehicleTrajectoryWaypoint subscriber started
- OnboardComputerStatus subscriber started
- TrajectoryBezier subscriber started
- VehicleTrajectoryBezier subscriber started
- VehicleMocapOdometry subscriber started
- VehicleVisualOdometry subscriber started
- TrajectorySetpoint subscriber started
-----------------------

----   Publishers  ----
- InputRc publisher started
- SatelliteInfo publisher started
- SensorCombined publisher started
- Timesync publisher started
- TrajectoryWaypoint publisher started
- VehicleAttitude publisher started
- VehicleControlMode publisher started
- VehicleLocalPosition publisher started
- VehicleOdometry publisher started
- VehicleStatus publisher started
- CollisionConstraints publisher started
- VehicleAngularVelocity publisher started
- VehicleTrajectoryWaypointDesired publisher started
```

3、回到PX4启动终端，启动microRTPS client

```shell
pxh> micrortps_client start -t UDP
```

4、新建一个终端，启动'listener'

```shell
$ source ~/px4_ros_com_ros2/install/setup.bash
$ ros2 launch px4_ros_com sensor_combined_listener.launch.py
```

如果一切正常，有如下结果打印：

```shell
RECEIVED DATA FROM SENSOR COMBINED
================================
ts: 870938190
gyro_rad[0]: 0.00341645
gyro_rad[1]: 0.00626475
gyro_rad[2]: -0.000515705
gyro_integral_dt: 4739
accelerometer_timestamp_relative: 0
accelerometer_m_s2[0]: -0.273381
accelerometer_m_s2[1]: 0.0949186
accelerometer_m_s2[2]: -9.76044
accelerometer_integral_dt: 4739
```

通过`ros2 topic hz`命令查看信息更新速度

```shell
ros2 topic hz /SensorCombined_PubSubTopic
###
average rate: 248.187
	min: 0.000s max: 0.012s std dev: 0.00147s window: 2724
average rate: 248.006
	min: 0.000s max: 0.012s std dev: 0.00147s window: 2972
average rate: 247.330
	min: 0.000s max: 0.012s std dev: 0.00148s window: 3212
average rate: 247.497
	min: 0.000s max: 0.012s std dev: 0.00149s window: 3464
```



### PX4飞控硬件连接

#### client PX4

```sh
make px4_fmu-v5_rtps
```

#### agent 机载计算机

```shell
```



## 问题和解决

问题Could not find a configuration file for package "ignition-common3-graphics"   

安装以下包即可

```shell
$ sudo apt-get install libignition-common3-graphics-dev
```





# 旧笔记

## 安装Fast DDS

> - Ubuntu 18.04: Fast RTPS 1.8.2 (or later) and Fast-RTPS-Gen 1.0.4 (not later!).
> - Ubuntu 20.04: Fast DDS 2.0.0 (or later) and Fast-RTPS-Gen 1.0.4 (not later!).

#### 安装Gradle

安装方法可以参考官网说明：https://gradle.org/install/#manually。

1、通过SDKMAN安装

我们推荐使用SDKMAN安装，首先安装SDKMAN

```shell
$ curl -s "https://get.sdkman.io" | bash
```

> 注意，安装完成后需要重新打开Terminal才生效。

然后检查java是否安装

```shell
alex@alex-xiaomi:/mnt/c/Users/alex$ java -version
openjdk version "1.8.0_292"
OpenJDK Runtime Environment (build 1.8.0_292-8u292-b10-0ubuntu1~20.04-b10)
OpenJDK 64-Bit Server VM (build 25.292-b10, mixed mode)
```

安装gradle

```shell
$ sdk install gradle 7.1.1
```

检查是否安装成功

```shell
$ gradle -v
```

2、手动安装

如果通过SDKMAN安装不成功，则只能手动安装：

1、下载源码

```shell
$ mkdir /opt/gradle
$ unzip -d /opt/gradle gradle-7.1.1-bin.zip
$ ls /opt/gradle/gradle-7.1.1
LICENSE  NOTICE  bin  getting-started.html  init.d  lib  media
```

2、增加环境变量

在`~/.bashrc`末尾添加

```bash
$ export PATH=$PATH:/opt/gradle/gradle-7.1.1/bin
```

3、检查

```shell
$ source ~/.bashrc
$ gradle -v
```



### 通过源码编译安装

#### Fast-RTPS (DDS)

```shell
### 下载源码
$ git clone --recursive https://github.com.cnpmjs.org/eProsima/Fast-DDS.git -b v2.0.0 ~/FastDDS-2.0.0
$ cd ~/FastDDS-2.0.0
$ mkdir build && cd build
### Linux环境安装
$ cmake -DTHIRDPARTY=ON -DSECURITY=ON ..
$ make -j$(nproc --all)
$ sudo make install
```

####  Fast-RTPS-Gen

```shell
$git clone --recursive https://github.com.cnpmjs.org/eProsima/Fast-DDS-Gen.git -b v1.0.4 ~/Fast-RTPS-Gen 
$ cd ~/Fast-RTPS-Gen 
$ ./gradlew assemble 
### 遇到问题
ubuntu@ubuntu:~/px4_fastRTPS/Fast-DDS-Gen$ ./gradlew assemble
/usr/bin/env: ‘sh\r’: No such file or directory
### 解决方法：转换一下文件格式
ubuntu@ubuntu:~/px4_fastRTPS/Fast-DDS-Gen$ dos2unix gradlew
dos2unix: converting file gradlew to Unix format...

$sudo ./gradlew install
```

### 通过Binaries安装

下载安装包

打开eprosima官网，进入下载界面https://www.eprosima.com/index.php/downloads-all，随便填写一些信息，选择下载eProsima Fast DDS。选择下载**eProsima Fast DDS 2.0.0 - Linux (32 & 64)**。

解压

```shell
tar -xzvf Fast-DDS-2.0.0.tar.gz
```

安装

参考[官网说明](https://fast-dds.docs.eprosima.com/en/latest/installation/binaries/binaries_linux.html)。

https://github.com/PX4/px4_msgs

https://github.com/PX4/px4_ros_com





## ROS2相关安装

#### 安装ROS2 Foxy

#### 安装相关依赖

```shell
sudo apt install python3-colcon-common-extensions
sudo apt install ros-foxy-eigen3-cmake-module
sudo pip3 install -U empy pyros-genmsg setuptools
sudo pip3 install launchpadlib
```

### 示例

PX4部分

```shell
###1
make px4_sitl_rtps gazebo
pxh>micrortps_client start -t UDP
###2
pxh>listener debug_vect
```

ROS2

```shell
###1
~/px4/px4_ros_com_ros2$source install/setup.bash
~/px4/px4_ros_com_ros2$ros2 topic echo /Sensor.....
###2
~/px4/px4_ros_com_ros2$ros2 run px4_ros_com debug_vect_advertiser
```



