



## 环境配置



### 安装依赖

运行`roslaunch px4 indoor1.launch`前一定要安装**xmlstarlet**，命令如下：

```shell
$ sudo apt-get install xmlstarlet
```



### mavros

如果launch文件启动之后无法订阅mavros消息，请确定launch文件中的飞控FCU地址进行修改。



```shell
INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 18570 remote port 14550
[typhoon_h480_0/typhoon_h480_0_spawn-5] process has finished cleanly
log file: /home/allex/.ros/log/0d0d3582-c377-11ec-8070-58961d663221/typhoon_h480_0-typhoon_h480_0_spawn-5*.log
INFO  [mavlink] mode: Onboard, data rate: 4000000 B/s on udp port 34580 remote port 24540
INFO  [mavlink] mode: Onboard, data rate: 4000 B/s on udp port 14280 remote port 14030
INFO  [mavlink] mode: Onboard, data rate: 4000 B/s on udp port 14558 remote port 14530

```





尝试`mavros_posix_sitl.launch`和`outdoor1.launch`进行修改如下：

修改如下：

```xml
<!-- 把飞控FCU地址 --> 
<arg name="fcu_url" default="udp://:24540@localhost:34580"/>
<!-- 改为如下 --> 
<arg name="fcu_url" default="udp://:14540@localhost:14550"/>
```



直接启动mavros

```shell
$ roslaunch mavros px4.launch  fcu_url:="udp://:14540@127.0.0.1:14550"
```



### 吊舱

吊舱相机相关话题：

- /

吊舱云台相关话题如下：

- /mavros/mount_control/command

  接收用户发布的消息，进行吊舱控制。注意一定要无人机解锁起飞后才可以响应云台控制。

- /mavros/mount_control/orientation

  发布当前吊舱姿态角

- /mavros/mount_control/status



#### 控制吊舱

如果启动的是`outdoor1.launch`，吊舱订阅话题为`/typhoon_h480_0/mavros/mount_control/command'`，其消息类型为`mavros_msgs/MountControl`，如下：

```shell
$ rosmsg info mavros_msgs/MountControl
uint8 MAV_MOUNT_MODE_RETRACT=0
uint8 MAV_MOUNT_MODE_NEUTRAL=1
uint8 MAV_MOUNT_MODE_MAVLINK_TARGETING=2
uint8 MAV_MOUNT_MODE_RC_TARGETING=3
uint8 MAV_MOUNT_MODE_GPS_POINT=4
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint8 mode
float32 pitch
float32 roll
float32 yaw
float32 altitude
float32 latitude
float32 longitude
```

发布控制指令，控制吊舱俯仰角向下30°：

```shell
$ rospic pub /typhoon_h480_0/mavros/mount_control/command mavros_msgs/MountControl "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
mode: 2
pitch: -30.0
roll: 0.0
yaw: 0.0
altitude: 0.0
latitude: 0.0
longitude: 0.0"
```



#### 使用注意

- 无人机起飞后才可以控制吊舱
- 无人机在地面上的时候视觉上会一直抖动（暂不清楚原因），起飞后吊舱画面不稳定，但一旦发送吊舱控制指令，设定一个角度后画面稳定性就好了。

