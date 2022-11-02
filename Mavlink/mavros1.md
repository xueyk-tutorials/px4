# MAVROS基础

## 介绍

Mavros是由PX4团队基于mavlink协议开发的ROS工具包，用于机载计算机与飞控的数据交互，使用Mavros你可以很方便的通过订阅获取飞控数据消息，或者通过服务和发布消息发送控制指令。

[mavros wiki]: http://wiki.ros.org/mavros
[mavros_extras wiki]: http://wiki.ros.org/mavros_extras
[GitHub repo]: https://github.com/mavlink/mavros

## 安装

### 安装方式一：apt-get

```shell
# For ubuntu16.04 kinetic, use:
sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
# For ubuntu18.04 melodic, use:
sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras
# For ubuntu20.04 noetic, use:
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
```

### 安装方式二：源码

如果你需要进行二次开发，必然要修改源码，那么只能使用这种安装方式。

参考教程：

Ref:https://www.ncnynl.com/archives/201709/2077.html

Ref:https://blog.csdn.net/qq_38649880/article/details/88082360?utm_medium=distribute.pc_relevant_t0.none-task-blog-BlogCommendFromMachineLearnPai2-1.channel_param&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-BlogCommendFromMachineLearnPai2-1.channel_param

### 安装GeographicLib

方式一：

通过运行install_geographiclib_datasets.sh进行安装，注意这种方式需要在能连接其服务器时才行，很多时候因为网络问题并不能安装成功，所以不建议采用这种安装方式，但可以先试试，万一网络顺畅安装好就省事了。

```shell
### opt1
# kinetic
sudo /opt/ros/kinetic/lib/mavros/install_geographiclib_datasets.sh
# melodic
sudo /opt/ros/melodic/lib/mavros/install_geographiclib_datasets.sh 

### opt2
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
```


方式二：

下载缺少的GeographicLib文件，在没有安装GeographicLib之前你可以先运行一下mavros，可以看到弹出错误：即无法在路径`/usr/share/GeographicLib/geoids`下找到egm96-5.pgm文件，你可以从下面链接下载：
https://sourceforge.net/projects/geographiclib/files/geoids-distrib/
选择`egm96-5.tar.bz2`下载解压后拷贝至`/usr/share/GeographicLib/geoids`。

### 安装信息查看

#### 安装路径

mavros也是普通的ros工具包，其安装路径默认是ros路径下的**share**文件夹！

```bash
$rospack list
....
mavlink /opt/ros/melodic/share/mavlink
mavros /opt/ros/melodic/share/mavros
mavros_extras /opt/ros/melodic/share/mavros_extras
mavros_msgs /opt/ros/melodic/share/mavros_msgs
....
```

#### 版本查看

```shell
ubuntu@ubuntu:~$ rosversion mavros
1.4.0
ubuntu@ubuntu:~$ rosversion mavros_extras
1.4.0
```



# MAVROS开发

## 框架理解

![image-20210717133041044](https://gitee.com/bpnotes/pic-museum/raw/master/pictures/image-20210717133041044.png)

## 运行

### 启动mavros

在实际运行过程中，用户需要首先运行mavros节点， 一般通过其自带的launch文件来启动，mavros提供了几个基本的launch文件，在mavros安装路径`/opt/ros/melodic/share/mavros/launch`下有如下几个launch文件：

- apm.launch：用于与运行APM固件的飞控进行交互；
- px4.launch：用于与运行PX4固件的飞控进行交互；

如果你希望操作PX4飞控，那么你可以启动px4.launch文件，并且同时需要制定连接飞控的方式，mavros提供了很多URL地址连接方式，你可以用来连接FCU以及GCS，具体命令格式如下：

- Serial: `/path/to/serial/device[:baudrate]`
- Serial: `serial:///path/to/serial/device[:baudrate][/?ids=sysid,compid]`
- Serial with hardware flow control: `serial-hwfc:///path/to/serial/device[:baudrate][?ids=sysid,compid]`
- UDP: `udp://[bind_host][:port]@[remote_host][:port][/?ids=sysid,compid]`
- UDP Broadcast: `udp-b://[bind_host][:port]@[:port][/?ids=sysid,compid]`
- TCP client: `tcp://[server_host][:port][/?ids=sysid,compid]`
- TCP server: `tcp-l://[bind_host][:port][/?ids=sysid,compid]`

使用示例如下：

```bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:57600
#
roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600
#
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

- Note: if `FCU: DeviceError:serial:open: Permission denied` happened, use: **sudo chmod 777 /dev/ttyACM0**获取串口控制权。

启动mavros后，可以通过订阅mavros提供的话题查看飞控信息，例如查看IMU数据：

```shell
 $ rostopic echo /mavros/imu/data
```

也可以使用mavros提供的服务进行设置或控制：

```shell
  $ rosservice call /mavros/set_stream_rate 0 10 1
```



### mavros启动过程解析

我们以使用PX4飞控为例，运行如下命令进行与飞控的通信：

`$roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600`

px4.launch文件内容如下：

```html
<launch>
	<!-- vim: set ft=xml noet : -->
	<!-- example launch script for PX4 based FCU's -->

	<arg name="fcu_url" default="/dev/ttyACM0:57600" />
	<arg name="gcs_url" default="" />
	<arg name="tgt_system" default="1" />
	<arg name="tgt_component" default="1" />
	<arg name="log_output" default="screen" />
	<arg name="fcu_protocol" default="v2.0" />
	<arg name="respawn_mavros" default="false" />

	<include file="$(find mavros)/launch/node.launch">
		<arg name="pluginlists_yaml" value="$(find mavros)/launch/px4_pluginlists.yaml" />
		<arg name="config_yaml" value="$(find mavros)/launch/px4_config.yaml" />

		<arg name="fcu_url" value="$(arg fcu_url)" />
		<arg name="gcs_url" value="$(arg gcs_url)" />
		<arg name="tgt_system" value="$(arg tgt_system)" />
		<arg name="tgt_component" value="$(arg tgt_component)" />
		<arg name="log_output" value="$(arg log_output)" />
		<arg name="fcu_protocol" value="$(arg fcu_protocol)" />
		<arg name="respawn_mavros" default="$(arg respawn_mavros)" />
	</include>
</launch>
```

px4.launch set arg:config_yaml = px4_config.yaml and pluginlists_yaml = px4.pluginlists.yaml

px4.launch文件会调用node.launch文件，其路径为/opt/ros/melodic/share/mavros/launch。

## mavros参数

通过`px4.launch`启动mavros时会发现会加载**.yaml**文件也即是加载参数的过程，其中有很多参数你必须要了解

- fcu_url

​    这个参数是表示飞控的“连接地址”！最常用的是通过串口连接飞控，那么这个参数就是要指定串口的地址，例如计算机通过USB转TTL连接飞控的TELEM2，则该参数为`/dev/ttyUSB0:921600`，这里921600为波特率。

- tgt_system

​    这个参数代表mavros要连接的飞控的system ID，如果你更改了飞控的system ID，那么需要重新设置该参数。PX4飞控默认的飞控system ID为1，可通过地面站参数列表中**MAV_SYSTEM_ID**进行修改。

### gcs_bridge

mavros可以作为飞控与地面站通信的中间桥梁！只需要启动mavros时设置**gcs_url**参数即可，例如以下可以作为单机通信的launch文件。

```xml
<?xml version="1.0"?>
<launch>
    <!-- MAVROS configs -->
    <arg name="fcu_url" default="/dev/ttyUSB0:921600"/>
    <!-- GCS address -->
    <arg name="gcs_url" default="udp://:14540@192.168.43.12:14550"/>
    <arg name="fcu_system_id" default="201"/>
    <!-- MAVROS -->
    <include file="$(find mavros)/launch/px4.launch">
        <arg name="gcs_url" value="$(arg gcs_url)"/>
        <!-- fcu system id and component id -->
        <arg name="tgt_system" value="$(arg fcu_system_id)"/>
        <arg name="tgt_component" value="1"/>
    </include>
</launch>
```

mavros给出的**gcs_url**格式为：`UDP: udp://[bind_host][:port]@[remote_host][:port][/?ids=sysid,compid]`

注意

- 如果是多机同时与地面站通信，那么每个无人机需要设置好system ID和component ID。可以先使用USB数据线或数传连接飞控设置**MAV_SYS_ID**和**MAV_COMP_ID**参数，通常来说GCS将根据不同的**system ID**即可区分不同的无人机。
- 一旦修改了无人机system ID，在launch文件中必须要提供**tgt_system**参数用于说明FCU的system ID！



## 坐标系定义

#### body坐标系

- 在px4中，body坐标系为FRD坐标系，前为x正方向，右为y正方向，下为z正方向，mavlink协议中的ATTITUDE message包含的欧拉角就是根据此坐标系定义的。
- 在mavros中，body坐标系为baselink坐标系，前为x正方向，左为y正方向，上为z正方向。特别注意，如果是仿真环境body坐标系是RFU，即右为x正方向，前为y正方向，上为z正方向。

#### local坐标系

- 在px4中，local坐标系为NED坐标系，北为x正方向，东为y正方向，地为z正方向。
- 在mavros中，以无人机上电位置为坐标原点，local坐标系为ENU坐标系，东为x正方向，北为y正方向，天为z正方向。



## 消息与服务

### parameter

```
res = rospy.get_param('mavros/setpoint_attitude/use_quaternion')
rospy.loginfo("Use quaternion flag:{}".format(res))
rospy.set_param('mavros/setpoint_attitude/use_quaternion', True)
```

### /mavros/imu/data

data中包含的四元素是在ENU坐标系下定义的。

在仿真环境v1.11.0下，根据**orientation**使用`euler_from_quaternion()`函数获取的**euler**，其**yaw**输出如下：

```
                (North)
                   ^ pi/2
pi               |
<-----------|-----------> 0   (East)
-pi              |
```

也即**euler**为ENU坐标系

### velocity control

#### setpoint_velocity

control the velocity in global frame

topic:

/mavros/setpoint_velocity/cmd_vel
/mavros/setpoint_velocity/cmd_vel_unstamped

| topic                                       | msg                        |                                                              |
| ------------------------------------------- | -------------------------- | ------------------------------------------------------------ |
| /mavros/setpoint_velocity/cmd_vel           | geometry_msgs/TwistStamped | std_msgs/Header header<br/>geometry_msgs/Twist twist<br/>   geometry_msgs/Vector3 linear<br/>   geometry_msgs/Vector3 angular<br/> |
| /mavros/setpoint_velocity/cmd_vel_unstamped | geometry_msgs/Twist        | geometry_msgs/Vector3 linear<br/>geometry_msgs/Vector3 angular<br/> |



#### setpoint_raw/local

Local position, velocity and acceleration setpoint. 

**Topic:**

`~setpoint_raw/local` ([mavros_msgs/PositionTarget](http://docs.ros.org/en/api/mavros_msgs/html/msg/PositionTarget.html)) 

**PositionTarget.msg Definition**

```
# Message for SET_POSITION_TARGET_LOCAL_NED
#
# Some complex system requires all feautures that mavlink
# message provide. See issue #402.

std_msgs/Header header

uint8 coordinate_frame
uint8 FRAME_LOCAL_NED = 1
uint8 FRAME_LOCAL_OFFSET_NED = 7
uint8 FRAME_BODY_NED = 8
uint8 FRAME_BODY_OFFSET_NED = 9

uint16 type_mask
uint16 IGNORE_PX = 1 # Position ignore flags
uint16 IGNORE_PY = 2
uint16 IGNORE_PZ = 4
uint16 IGNORE_VX = 8 # Velocity vector ignore flags
uint16 IGNORE_VY = 16
uint16 IGNORE_VZ = 32
uint16 IGNORE_AFX = 64 # Acceleration/Force vector ignore flags
uint16 IGNORE_AFY = 128
uint16 IGNORE_AFZ = 256
uint16 FORCE = 512 # Force in af vector flag
uint16 IGNORE_YAW = 1024
uint16 IGNORE_YAW_RATE = 2048

geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration_or_force
float32 yaw
float32 yaw_rate
```

### attitude control

**topic:**

`~setpoint_raw/attitude` ([mavros_msgs/AttitudeTarget](http://docs.ros.org/en/api/mavros_msgs/html/msg/AttitudeTarget.html)) 

**msg definition:**

mavros_msgs/AttitudeTarget.msg

三个欧拉角，或者 4 元数

in px4_config.yaml:

```yaml
# setpoint_attitude
setpoint_attitude:
  reverse_thrust: false     # allow reversed thrust
  #before:
  #use_quaternion: false     # enable PoseStamped topic subscriber
  #after:
  use_quaternion: true     # enable PoseStamped topic subscriber
```



**topic:**

`~setpoint_raw/attitude` ([mavros_msgs/AttitudeTarget](http://docs.ros.org/en/api/mavros_msgs/html/msg/AttitudeTarget.html)) 

**msg definition:**

mavros_msgs/AttitudeTarget.msg

```
# Some complex system requires all feautures that mavlink
# message provide. See issue #402, #418.

std_msgs/Header header

uint8 type_mask
uint8 IGNORE_ROLL_RATE = 1 # body_rate.x
uint8 IGNORE_PITCH_RATE = 2 # body_rate.y
uint8 IGNORE_YAW_RATE = 4 # body_rate.z
uint8 IGNORE_THRUST = 64
uint8 IGNORE_ATTITUDE = 128 # orientation field

geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 body_rate
float32 thrust
```



```
quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
#type(pose) = geometry_msgs.msg.Pose
pose.orientation.x = quaternion[0]
pose.orientation.y = quaternion[1]
pose.orientation.z = quaternion[2]
pose.orientation.w = quaternion[3]

#type(pose) = geometry_msgs.msg.Pose
quaternion = (
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w)
euler = tf.transformations.euler_from_quaternion(quaternion)
roll = euler[0]
pitch = euler[1]
yaw = euler[2]
```



```
//! Old: A simple low pass filter implementation
// Taken from http://en.wikipedia.org/wiki/Low-pass_filter
double lowPassFilter(double x, double y0, double dt, double T) 
{
    double res = y0 + (x - y0) * (dt_/(dt_+T_));
    return res;
}
```

### Imu

**File: sensor_msgs/Imu.msg**

```
std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
```

**geometry_msgs/Vector3.msg**

```
float64 x
float64 y
float64 z
```

## 高级探究

为了研究MAVROS与飞控之间到底通信的消息是什么，

### 与MAVROS的通信

启动MAVROS

```xml
<?xml version="1.0"?>
<launch>
    <arg name="fcu_url" default="udp://:14540@127.0.0.1:14550"/>
    <arg name="fcu_system_id" default="1"/>
    <!-- MAVROS -->
    <include file="$(find mavros)/launch/px4.launch">
        <arg name="fcu_url" value="$(arg fcu_url)"/> 
    <arg name="tgt_system" value="$(arg fcu_system_id)"/>
    <arg name="tgt_component" value="1"/>
    </include>
</launch>
```



### 使用QGC通信

在QGC新建一个`Comm Links`如下：

没看到连接进度没关系，本身MAVROS不是一个飞机，QGC肯定无法像连接飞机一样连接MAVROS，但我们可以在`MAVLink Inspector`下看当前接收的消息。

会看到MAVROS发送过来的消息有：

```shell
HEARTBEAT 1.0Hz
SYSTEM_TIME 1.0Hz
TIMESYNC 10.0Hz
```

### 使用UDP-server通信

```python
import socket
import time
BUFSIZE = 1024
ip_port = ('', 14550)
server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server.bind(ip_port)

while True:
    data,client_addr = server.recvfrom(BUFSIZE)
    print('device addr:', client_addr)
    print('server recv:', data)
    #server.sendto(data.upper(),client_addr)

server.close()
```



## 踩坑总结

我们在使用mavros进行无人机控制时，遇到一些需要特别注意的地方，例如你在写自己的程序时应当：

1. 启动程序时最好等待几秒钟，因为mavros的消息订阅后，回调函数更新参数需要一定时间，等数据正确获取后再使用；
2. 使用**set_mode**服务设置无人机模式为**OFFBOARD**后，必须一直发布无人机控制消息，例如`mavros/setpoint_raw/local`，不然无人机的模式会自动切换至`LOIATOR`，你无法调用一次**set_mode**服务就让无人机切换至**OFFBOARD**模式！
3. 使用**set_mode**服务设置无人机模式为**OFFBOARD**的代码行后不能有sleep()类似的函数；
4. 关于**home_position**的问题， 话题`mavros/home_position/home`会发布home点在global和local坐标系下的坐标，大约1s更新一次，注意这个home点只在是每次检测到takeoff时更新，无人机移动过程中home点不变；
5. 在仿真环境下，body坐标系为RFU，而在真机中，body坐标系为baselink；

## Q&A

### Is /clock being published?

启动mavros仿真后，无法输出topic信息，报错如下：

```bash
alex@alex_laptop:~$ rostopic echo /mavros/imu/data
WARNING: no messages received and simulated time is active.
Is /clock being published?
```

网上很多说进行参数设置`rosparam set use_sim_time false`，然并卵！

最好使用命令`make px4_sitl_default gazebo`重新编译。













