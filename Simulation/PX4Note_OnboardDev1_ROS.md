# 飞控机载应用开发

​        无人机本身应该被定义为飞行平台，其根本功能就是保证稳定飞行，具体飞行目的和任务是什么不应该由飞行本身决定，应该由用户要解决的特定问题定义！而用户的问题可能多种多样，飞行任务也会千差万别，另外用户在使用无人机完成特定任务时不可避免的需要挂载各类载荷，那么用户如何才能够随意的操控无人机和载荷呢？

​        用户可以将问题的解决方案通过编程来实现，无人机平台就应该作为一个底层系统来准确、及时、稳定地响应用户的“自定义”操作，说白了就是需要提供SDK供用户调用，本教程将以通用任务如目标跟踪、避障、多机协同等为出发点，具体讲解机载开发所需的知识、无人机平台搭建以及实例演示！

## 系统架构

​		整个机载开发平台包括无人机平台和地面站，无人机平台包括飞控（PX4 V5+）、机载计算机（Jetson或树莓派等）、动力系统、通信模块，地面站包括运行QGC的Ubuntu系统计算机、通信模块，如果是对于短距离测试通信方式可以采用WiFi，即地面建立一个大功率路由器当做通信模块，记载端计算机就可以使用无线介入局域网了，如果是距离较远如大于300米，那么就需要支持网络层的通信模块。

​		其整个系统的连接示意图如下所示：

```bash
_________________
|       机载计算机          |
|                         |                           
|       |-------->(串口)  |<--------->PX4飞控TELEM2
| (应用程序)               |
|		^\			     |
|         \              |
|             (WiFi/网口) |<---------->地面无线路由器机载通信模块
_________________
																																				____________________																					|          地面计算机  |
                                        |	                  |
     地面无线路由器机载通信模块<----------->|------->内部端口(QGC)  |
                                       	|                     |
                                       	|-------->显控程序     |																			    ____________________
```

  - 机载计算机通过串口或者USB转串口模块连接飞控的串口（TELEM2），这个连接主要用于机载计算机获取飞控信息并发送控制指令，其传输协议为mavlink；
  - 地面与机载的通信我们选用ROS实现，即机载端运行ROS节点，地面计算机运行Master节点；
  - 机载计算机与地面的数据交互通过局域网交互，局域网的构建可以使用路由器或其他组网模块，这样可以用于机载节点与地面主节点进行数据传输；
  - 根据任务需求，应用开发可以分为两个部分，一个部分是机载计算机运行的应用程序，一个部分是地面计算机的显控程序部分，其中机载应用程序是实现不同应用场景下的飞行控制例如多机协同、目标识别和跟踪等，其侧重上层复杂应用开发，地面显控程序主要与QGC一起作为显控操作，下发应用指令，例如多机编队队形变换、启动跟踪等等。

## 地面的计算机配置

​		地面的计算机安装Ubuntu18.04系统，需要安装ROS（机器人操作系统）、MAVROS（飞控工具包）等。

### ROS

​		关于ROS的安装参考其官网指导说明：http://wiki.ros.org/melodic/Installation/Ubuntu，这里我们选择安装`ros-melodic-desktop-full`，这个是ROS完整版，所要要用的ROS包基本上都齐全。

### mavros

​        mavros的本质是使用ROS实现对底层的mavlink通信的封装，也就是说你不需要关系mavlink协议如何实现，也不需要关心串口编程或者UDP/TCP编程如何实现，只需要会ROS即可。运行ROS的计算机会自动解析mavlink数据并按类型分后发布到相应的话题，用户只需要订阅话题即可获取飞控状态信息，同样的，用户需要实现的飞行控制时，只需要向相应话题发布msg或者调用相应service即可实现对飞控的控制指令发送。

​        其安装说明可以参考如下：

```bash
$ sudo apt install ros-kinetic-mavros ros-kinetic-mavros-extras           # for ros-kinetic
$ sudo apt install ros-melodic-mavros ros-melodic-mavros-extras     # for ros-melodic
$ wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
$ sudo chmod a+x ./install_geographiclib_datasets.sh
$ sudo ./install_geographiclib_datasets.sh                                                     #这步需要装一段时间,请耐心等待PX4配置
```

​        其wiki可以去网站学习：http://wiki.ros.org/mavros。

## mavlink相关控制指令



#### SET_POSITION_TARGET_LOCAL_NED ([ #84 ](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED))

#### SET_POSITION_TARGET_GLOBAL_INT ([ #86 ](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT))



## 飞行平台搭建

### PX4飞控设置

#### 串口设置

由于机载计算机进行飞行控制时需要很高的实时性，mavros会检测通信实时性，如果串口波特率设置太低，则会提示`TM:RTT too high for timesync:17.85ms`，故应该配置高速波特率。

#### 使用QGC设置

QGC连接飞控后，在参数中搜索`SER_TEL1_BAUD`，该参数用来设置TELEM1串口的波特率，选择**921600 8N1**，重启飞控生效。

飞控的`SYS_COMPANION` 参数设置为`Companion Link (921600 baud, 8N1)`

### system id

确保mavros的launch文件中sys_id与飞控ID对应一致。

### 机载计算机配置

​		机载计算机我们仍然推荐使用Ubuntu18.04系统，在安装ROS的时候可以选择安装“小体积”的`ros-melodic-ros-base`版本，当然如何性能很高的机载计算机仍然可以安装完整版。

## 多机

​        在进行多机开发中，需要进行如下配置：

- 每个无人机的飞控system ID需要分别设定，默认的system ID=1（连接飞控后可通过QGC修改MAV_SYS_ID参数），例如我们可以将多机分别设置为201、202、203等；
- mavros在与飞控进行通信时需要根据飞控的system ID设置参数`target_system_id`，在`px4.launch`文件中可以看到这个参数由`tgt_system`设置；
- 每个机载计算机的hostname需要设置；
- 每个机载计算机运行一个节点，在进行局域网节点通信时，需要在机载计算机和地面计算机的`~/.bashrc`中进行相应修改。

### IP分配

​		我们假设局域网内各设备的IP分配如下：

1、地面计算机IP=192.168.43.12；

2、无人机1号机的IP=192.168.43.201，2号机IP=192.168.43.202，依次类推。

### 机载计算机配置

#### launch文件

​        如果进行多机控制，那么需要区分每个无人机，获取其信息并发送控制指令，我们可以在`.launch`文件中使用<group>标签来定义一个或多个命名空间，这样在每个命名空间下运行的mavros节点就可以相互独立。 

```xml
<?xml version="1.0"?>
<launch>
    <!-- namespace UAV1 -->
    <group ns="uav1">
        <!-- MAVROS and vehicle configs -->
        <arg name="fcu_url" default="/dev/ttyACM0:921600"/>
        <arg name="gcs_url" default="udp://:14540@192.168.43.12:14550"/>
		<arg name="fcu_system_id" default="201"/>
        <!-- MAVROS -->
        <include file="$(find mavros)/launch/px4.launch">
            <arg name="fcu_url" value="$(arg fcu_url)"/>
            <arg name="gcs_url" value="$(arg gcs_url)"/>
            <!-- setup FCU's system ID base on MAV_SYS_ID param showed in QGC -->
            <arg name="tgt_system" value="$(arg fcu_system_id)"/>
            <arg name="tgt_component" value="1"/>
        </include>
    </group>
</launch>
```

​		说明：上面这个`xxx.launch`文件需要放置到你的ROS工作空间下launch文件夹中，也就是你首先需要在机载计算机上创建workspace和package。

#### hostname

多机中，机载计算机hostname可以依次为rpi_flocking1、rpi_flocking2等，以1号机设置为例，编辑`/etc/hosts`文件，修改如下：

```
127.0.0.1	localhost
127.0.1.1	ubuntu
192.168.43.201    rpi_flocking1
192.168.43.12   alex_laptop

# The following lines are desirable for IPv6 capable hosts
::1     ip6-localhost ip6-loopback
fe00::0 ip6-localnet
ff00::0 ip6-mcastprefix
ff02::1 ip6-allnodes
ff02::2 ip6-allrouters
```

#### bashrc

为了进行同一个网络中，不同主机ROS节点通信，需要对`./bashrc`文件进行配置，添加ROS相关配置参数，以1号机为例：

```bash
export ROS_HOSTNAME=rpi_flocking1
export ROS_MASTER_URI=http://alex_laptop:11311
```

​		说明：在ROS配置中，bashrc文件内export行必须要在source行之后才行！

### 地面端计算机配置

#### hostname

机载计算机的hostname设置为alex_laptop。编辑`/etc/hosts`文件，修改如下：

```
127.0.0.1	localhost
127.0.1.1	alex_laptop
192.168.43.12    alex_laptop
192.168.43.201   rpi_flocking1
192.168.43.202   rpi_flocking2
192.168.43.203   rpi_flocking3

199.232.96.133 raw.githubusercontent.com
# The following lines are desirable for IPv6 capable hosts
::1     ip6-localhost ip6-loopback
fe00::0 ip6-localnet
ff00::0 ip6-mcastprefix
ff02::1 ip6-allnodes
ff02::2 ip6-allrouters
```

#### bashrc

```bash
export ROS_HOSTNAME=alex_laptop
export ROS_MASTER_URI=http://alex_laptop:11311
```



## 实例演示

### C++例程

本示例讲解如何使用ROS建立一个样例工程，用来控制无人机起飞，代码示例来自于[官网](https://dev.px4.io/master/en/ros/mavros_offboard.html)。

**建立ros工作空间**

创建工作空间，并且创建*offboard_takeoff*的package：

```bash
$cd ~/Desktop
$mkdir catkin_offboard
$cd catkin_offboard
$mkdir src
$cd src
$catkin_init_workspace
$catkin_create_pkg offboard_takeoff std_msgs mavros_msgs ros_py ros_cpp geometry_msgs
```

**在/src下添加程序**

在/src下新建文件offb_node.cpp，添加如下代码：

```c++
/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
```



**编译**

在CMakeLists.txt的*build*部分增加对文件*offb_node.cpp*文件的编译

```cmake
add_executable(offboard src/offb_node.cpp)
target_link_libraries(offboard ${catkin_LIBRARIES})
```

在工作空间下，运行*catkin_make*命令，编译并生成可执行程序。

**运行**

首先打开一个终端运行*mavros*：

```bash
$roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB0:57600
```

再打开一个终端运行编译生成的可执行程序*offboard*：

```bash
$rosrun offboard_takeoff offboard
```

### 仿真例程

In this example we will control drone fly in a circle while keep the drone heading the center of circle.

```python
import time
import sys
import rospy
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Header

target_velocity = Twist()
target_raw = PositionTarget()

rospy.init_node("set_velocity", anonymous=True)
rospy.loginfo("Test setpoint_velocity")

service_timeout = 30
try:
    rospy.wait_for_service('mavros/cmd/arming', service_timeout)
    rospy.wait_for_service('mavros/set_mode', service_timeout)
except ROSException:
    rospy.loginfo("ROS exception")
srv_arm  = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
srv_setmode = rospy.ServiceProxy("mavros/set_mode", SetMode)
pub_local_raw = rospy.Publisher('mavros/setpoint_raw/local', 
                                PositionTarget, queue_size=1)
def ctrl_arm_disarm(arm_flag):
    res = srv_arm(arm_flag)
    rospy.loginfo("Arm ctrl result {}".format(res))

def ctrl_set_mode(mode, timeout=10):
    base_mode   = 0
    custom_mode = mode
    res = srv_setmode(base_mode, custom_mode)
    rospy.loginfo("Set mode {} res: {}".format(mode, res))

def velocity_ctrl_raw():
    """
    Topic: ~setpoint_raw/local
    Function: set velocity and yaw rate in local frame, the drone will fly in a circle!
    """
    # Header
    target_raw.header = Header()
    # set coordinate frame
    target_raw.coordinate_frame = target_raw.FRAME_BODY_NED
    # ignore position and acc/force ctrl
    target_raw.type_mask = target_raw.IGNORE_PX | target_raw.IGNORE_PY | target_raw.IGNORE_PZ | target_raw.IGNORE_YAW
    target_raw.type_mask += target_raw.IGNORE_AFX | target_raw.IGNORE_AFY | target_raw.IGNORE_AFZ
    # just ctrl velocity and yaw rate
    target_raw.velocity.x = 2.0
    target_raw.yaw_rate = 0.5
    rate = rospy.Rate(10)

    ctrl_arm_disarm(True)
    ctrl_set_mode('AUTO.TAKEOFF')
    time.sleep(15)
    ctrl_set_mode('OFFBOARD')
    while not rospy.is_shutdown():
        target_raw.header.stamp = rospy.Time.now()
        pub_local_raw.publish(target_raw)
        #ctrl_set_mode('OFFBOARD')
        rate.sleep()

if __name__ == '__main__':
    velocity_ctrl_raw()



```

## 仿真

​		很多时候，在开发实际应用程序之前，都要在仿真环境中进行开发和测试，仿真环境的搭建可以参考**[XTDrone](https://www.yuque.com/xtdrone/manual_cn)**开源项目的使用文档，其代码在码云和GitHub上同步维护。

### 仿真环境搭建

参考：https://www.yuque.com/xtdrone/manual_cn/basic_config_1.11。

#### PX4_Firmware

首先有必要了解以下PX4_Firmare中关于仿真的文件结构。

```python
PX4_Firmware/              -- WORKSPACE
|--launch/
  |--mavros_posix_sitl.launch
  |--multi_vehicle.launch
|--Tools/                    
  |--sitl_gazebo/             -- used as package name: mavlink_sitl_gazebo($rospack list)
    |--models/                -- model definations
      |--iris/
        |--iris.sdf
        |--model.config
      |--M100/
      |--rotors_description/
        |--urdf/                           // 
          |--iris.xacro multirotor_base.xacro plane.xacro standard_vtol.xacro
        |--meshes/
        |--model.config                    //
```

### 运行

#### 编译PX4

```shell
$make px4_sitl_default gazebo
```

编译完成后会打开一个gazebo窗口，确定编译完成后，按ctrl+c停止这个进程运行即可。

#### 启动PX4 gazebo仿真环境

在*PX4_Firmware/launch*中有很多launch用于启动基于gazebo的仿真环境，可以通过如下命令启动：

```
$ roslaunch px4 mavros_posix_sitl.launch
```

以上命令会打开一个gazebo窗口并且显示一个四旋翼无人机，同时会启动mavros，接下来就可以通过GCS或者编写程序来控制无人机了。

**启动流程的相关解释：**

mavros_posix_sitl.launch 调用`px4.launch(/opt/ros/melodic/share/mavros/launch) `

px4.launch进行参数配置，加载文件`px4_config.yaml` and `px4.pluginlists.yaml`

px4.launch 调用`node.launch(/opt/ros/melodic/share/mavros/launch)`

#### 设置无人机类型

~/PX4_Firmware/Tools/sitl_gazebo/models
~/PX4_Firmware/Tools/sitl_gazebo/models/rotors_description/urdf

availiable vehicle type: iris, standard_vtol, plane

```xml
<launch>
    <!-- vehicle type -->
    <arg name="vehicle" default="standard_vtol"/>
    ...
    <group ns="uav0">
    	<include file="$(find px4)/launch/single_vehicle_spawn.launch">
            <arg name="vehicle" value="$(arg vehicle)"/>
        </include>
    </group>
</launch>
```

### 多机仿真

我们使用*PX4_Firmware/launch*中的*multi_uav_mavros_sitl.launch*文件启动多机的仿真环境，运行后可以生成名字为*uav0, uav1, uav2*三个mavros节点。

```
$ roslaunch px4 multi_uav_mavros_sitl.launch
... logging to /home/alex/.ros/log/9d888692-11e0-11eb-a921-e86a64796890/roslaunch-alex-31117.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

xacro: in-order processing became default in ROS Melodic. You can drop the option.
xacro: in-order processing became default in ROS Melodic. You can drop the option.
xacro: in-order processing became default in ROS Melodic. You can drop the option.
started roslaunch server http://alex:38969/

SUMMARY
========

CLEAR PARAMETERS
 * /uav0/mavros/
 * /uav1/mavros/
 * /uav2/mavros/

PARAMETERS
 * /gazebo/enable_ros_network: True
 * /rosdistro: melodic
 * /rosversion: 1.14.7
 * /uav0/mavros/cmd/use_comp_id_system_control: False
 * /uav0/mavros/conn/heartbeat_rate: 1.0
 * /uav0/mavros/conn/system_time_rate: 1.0
...
...
 * /uav0/mavros/wheel_odometry/wheel1/x: 0.0
 * /uav0/mavros/wheel_odometry/wheel1/y: 0.15
 * /uav0/rotors_description: <?xml version="1....
 ...
 ...
 * /uav1/mavros/cmd/use_comp_id_system_control: False
 * /uav1/mavros/conn/heartbeat_rate: 1.0
 * /uav1/mavros/conn/system_time_rate: 1.0
 * /uav1/mavros/conn/timeout: 10.0
 * /uav1/mavros/conn/timesync_rate: 10.0
 * /uav1/mavros/distance_sensor/hrlv_ez4_pub/field_of_view: 0.0
 * /uav1/mavros/distance_sensor/hrlv_ez4_pub/frame_id: hrlv_ez4_sonar
 ...
 ...
 * /uav2/mavros/cmd/use_comp_id_system_control: False
 * /uav2/mavros/conn/heartbeat_rate: 1.0
 * /uav2/mavros/conn/system_time_rate: 1.0
 * /uav2/mavros/conn/timeout: 10.0
 * /uav2/mavros/conn/timesync_rate: 10.0
 ...
 ...
NODES
  /
    gazebo (gazebo_ros/gzserver)
    gazebo_gui (gazebo_ros/gzclient)
  /uav0/
    iris_0_spawn (gazebo_ros/spawn_model)
    mavros (mavros/mavros_node)
    sitl_0 (px4/px4)
  /uav1/
    iris_1_spawn (gazebo_ros/spawn_model)
    mavros (mavros/mavros_node)
    sitl_1 (px4/px4)
  /uav2/
    iris_2_spawn (gazebo_ros/spawn_model)
    mavros (mavros/mavros_node)
    sitl_2 (px4/px4)
process[uav0/iris_0_spawn-5]: started with pid [31173]
INFO  [px4] Creating symlink /home/alex/PX4_Firmware/ROMFS/px4fmu_common -> /home/alex/.ros/etc

______  __   __    ___ 
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.
...
...
INFO  [px4] Calling startup script: /bin/sh etc/init.d-posix/rcS 1
process[uav1/iris_1_spawn-8]: started with pid [31200]
process[uav1/mavros-9]: started with pid [31201]
process[uav2/sitl_2-10]: started with pid [31204]
INFO  [px4] Creating symlink /home/alex/PX4_Firmware/ROMFS/px4fmu_common -> /home/alex/.ros/etc

______  __   __    ___ 
| ___ \ \ \ / /   /   |
| |_/ /  \ V /   / /| |
|  __/   /   \  / /_| |
| |     / /^\ \ \___  |
\_|     \/   \/     |_/

px4 starting.

```

启动的gazebo窗口会出现3架无人机，如下图：

![image-20210717132411796](https://gitee.com/bpnotes/pic-museum/raw/master/pictures/image-20210717132411796.png)

launch文件定义了三个无人机对应的节点，依次为*/uav0/mavros，/uav1/mavros，/uav2/mavros*，例如通过查看节点的结果如下：

```
alex@alex:~$ rosnode list
/gazebo
/gazebo_gui
/rosout
/rqt_gui_py_node_32167
/uav0/mavros
/uav1/mavros
/uav2/mavros
alex@alex:~$ 

```

通过运行*rqt_graph*获取的结果如下：

<img src="https://gitee.com/bpnotes/pic-museum/raw/master/pictures/multidrone_sim_node_graph.PNG" style="zoom:40%;" />







