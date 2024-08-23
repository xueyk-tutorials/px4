

## 参考

### 开源项目volans

https://gitee.com/bingobinlw/volans/blob/master/doc/QuadGuidance.md



### 博文

#### 仿真Gazebo+PX4+VINS+FastPlanner

https://zhuanlan.zhihu.com/p/377419124

## gazebo教程

http://wiki.ros.org/navigation

### PX4官方教程

- 外部定位设备与PX4通信的mavlink消息

  https://docs.px4.io/master/en/ros/external_position_estimation.html#px4-mavlink-integration

- PX4  ECL EKF说明

  https://docs.px4.io/master/en/advanced_config/tuning_the_ecl_ekf.html#external-vision-system

- PX4支持T265

  https://docs.px4.io/master/en/peripherals/camera_t265_vio.html

- T265与mavros

  https://github.com/Auterion/VIO_bridge

- 树莓派部署t265+px4飞控实现无人机视觉定位

  https://blog.csdn.net/fcts1230/article/details/107958708



## 参考项目1

http://shequ.dimianzhan.com/articles/376

### 参数设置

**EKF2_AID_MASK**的参数改为`vision position fusion`，对应值为 “8”。

### Gazebo仿真

开启仿真环境

```shell
roslaunch px4 slam.launch
```

查看MAVROS的连接情况

```shell
rostopic echo /mavros/state
```

实时SLAM位置估计信息

```shell
rostopic echo /mavros/vision_pose/pose
```



## 设备介绍

### T265

 T265 包含两个鱼眼镜头传感器、一个 IMU 和一个英特尔® Movidius™ Myriad™ 2 VPU。所有的 V‑SLAM 算法都直接在 VPU 上运行，能够实现非常低的延迟和非常高效的功耗。T265 经过广泛的性能测试和验证，在预期使用条件下，闭环偏移小于 1%。姿态动作与动作反射之间的延迟不到 6 毫秒。

T265 的镜头上有一个红外截止滤光器，可以忽略 D400 系列深度摄像头的投影图案，故而二者可以同时使用！



**硬件分析**

| 硬件          | 参数                   | 备注   |
| ------------- | ---------------------- | ------ |
| 2个鱼眼摄像头 | 163±5°                 | OV9282 |
| 1个IMU        |                        | BMI055 |
| USB3.1        |                        |        |
| 尺寸          | 108 × 24.5 × 12.5 mm   |        |
| 重量          | 55g                    |        |
| 安装方式      | 两个M3螺丝孔，间距50mm |        |



## PX4视觉定位分析

### 数据获取

#### VIO

#### 点云

### 外部设备->EGO-Planner



### 外部设备->PX4(VIO数据输入)

在没有GPS条件下，PX4飞控可以接收其他设备（如深度相机、超宽带定位）提供的定位信息并进行EKF融合出最终的位置信息。

如何将外部设备定位信息传递给PX4呢？最根本上说还是外部设备通过[MAVLink消息发送给PX4](https://docs.px4.io/master/en/ros/external_position_estimation.html#px4-mavlink-integration)，PX4接收相关消息后映射至对应的uORB话题：

| MAVLink                                                      | uORB                      |
| ------------------------------------------------------------ | ------------------------- |
| [VISION_POSITION_ESTIMATE](https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE) | `vehicle_visual_odometry` |
| [ODOMETRY ](https://mavlink.io/en/messages/common.html#ODOMETRY)(`frame_id =` [MAV_FRAME_LOCAL_FRD) | `vehicle_visual_odometry` |
| [ATT_POS_MOCAP](https://mavlink.io/en/messages/common.html#ATT_POS_MOCAP) | `vehicle_mocap_odometry`  |
| [ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY)(`frame_id =` [MAV_FRAME_MOCAP_NED ) | `vehicle_mocap_odometry`  |

uORB将这些话题消息进一步传递给EKF2进行融合。

需要注意的是，这些MAVLink消息必须以30~50Hz的频率发送给PX4飞控。

#### 通过mavros传递信息

当我们希望使用深度相机进行定位时，需要将VIO信息提供给PX4，这时我们可以使用mavros了，因为它提供了“MAVLink接口”，将深度相机的数据传递给PX4！

**主要的topic**

当启动mavros后，可以看到如下几个比较重要的**topic**：

- /mavros/vision_pose/pose     

  **这个消息就是外部视觉模块得到VIO相关信息后发布给mavros，然后mavros再给PX4!**

  本话题要发送飞控(通过mavros_extras/src/plugins/vision_pose_estimate.cpp发送), 对应Mavlink消息为VISION_POSITION_ESTIMATE(#102), 对应的飞控中的uORB消息为vehicle_vision_position.msg 及 vehicle_vision_attitude.msg

- /mavros/vision_pose/pose_cov

  VIO的噪声，PX4内部EKF2进行位置信息融合时需要。好像很多开源工程中并没有使用它，也就是没有传递给PX4

- /mavros/odometry/in 

- /mavros/odometry/out

- /mavros/trajectory/desired

- /mavros/trajectory/generated

- /mavros/trajectory/path

**查看topic的消息类型**

```shell
alex@alex-Mi-Gaming-Laptop-15-6:~$ rostopic info /mavros/vision_pose/pose
Type: geometry_msgs/PoseStamped

Publishers: None

Subscribers: 
 * /mavros (http://alex-Mi-Gaming-Laptop-15-6:45493/)
alex@alex-Mi-Gaming-Laptop-15-6:~$ rostopic info /mavros/odometry/in 
Type: nav_msgs/Odometry
Publishers: 
 * /mavros (http://alex-Mi-Gaming-Laptop-15-6:45493/)
Subscribers: None

alex@alex-Mi-Gaming-Laptop-15-6:~$ rostopic info /mavros/odometry/out
Type: nav_msgs/Odometry
Publishers: None
Subscribers: 
 * /mavros (http://alex-Mi-Gaming-Laptop-15-6:45493/)

```



### 机载计算机->PX4(控制指令输入)

#### 发送控制指令

**Promentheus发送的Mavlink消息**

Prometheus项目控制飞控必须向飞控发送相关解/上锁、设定模式、设定目标点等指令，因此需向飞控发送相应的Mavlink消息（PX4代码中mavlink_receiver.cpp负责接收）。

此处仅列出必要的Mavlink消息：

- MAVLink Commands (MAV_CMD) 指令类消息
- SET_ATTITUDE_TARGET ( #82 )
- SET_POSITION_TARGET_LOCAL_NED ( #84 )

通过视觉获得定位后，应该可以发送`SET_POSITION_TARGET_LOCAL_NED`消息控制飞机移动。



## /mavros/vision_pose/pose 

这个可以有多种方式获得，例如从深度相机获得、激光雷达等等。

### 消息具体内容

```shell
geometry_msgs/PoseStamped

>>>
$ rosmsg info geometry_msgs/PoseStamped 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
```







## PX4飞控参数设置

### EKF2_AID_MASK

#### 解释

https://docs.px4.io/master/en/advanced_config/parameter_reference.html#EKF2_AID_MASK

**Bitmask:**

- **0:** use GPS
- **1:** use optical flow
- **2:** inhibit IMU bias estimation
- **3:** vision position fusion        融合外部视觉（SLAM）的位置估计出的位置数据
- **4:** vision yaw fusion
- **5:** multi-rotor drag fusion
- **6:** rotate external vision
- **7:** GPS yaw fusion
- **8:** vision velocity fusion

**Reboot required:** true

必须重启飞控才可以生效！

#### 参数设置

根据amovlab，该参数设置为vision position fusion+vision yaw fusion。
