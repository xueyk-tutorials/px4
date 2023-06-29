# px4-gazebo



设置无人机sdf

```xml
//注释掉默认的
    <!-- <arg name="sdf" default="$(find mavlink_sitl_gazebo)/models/$(arg vehicle)/$(arg vehicle).sdf"/> -->
//增加带有fpv的无人机
    <arg name="sdf" default="$(find mavlink_sitl_gazebo)/models/iris_fpv_cam/iris_fpv_cam.sdf"/>
```



启动

```shell
$ cd PX4-Autopilot/
$ roslaunch px4 mavros_posix_sitl.launch 
```



### 吊舱

typhoon_h480加载了吊舱插件（[gazebo_gimbal_controller_plugin](sitl_gazebo/src/gazebo_gimbal_controller_plugin.cpp)）



### 获取各模型信息

```shell
$ rostopic echo /gazebo/model_states
```





## 下载gazebo模型

```shell
$ cd ~/.gazebo/
$ mkdir -p models
$ cd ~/.gazebo/models/
$ wget http://file.ncnynl.com/ros/gazebo_models.txt
$ wget -i gazebo_models.txt
$ ls model.tar.g* | xargs -n1 tar xzvf
```

方式二：通过仓库下载

```shell
cd ~/.gazebo/
git clone https://github.com/osrf/gazebo_models.git models
```



## gazebo world

```shell
# 查看本地的默认worlds
$ ls /usr/share/gazebo-9/worlds/
```



# 问题

## 场景世界太暗

描述：打开gazebo后，即便使用默认的empty.world，整个画面很暗，主要原因是没有启用Nvidia显卡！

解决方法：

参考：https://www.yun88.com/news/1241.html

1、启动Nvidia显卡；

2、尝试在模型文件中，将地面材质由Gazebo/Grey更改为Gazebo/LightGrey
