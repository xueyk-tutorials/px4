# PX4仿真——gazebo

## 环境配置

### 安装gazebo

推荐通过安装ROS实现gazebo的安装。

查看gazebo版本命令：

```shell
$ dpkg -l | grep gazebo
```

### 安装显卡驱动

由于仿真对显卡有一定要求，推荐搭载英伟达独显并且安装好驱动（ubuntu默认安装的是nouveau驱动，无法真正调用显卡性能），显卡性能差或者驱动没安装会导致仿真画面黯淡无光。

### 下载PX4源码

请先参考PX4开发笔记1，下载PX4源码并运行`ubuntu.sh`进行基本配置。



## 单机仿真

新建一个命令窗口，进行编译

```shell
$cd PX4-Autopilot
$make px4_sitl gazebo
```

编译完成会弹出gazebo窗口，可以通过运行QGC进行飞行控制。

## 多机仿真

PX4的Tools目录下提供了多机仿真启动脚本**gazebo_sitl_multiple_run.sh**。

具体请参考：https://docs.px4.io/master/en/simulation/multi_vehicle_simulation_gazebo.html。

### 命令格式

`.Tools/gazebo_sitl_multiple_run.sh [-m <model>] [-n <number_of_vehicles>] [-w <world>] [-s <script>] [-t <target>] [-l <label>]`

### 更改脚本权限

一般这个脚本没有sudo权限，请使用如下命令赋予该脚本启动权限：

```shell
$ cd PX4-Autopilot
$ sudo chmod +777 ./Tools/gazebo_sitl_multiple_run.sh
```

> 注意：最好启动多机前，先运行下单机仿真！

### 启动多机仿真

```shell
$ cd PX4-Autopilot
$ ./Tools/gazebo_sitl_multiple_run.sh -m iris -n 4
```

这个会启动gazebo仿真窗口并且具备四个无人机，要对这四个无人机进行连接，请注意如下信息：

| 无人机 | offboard UDP                 | sys_id/comp_id |
| ------ | ---------------------------- | -------------- |
| 1      | udp://:14540@127.0.0.1:14580 | 1/1            |
| 2      | udp://:14541@127.0.0.1:14581 | 2/1            |
| 3      | udp://:14542@127.0.0.1:14582 | 3/1            |
| 4      | udp://:14543@127.0.0.1:14583 | 4/1            |

## gazebo仿真一些设置

### 地图设置

#### 改变仿真环境地图

在`Tools/sitl_gazebo/worlds/`目录下提供了很多地图，文件类型为.world。

通过给`-w`参数即可加载这个目录下的相应环境，例如加载baylands.world。

```shell
$ ./Tools/gazebo_sitl_multiple_run.sh -m iris -n 4 -w baylands
```

启动后需要等一会进行地图加载！

#### 设置地图原点坐标

另外，可以通过编辑相应的`.world`文件，更改原点对应的GPS坐标！

例如对`Tools/sitl_gazebo/worlds/baylands.world`进行如下修改：

```xml
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>29.377570</latitude_deg>
      <longitude_deg>104.622870</longitude_deg>
      <elevation>300</elevation>
    </spherical_coordinates>
```

# gazebo-ROS

## 编译

新建一个命令窗口，进行编译

```shell
$cd PX4-Autopilot
$make px4_sitl_default gazebo
```

编译完成会弹出gazebo窗口，可以通过运行QGC进行飞行控制。

## 配置~/.bashrc

```bash
source /opt/ros/melodic/setup.bash
source /home/alex/Desktop/PX4-Autopilot/ /home/alex/Desktop/PX4-Autopilot/Tools/setup_gazebo.bash  /home/alex/Desktop/PX4-Autopilot/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Desktop/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Desktop/PX4-Autopilot/Tools/sitl_gazebo
```

## 启动

新建一个命令窗口启动ROS

```shell
$roscore
```

新建一个命令窗口，通过launch文件启动

```shell
$cd PX4-Autopilot
$roslaunch px4 mavros_posix_sitl.launch
```

可以使用`rostopic list`查看mavros发布的无人机相关话题。

## 仿真

通过mavros编写仿真程序，进行仿真。

# jmavsim

新建一个命令窗口，进行编译

```shell
$ cd PX4-Autopilot
$ make px4_sitl jmavsim
```

# Airsim

```shell
$ cd PX4-Autopilot
$ make px4_sitl_default none_iris
```

