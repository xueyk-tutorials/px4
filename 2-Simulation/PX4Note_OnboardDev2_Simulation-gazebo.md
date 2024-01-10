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
推荐使用Ubuntu自带的software update进行显卡驱动的更新。如果显卡驱动更新后gazebo显示画面还是黯淡无光，需要进行一下显卡性能配置：
```shell
$ nvidia-settings
```
然后在NVIDIA X Server Settings界面中左侧选择PRIME Profiles，然后勾选NVIDIA(Performance Mode)，重启即可。

### 下载PX4源码

- 配置全局加速，防止GitHub仓库下载出现问题：

```shell
$ git config --global url."https://github.91chi.fun/https://github.com/".insteadOf https://github.com/
```

- 下载源码

```shell
$ git clone https://github.com/PX4/PX4-Autopilot.git
```

- 检出版本v1.12.3

```shell
$ cd PX4-Autopilot
$ git tag					# 查看可以选择的版本
$ git checkout v1.12.3    	# 比如此处选择 v1.12.3 版本
```

- 更新子仓库

```shell
$ git submodule sync --recursive
$ git submodule update --init --recursive
```

### 下载依赖

运行`ubuntu.sh`进行基本配置。

```shell
$ cd PX4-Autopilot
# 选择1.全部下载安装（推荐）
$ bash ./Tools/setup/ubuntu.sh
# 选择2.忽略nuttx
$ bash ./Tools/setup/ubuntu.sh --no-nuttx 
# 选择3.忽略sim工具
$ bash ./Tools/setup/ubuntu.sh --no-sim-tools
```

### QGC安装

QGC（QGroundControl）是PX4飞控的地面控制站。

安装请参考: https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html

## 单机仿真

新建一个命令窗口，进行编译

```shell
$cd PX4-Autopilot
$make px4_sitl gazebo
```

编译完成会弹出gazebo窗口，显示一个四旋翼iris无人机，可以通过运行QGC进行飞行控制。

其他仿真编译命令：

- 垂起仿真

  ```shell
  $ make px4_sitl gazebo_standard_vtol
  ```

- 车辆

  ```shell
  $ make px4_sitl gazebo_rover
  ```

  

## 多机仿真

PX4的Tools目录下提供了多机仿真启动脚本**gazebo_sitl_multiple_run.sh**。

具体请参考：https://docs.px4.io/master/en/simulation/multi_vehicle_simulation_gazebo.html。

### 命令格式

`.Tools/gazebo_sitl_multiple_run.sh [-m <model>] [-n <number_of_vehicles>] [-w <world>] [-s <script>] [-t <target>] [-l <label>]`

- -m：选择无人机类型

  默认m=iris。启动后会根据无人机类型加载gazebo模型，故必须保证对应模型存在，否则出错。目前支持的模型有iris、plane、standard_vtol、 rover、r1_rover、typhoon_h480。

- -n：设置无人机数量

  默认n=2。

- -w：设置gazebo仿真世界模型



### 更改脚本权限

一般这个脚本没有sudo权限，请使用如下命令赋予该脚本启动权限：

```shell
$ cd PX4-Autopilot
$ sudo chmod +777 ./Tools/gazebo_sitl_multiple_run.sh
```

> 注意：最好启动多机前，先运行下单机仿真！

### 启动多机仿真

输入如下命令

```shell
$ cd PX4-Autopilot
$ ./Tools/gazebo_sitl_multiple_run.sh -m iris -n 4
```

这个会启动gazebo仿真窗口并且具备四个无人机，每个无人机都会启动用于地面站gcs连接的mavlink和用于offboard连接的mavlink，默认IP和端口如下：

| 无人机 | offboard UDP                 | gcs UDP                      | sys_id/comp_id |
| ------ | ---------------------------- | ---------------------------- | -------------- |
| 1      | udp://:14540@127.0.0.1:14580 | udp://:14550@127.0.0.1:18570 | 1/1            |
| 2      | udp://:14541@127.0.0.1:14581 | udp://:14550@127.0.0.1:18571 | 2/1            |
| 3      | udp://:14542@127.0.0.1:14582 | udp://:14550@127.0.0.1:14582 | 3/1            |
| 4      | udp://:14543@127.0.0.1:14583 | udp://:14550@127.0.0.1:14583 | 4/1            |

- gcs连接：打开QGC后，只需要监听本地14550端口即可完成多机连接；
- offboard连接：用于启动自定义的机载控制程序后，根据要连接无人机编号对应的UDP地址设置url即可，例如使用mavsdk连接无人机3，则需要创建UDP（本地端口为14542，远端主机127.0.0.1，远端端口14582）。

### 选择机架

在ROMFS/px4fmu_common/init.d-posix/airframes中，定义了很多不同的机架文件，统一命令格式为：idx_name，其中idx是使用数字表示的机架编号，name是机架名称。我们可以通过查看机架名称作为命令参数，例如垂起无人机机架文件是1140_standard_vtol，那么可以通过如下命令启动垂起机型的多极仿真：

```shell
$ ./Tools/gazebo_sitl_multiple_run.sh -m standard_vtol -n 4
```

> 注意：
>
> 但由于gazebo模型库的限制，并不是所有机型都有，当前支持的仿真机架有[iris plane standard_vtol rover r1_rover typhoon_h480]

### 问题和解决

#### 无人机模型鬼畜

如果模型因尺寸问题彼此有冲突或者跟地面有冲突可能导致模型鬼畜现象。

打开`Tools/gazebo_sitl_multiple_run.sh`修改如下：

```shell
# 将加载模型时放置到世界中时彼此Y轴距离加大（3->10）
Y=${Y:=$((10*${N}))}

# 将加载模型时放置到世界中的坐标z加大一些（0->0.5)
gz model --spawn-file=/tmp/${MODEL}_${N}.sdf --model-name=${MODEL}_${N} -x ${X} -y ${Y} -z 0.5
```



## mavlink设置

​		默认情况下，px4_sitl启动后仅仅与所在运行环境上的QGC等通信，假如px4仿真所在的计算机与QGC所在的计算机不是同一个，但二者在同一个局域网，那么如何实现px4仿真与QGC的通信呢？

​		我们可以通过设置mavlink实例启动参数，指定QGC所在计算机的IP和端口。

> 通用适用于WSL2运行px4仿真的情况。

**修改启动脚本**

打开文件`build\px4_sitl_default\etc\init.d-posix\px4-rc.mavlink`，修改如下：

```shell
mavlink start -x -u $udp_offboard_port_local -r 4000000 -f -m onboard -o $udp_offboard_port_remote -t 192.168.1.6 -p
```

**在shell中启动**

```shell
mavlink start -x -u 14220 -r 4000000 -f -m onboard -o 14580 -t 192.168.1.6 -p
```

**参数如下**

- -t：远端IP（QGC或控制程序所在计算机IP）
- -o：远端端口（QGC或控制程序所在计算机接收端口）
- -u：本地端口（仿真端口）

## QGC连接

启动PX4 gazebo仿真后，可以根据终端窗口打印出的相关信息确定连接配置。例如启动仿真输出的相关信息如下：

```shell
[Msg] Waiting for master.
[Msg] Connected to gazebo master @ http://127.0.0.1:11345
[Msg] Publicized address: 192.168.215.89


INFO  [simulator] Waiting for simulator to accept connection on TCP port 4560
INFO  [simulator] Simulator connected on TCP port 4560.

INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 18570 remote port 14550
INFO  [mavlink] mode: Onboard, data rate: 4000000 B/s on udp port 14580 remote port 14540
INFO  [mavlink] mode: Onboard, data rate: 4000 B/s on udp port 14280 remote port 14030
INFO  [mavlink] mode: Gimbal, data rate: 400000 B/s on udp port 13030 remote port 13280
```

根据以上信息说明目的地址为**192.168.215.89**，启动了4路mavlink连接，一路Normal一般用于连接QGC，两路Onboard一般用于机载计算机连接，一路Gimbal一般用于吊舱连接。

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
source /home/alex/Desktop/PX4-Autopilot/Tools/setup_gazebo.bash /home/alex/Desktop/PX4-Autopilot/ /home/alex/Desktop/PX4-Autopilot/build/px4_sitl_default

source /mnt/e/D-drone/A1-PX4/Firmware/PX4-Autopilot_v1.12.3_learning/Tools/setup_gazebo.bash /mnt/e/D-drone/A1-PX4/Firmware/PX4-Autopilot_v1.12.3_learning/ /mnt/e/D-drone/A1-PX4/Firmware/PX4-Autopilot_v1.12.3_learning/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/mnt/e/D-drone/A1-PX4/Firmware/PX4-Autopilot_v1.12.3_learningt
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/mnt/e/D-drone/A1-PX4/Firmware/PX4-Autopilot_v1.12.3_learning/Tools/sitl_gazebo
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

