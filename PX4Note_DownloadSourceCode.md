# 参考

https://docs.px4.io/master/en/dev_setup/building_px4.html

# 源码下载

## 下载主体代码

PX4的源码是托管在github上的，由于是国外平台，网络不稳定很容易导致无法下载。如果你计算机所在的网络环境很好，甚至能够接入外网，那么可以直接运行如下命令试试：

```shell
$git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

对于大部分国内开发者，比较好的方法是在码云gitee上新建一个仓库并选择导入[PX4的github仓库](https://github.com/PX4/PX4-Autopilot.git)，或者也可以在gitee上找找现有的，然后进行下载：

```shell
$ git clone https://gitee.com/boolpi/PX4-Autopilot.git
```

## 选择版本

这里我们以v1.12.0为例：

```shell
$ cd PX4-Autopilot
$ git tag					# 查看可以选择的版本
$ git checkout v1.12.0    	# 比如此处选择 v1.10.0 版本
```

## 下载依赖模块

由于PX4-Autopilot仓库源码也依赖很多其他开源代码，例如`mavlink`等，这行依赖的模块是由其他开发者独立维护的，熟悉git的人都知道，如果要引用其他模块，不需要把代码直接添加到自己的仓库中，只需要通过在项目根目录下的`.gitmodules`文件配置即可。

PX4项目根目录也有一个`.gitmodules`文件，详细罗列了各依赖模块的名称、仓库地址等等。由于这些仓库地址也都是放到github上的，在下载的时候非常慢。我们的做法是：

1. 将这些依赖模块的仓库地址修改为国内的镜像地址。

打开编辑器，编辑文件`.gitmodules`，将所有的**github.com**修改为**github.com.cnpmjs.org**。

2. 更新

修改了.gitmodules文件后，运行如下命令：

```shell
$ cd PX4-Autopilot
$ git submodule sync --recursive
$ git submodule update --init --recursive
```

这个命令会从国内镜像地址下载依赖的模块，下载完成后你会发现，这些依赖模块又依赖其他的模块！

3. 再次修改新增模块的.gitmodules

继续进入新生成的目录，如下：

- PX4-Autopilot/Tools/flightgear_bridge
- PX4-Autopilot/Tools/sitl_gazebo
- PX4-Autopilot/src/drivers/uavcan/libuavcan
- PX4-Autopilot/Tools/sitl_gazebo/external

然后依次修改.gitmodules文件，并在这些目录下依次运行`git submodule sync`，然后回到根目录下运行 `git submodule update --init --recursive`。

# 编译

PX4项目编译是通过cmake，cmake文件都在路径：`PX4-AUTOPILOT/boards/px4`下。

默认的cmake文件为：`PX4-AUTOPILOT/boards/px4/fmu-v5/src/default.cmake`。

可以通过如下命令查看当前可编译出的固件：

```shell
$ make list_config_targets
```

## 开发环境配置

在Ubuntu系统下配置PX4开发环境最简单的方法是运行ubuntu.sh脚本，这样就可以方便的安装包括NuttX/Pixhawk toolchain、Gazebo9、jMAVSim等。

命令如下：

```bash
# 选择1.全部下载安装（推荐）
bash ./Tools/setup/ubuntu.sh
# 选择2.忽略nuttx
bash ./Tools/setup/ubuntu.sh --no-nuttx 
# 选择3.忽略sim工具
bash ./Tools/setup/ubuntu.sh --no-sim-tools
```

脚本运行完成后，重启计算机即可。

## 各飞控平台编译选项

由于当前PX4支持开源飞控硬件平台非常多，所有需要根据硬件平台选择对应的固件编译。

各类[Pixhawk standard](https://docs.px4.io/master/en/flight_controller/autopilot_pixhawk_standard.html) boards的编译命令为：

- [Pixhawk 4](https://docs.px4.io/master/en/flight_controller/pixhawk4.html): `make px4_fmu-v5_default`
- [Pixhawk 4 Mini](https://docs.px4.io/master/en/flight_controller/pixhawk4_mini.html): `make px4_fmu-v5_default`
- [CUAV V5+](https://docs.px4.io/master/en/flight_controller/cuav_v5_plus.html): `make px4_fmu-v5_default`
- [CUAV V5 nano](https://docs.px4.io/master/en/flight_controller/cuav_v5_nano.html): `make px4_fmu-v5_default`
- [Pixracer](https://docs.px4.io/master/en/flight_controller/pixracer.html): `make px4_fmu-v4_default`
- [Pixhawk 3 Pro](https://docs.px4.io/master/en/flight_controller/pixhawk3_pro.html): `make px4_fmu-v4pro_default`
- [Pixhawk Mini](https://docs.px4.io/master/en/flight_controller/pixhawk_mini.html): `make px4_fmu-v3_default`
- [Pixhawk 2 (Cube Black)](https://docs.px4.io/master/en/flight_controller/pixhawk-2.html): `make px4_fmu-v3_default`
- [mRo Pixhawk](https://docs.px4.io/master/en/flight_controller/mro_pixhawk.html): `make px4_fmu-v3_default` (supports 2MB Flash)
- [Holybro pix32](https://docs.px4.io/master/en/flight_controller/holybro_pix32.html): `make px4_fmu-v2_default`
- [Pixfalcon](https://docs.px4.io/master/en/flight_controller/pixfalcon.html): `make px4_fmu-v2_default`
- [Dropix](https://docs.px4.io/master/en/flight_controller/dropix.html): `make px4_fmu-v2_default`
- [Pixhawk 1](https://docs.px4.io/master/en/flight_controller/pixhawk.html): `make px4_fmu-v2_default`

## 示例

以下是实际编译`CUAV V5+`飞控固件过程的输出结果：

```bash
alex@alex_laptop:~/Desktop/PX4-Autopilot$ make px4_fmu-v5_default
[0/1351] git submodule src/drivers/gps/devices
[2/1351] git submodule src/lib/ecl
[5/1351] git submodule src/drivers/uavcan/libuavcan
[9/1351] git submodule mavlink/include/mavlink/v2.0
[311/1351] Performing configure step for 'px4io_firmware'
-- PX4 version: v1.12.0-beta5-65-gb264577daa
-- PX4 config file: /home/alex/Desktop/PX4-Autopilot/boards/px4/io-v2/default.cmake
-- PX4 config: px4_io-v2_default
-- PX4 platform: nuttx
-- cmake build type: MinSizeRel
-- The CXX compiler identification is GNU 9.3.1
-- The C compiler identification is GNU 9.3.1
-- The ASM compiler identification is GNU
-- Found assembler: /opt/gcc-arm-none-eabi-9-2020-q2-update/bin/arm-none-eabi-gcc
-- Check for working CXX compiler: /opt/gcc-arm-none-eabi-9-2020-q2-update/bin/arm-none-eabi-g++
-- Check for working CXX compiler: /opt/gcc-arm-none-eabi-9-2020-q2-update/bin/arm-none-eabi-g++ -- works
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Check for working C compiler: /opt/gcc-arm-none-eabi-9-2020-q2-update/bin/arm-none-eabi-gcc
-- Check for working C compiler: /opt/gcc-arm-none-eabi-9-2020-q2-update/bin/arm-none-eabi-gcc -- works
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Detecting C compile features
-- Detecting C compile features - done
-- ccache enabled (export CCACHE_DISABLE=1 to disable)
-- Found PythonInterp: /usr/bin/python3 (found suitable version "3.6.9", minimum required is "3") 
-- build type is MinSizeRel
-- PX4 ECL: Very lightweight Estimation & Control Library v1.9.0-rc1-569-g71fc1b8
-- Configuring done
-- Generating done
-- Build files have been written to: /home/alex/Desktop/PX4-Autopilot/build/px4_fmu-v5_default/external/Build/px4io_firmware
[311/1351] Performing build step for 'px4io_firmware'
[0/244] git submodule platforms/nuttx/NuttX/nuttx
[2/244] git submodule platforms/nuttx/NuttX/apps
[242/244] Linking CXX executable px4_io-v2_default.elf
Memory region         Used Size  Region Size  %age Used
           flash:       58452 B        60 KB     95.14%
            sram:        3856 B         8 KB     47.07%
[244/244] Creating /home/alex/Desktop/PX4-Autopilot/build/px4_fmu-v5_default/external/Build/px4io_firmware/px4_io-v2_default.px4
[1349/1351] Linking CXX executable px4_fmu-v5_default.elf
Memory region         Used Size  Region Size  %age Used
      FLASH_ITCM:          0 GB      2016 KB      0.00%
      FLASH_AXIM:     1889441 B      2016 KB     91.53%
        ITCM_RAM:          0 GB        16 KB      0.00%
        DTCM_RAM:          0 GB       128 KB      0.00%
           SRAM1:       45748 B       368 KB     12.14%
           SRAM2:          0 GB        16 KB      0.00%
[1351/1351] Creating /home/alex/Desktop/PX4-Autopilot/build/px4_fmu-v5_default/px4_fmu-v5_default.px4

```

## 清除编译

如果需要清除编译结果，运行`make clean`即可。

```bash
$make clean
```

有时候如果clean不干净，运行`make distclean`。

```bash
$make distclean
```

# QGC安装

QGC（QGroundControl）是PX4飞控的地面控制站。

参考: https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html

## ubuntu下安装QGC

### 预先设置

Before installing *QGroundControl* for the first time:

1. On the command prompt enter:

   ```sh
   $ sudo usermod -a -G dialout $USER
   $ sudo apt-get remove modemmanager -y
   $ sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
   ```

2. Logout and login again to enable the change to user permissions.

###  安装

1. Download [QGroundControl.AppImage](https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage).

2. Install (and run) using the terminal commands:

   ```sh
   $ chmod +x ./QGroundControl.AppImage
   $ ./QGroundControl.AppImage  #(或者双击)
   ```

# 问题和解决

### 权限问题-make

**问题**

如果使用过`sudo make **`去编译，那么会造成build文件夹下很多内容权限变为了管理员所有，再次`make`时会报错如下：

```shell
ninja: error: opening build log: Permission denied The terminal process
```

**解决**

```shell
$ cd /PX4-Autopilot
$ sudo make clean
$ chmod -R 777 build/        # 更改文件夹内所有文件的权限
```

> 如果还是提示有些文件没有操作权限，再使用上面权限更改命令修改对应的文件夹！

### 权限问题-Tools/check_submodules.sh

[0/817] git submodule src/drivers/gps/devices
/bin/sh: 1: Tools/check_submodules.sh: Permission denied

```shell
$ cd PX4-Autopilot
$ sudo chmod +777 Tools/check_submodules.sh 
```

### FAILED: ROMFS/romfs_extract.stamp

Error

```
[0/1368] git submodule platforms/nuttx/NuttX/nuttx
[3/1368] git submodule platforms/nuttx/NuttX/apps
[4/1368] Generating romfs_extract.stamp
FAILED: ROMFS/romfs_extract.stamp 
cd /home/alex/Desktop/PX4-Autopilot/build/px4_fmu-v5_default/etc && /usr/bin/cmake -E remove_directory /home/alex/Desktop/PX4-Autopilot/build/px4_fmu-v5_default/etc/* && /usr/bin/cmake -E tar xf /home/alex/Desktop/PX4-Autopilot/build/px4_fmu-v5_default/romfs_files.tar && /usr/bin/cmake -E touch /home/alex/Desktop/PX4-Autopilot/build/px4_fmu-v5_default/ROMFS/romfs_extract.stamp
CMake Error: cmake version 3.10.2
Usage: /usr/bin/cmake -E <command> [arguments...]
Available commands: 

```

Solution

```
$git submodule update --recursive
$make distclean

```

