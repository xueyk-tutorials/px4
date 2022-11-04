# 仿真启动流程

## 设置~/.bashrc

为了保证正常的编译后的运行，需要设置下环境变量。假设PX4工程放置在用户根目录下，则在~/.bashrc添加如下内容：

```shell

### PX4-Autopilot
source ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot/ ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/sitl_gazebo

```



## 编译

### 命令

1. gazebo，四旋翼仿真 默认使用iris

   ```shell
   make px4_sitl_default gazebo
   ```

2. gazebo，垂起仿真

   ```shell
   make px4_sitl gazebo_standard_vtol
   ```

编译命令参数说明：

### 编译

#### sitl_gazebo模块编译

- 插件编译

  插件代码都在Tools/sitl_gazebo/src路径下。

> 注意Tools/simulation/gazebo/sitl_gazebo是与Tools/sitl_gazebo相同的代码，但编译时没使用

## PX4程序启动流程

### sitl_target.cmake

**使用add_custom_target函数生成target，这样用户就可以在命令行中调用make命令编译不同的target了。**

```cmake
add_custom_target(run_config
	COMMAND Tools/sitl_run.sh $<TARGET_FILE:px4> ${config_sitl_debugger} ${config_sitl_viewer} ${config_sitl_model} ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
	WORKING_DIRECTORY ${SITL_WORKING_DIR}
	USES_TERMINAL
	DEPENDS px4 logs_symlink
)
```

每个target中都会包括运行sitl_run.sh脚本的命令！

另外DEPENDS显示，这些target依赖px4

也就是用户执行make命令后会运行Tools/sitl_run.sh脚本，该脚本接受的第一个参数是px4可执行文件（其编译后在build/px4_sitl_default/bin/px4路径下）。

### 运行sitl_run.sh

该脚本所在路径为Tools/sitl_run.sh.

由sitl_target.cmake编译生成target。用户通过make调用target后会运行该脚本，并给定参数。

```shell
# 运行脚本需要传入的参数
sitl_bin="$1"           # PX4可执行文件
debugger="$2"           # 调试器，可以不指定，传入none即可
program="$3"            # 仿真器，当使用gazebo时，传入gazebo
model="$4"              # gazebo模型，如果不指定，将使用iris
world="$5"              # gazebo世界，如果不指定，将使用empty
src_path="$6"           # PX4程序路径，也就是工程根目录
build_path="$7"         # 编译后的程序所在路径，build/px4_sitl_default

# 打印仿真程序，根据编译命令输入的参数可知，一般为gazebo
echo program: $program

# 设置model，如果脚本参数没有设置，则设置默认值为iris
		echo "empty model, setting iris as default"
		model="iris"
# 设置变量PX4_SIM_MODEL
export PX4_SIM_MODEL=${model}


# 设置gazebo插件路径、模型路径
		source "$src_path/Tools/setup_gazebo.bash" "${src_path}" "${build_path}"

# 根据传入的编译好的PX4程序，准备参数
if [[ ${model} == test_* ]] || [[ ${model} == *_generated ]]; then
	sitl_command="\"$sitl_bin\" $no_pxh \"$src_path\"/ROMFS/px4fmu_test -s \"${src_path}\"/posix-configs/SITL/init/test/${model} -t \"$src_path\"/test_data"
else
	sitl_command="\"$sitl_bin\" $no_pxh \"$build_path\"/etc -s etc/init.d-posix/rcS -t \"$src_path\"/test_data"
fi
# 执行PX4程序
eval $sitl_command
```

如果已经完成编译，用户也可以直接运行该脚本，只要传入对应参数即可：

```shell
$ ./Tools/sitl_run.sh \
/media/allex/Develop/Develop_drone/A1-PX4/Firmware/PX4-Autopilot_v1.12.3_learning/build/px4_sitl_default/bin/px4 \
none \
gazebo \
none \
none \
/media/allex/Develop/Develop_drone/A1-PX4/Firmware/PX4-Autopilot_v1.12.3_learning \
/media/allex/Develop/Develop_drone/A1-PX4/Firmware/PX4-Autopilot_v1.12.3_learning/build/px4_sitl_default

```

### 启动PX4

一旦在sitl_run.sh中运行PX4仿真程序，就会运行`platforms/posix/src/px4/main.cpp `中的`run_startup_script()`函数，进而执行rcS脚本。

### 启动rcS

ROMFS/px4fmu_common/init.d-posix/rcS

```shell
# 重要环境变量说明：
# 其中PX4_SIM_MODEL为要启动的模型，根据用户的编译命令指定，例如make px4_sitl gazebo_standard_vtol编译命令，PX4_SIM_MODEL=standard_vtol


# 通过正则匹配，在etc/init.d-posix/airframes中找到名称符合[0-9]+_${PX4_SIM_MODEL}的文件，并保存文件编号至SYS_AUTOSTART
REQUESTED_AUTOSTART=$(ls "${R}etc/init.d-posix/airframes" | sed -n 's/^\([0-9][0-9]*\)_'${PX4_SIM_MODEL}'$/\1/p')


# 根据编译选项，启动airframes下的机架脚本
. "$autostart_file"

# 启动仿真
. px4-rc.simulator

# 启动mavlink连接
. px4-rc.mavlink
```

### 启动iris脚本

build/px4_sitl_default/tmp/rootfs/etc/init.d-posix/airframes/10016_iris

### 启动四旋翼脚本

etc/init.d/rc.mc_defaults



### 运行PX4模块

#### simulator

仿真模块代码路径为：src/modules/simulator

run函数在simulator_mavlink.cpp中

```shell
Simulator::run()
```

## gazebo启动流程

gazebo插件路径：

GAZEBO_PLUGIN_PATH : build/px4_sitl_default/build_gazebo

gazebo模型路径：

GAZEBO_MODEL_PATH : Tools/sitl_gazebo/models

> 默认情况下，使用的iris模型为Tools/sitl_gazebo/models/iris/iris.sdf

