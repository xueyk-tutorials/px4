# 简介

这篇文档简要记录下VSCode通过cmake-tools进行PX4构建仿真目标的流程，并对CMake相关代码做一些简单分析。

# 构建文件目录说明

相关构建文件目录如下：

```bash
|-- PX4-Autopilot/
	|-- CMakeLists.txt # 顶层CMakeLists.txt
	|-- cmake/
		|-- kconfig.cmake # kconfig用于代码裁剪
		|-- px4_add_library.cmake # 提供px4_add_library()函数，用于库的编译
		|-- px4_add_module.cmake  # 提供px4_add_module()函数，用于功能模块的编译
		|-- px4_config.cmake
		|-- px4_git.cmake
		|-- ...
	|-- boards/px4/sitl/
		|-- default.px4board # 板级配置
	|-- platforms/
		|-- posix/
			|-- CMakeLists.txt # 生成可执行程序
```



# 重要变量

仿真程序构建过程中会涉及到很多CMake变量，这里我们记录下比较重要的一些变量。

| 名称                           | 值                                             | 所在文件         | 说明                                                         |
| ------------------------------ | ---------------------------------------------- | ---------------- | ------------------------------------------------------------ |
| CONFIG                         | px4_sitl_default                               |                  |                                                              |
| PX4_SOURCE_DIR                 | PX4-Autopilot                                  | CMakeLists.txt   | 即${CMAKE_CURRENT_SOURCE_DIR}，由cmake-tools指定             |
| PX4_BINARY_DIR                 | PX4-Autopilot/build/px4_sitl_default           | CMakeLists.txt   | 即${CMAKE_CURRENT_BINARY_DIR}，由cmake-tools指定，根据setting.json内"cmake.buildDirectory"确定，根据开发者在cmake工具视图下variant的选择确定。 |
| CMAKE_BUILD_TYPE               | RelWithDebInfo                                 | CMakeLists.txt   |                                                              |
| PX4_CONFIG_FILE                | PX4-Autopilot/boards/px4/sitl/default.px4board | px4_config.cmake |                                                              |
| PX4_CONFIG                     | px4_sitl_default                               | px4_config.cmake |                                                              |
| PX4_BOARD                      | px4_sitl                                       | px4_config.cmake |                                                              |
| PX4_BOARD_NAME                 | PX4_SITL                                       | px4_config.cmake |                                                              |
| PLATFORM                       | posix                                          | kconfig.cmake    |                                                              |
| PX4_PLATFORM                   | posix                                          | kconfig.cmake    |                                                              |
|                                |                                                |                  |                                                              |
| CMAKE_BINARY_DIR               | PX4-Autopilot/build/px4_sitl_default           |                  |                                                              |
| CMAKE_RUNTIME_OUTPUT_DIRECTORY | PX4-Autopilot/build/px4_sitl_default/bin       |                  |                                                              |
|                                |                                                |                  |                                                              |



# 根目录下CMakeLists.txt 

## 初始化

创建变量保存源码路径，这里源码路径即项目根目录所在路径。

```cmake
set(PX4_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE FILEPATH "PX4 source directory" FORCE)
```

创建变量保存编译后的二进制所在路径，即build路径。

```cmake
set(PX4_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}" CACHE FILEPATH "PX4 binary directory" FORCE)
```

指定自定义cmake模块路径。

```cmake
list(APPEND CMAKE_MODULE_PATH ${PX4_SOURCE_DIR}/cmake)
```

> cmake知识点：
>
> CMAKE_MODULE_PATH关键字用来指定自定义的*.cmake文件路径，方便查找这些自定义的模块，进而使用include()包含这些模块。



## git处理

可以通过如下git命令获取项目tag版本信息：

```bash
$ git describe --exclude ext/* --always --tags
v1.14.3
```

## 属性定义

定义全局属性，如[PX4_MODULE_LIBRARIES](#PX4_MODULE_LIBRARIES)等。

```bash
define_property(GLOBAL PROPERTY PX4_MODULE_LIBRARIES
                 BRIEF_DOCS "PX4 module libs"
                 FULL_DOCS "List of all PX4 module libraries"
                 )

define_property(GLOBAL PROPERTY PX4_KERNEL_MODULE_LIBRARIES
                 BRIEF_DOCS "PX4 kernel side module libs"
                 FULL_DOCS "List of all PX4 kernel module libraries"
                 )

define_property(GLOBAL PROPERTY PX4_MODULE_PATHS
                 BRIEF_DOCS "PX4 module paths"
                 FULL_DOCS "List of paths to all PX4 modules"
                 )
define_property(GLOBAL PROPERTY PX4_SRC_FILES
                 BRIEF_DOCS "src files from all PX4 modules & libs"
                 FULL_DOCS "SRC files from px4_add_{module,library}"
                 )
```



## 配置

### 添加px4_add_module函数

```cmake
include(px4_add_module)
```

### 配置处理

```cmake
include(px4_config)
```

### 模块裁剪

```cmake
include(kconfig)
```

根据模块裁剪，设置如下变量：

- config_user_list
- config_module_list: 存放src/modules下要编译的模块名称；

- config_kernel_list
- config_romfs_extra_dependencies
- config_romfs_extra_files
- config_romfs_root

通过在命令终端输入`make px4_sitl_default boardconfig`，进入menuconfig配置界面，完成配置后可以按`S键`将配置保存至`build/px4_sitl_default/boardconfig`文件。

在编译过程中，根据配置文件生成`build/px4_sitl_default/px4_boardconfig.h`头文件。

### 外部模块处理

根据运行平台包含px4_impl_os.cmake，如果是在Ubuntu下仿真，故这里添加`platforms/posix/cmake`路径下的。

```cmake
include(platforms/${PX4_PLATFORM}/cmake/px4_impl_os.cmake)
```

根据运行平台添加cmake模块路径，如果是在Ubuntu下仿真，故这里添加`platforms/posix/cmake`路径。

```cmake
list(APPEND CMAKE_MODULE_PATH ${PX4_SOURCE_DIR}/platforms/${PX4_PLATFORM}/cmake)
```



## 设置工程编译过程

项目名称命名为px4。

```cmake
project(px4 CXX C ASM)
```

设置相关变量如下：

- CMAKE_BUILD_TYPE
- CMAKE_CXX_STANDARD: C++标准为14
- CMAKE_CXX_STANDARD_REQUIRED: ON，开启C++标准检查
- CMAKE_C_STANDARD: C标准为11
- CMAKE_C_STANDARD_REQUIRED: ON，开启C标准检查
- CMAKE_RUNTIME_OUTPUT_DIRECTORY: 运行输出路径，即编译目标目录，即build/px4_sitl_default

## 添加并构建子目录

通过add_subdirectory()添加并构建子目录。这里需要用到的子目录包括：

- src/lib: 通用库；

- platforms: 运行平台相关；
- platforms/posix: Ubuntu下仿真就是posix，**可执行文件就由该目录下CMakeLists.txt生成**；

- platforms/posix/src/px4: 

- boards/px4/sitl: 板子；

- src/module/???: 根据kconfig确定哪些模块编译；
- src/lib/events
- src/lib/metadata
- src/lib/parameters



### 根据kconfig添加module

模块裁剪之后生成config_module_list变量，存放了要编译的module列表。加入编译：

```cmake
foreach(module ${config_module_list})
	add_subdirectory(src/${module})
endforeach()
```



### 生成可执行文件

添加子目录`platforms/posix`。

```cmake
# firmware added last to generate the builtin for included modules
add_subdirectory(platforms/${PX4_PLATFORM})
```

在`platforms/posix/CMakeLists.txt`文件中，添加px4可执行文件。

```cmake
else()
	add_definitions(-DPX4_SOURCE_DIR="${PX4_SOURCE_DIR}" -DPX4_BINARY_DIR="${PX4_BINARY_DIR}")

	add_executable(px4
		src/px4/common/main.cpp
		apps.cpp)
endif()
```



链接各模块静态库，这里module_libraries就是从属性[PX4_MODULE_LIBRARIES](#PX4_MODULE_LIBRARIES)中获取的各静态库名称。

```cmake
target_link_libraries(px4
	PRIVATE
		${module_libraries}
		m
		parameters
)
```



## 生成uORB图

方便开发者更快速理清各模块通信和交互关系。

# cmake模块

## px4_parse_function_args.cmake

定义px4_parse_function_args()函数，在其他函数体头部被调用，用了解析其他函数被调时传入的参数。

重写cmake_parse_arguments函数。



## px4_add_library.cmake

定义px4_add_library()函数，用于构建静态库。

## px4_add_module.cmake

定义px4_add_module()函数，用于将指定模块编译成静态库/动态库。

该函数参数格式如下：

```cmake
px4_add_module(MODULE <string>
    MAIN <string>
    [ STACK_MAIN <string> ]
    [ STACK_MAX <string> ]
    [ COMPILE_FLAGS <list> ]
    [ INCLUDES <list> ]
    [ DEPENDS <string> ]
    [ SRCS <list> ]
    [ MODULE_CONFIG <list> ]
    [ EXTERNAL ]
    [ DYNAMIC ]
)
```

各参数含义为：

- MODULE	   : 模块库名称，必须独一不能重名，故一般都使用`模块所在上级目录_模块名`进行命名；
- MAIN		  	: 入口函数；
- STACK			: deprecated use stack main instead
- STACK_MAIN		: 给模块运行分配的栈大小；
- STACK_MAX	 	: maximum stack size of any frame
- COMPILE_FLAGS : 编译选项
- LINK_FLAGS		 : 链接标志
- SRCS			         : 源文件
- MODULE_CONFIG		: yaml config file(s)
- INCLUDES		   : 头文件
- DEPENDS			: 模块库依赖的目标
- EXTERNAL		   : flag to indicate that this module is out-of-tree
- DYNAMIC			: 不要编译进px4固件（也就是不生成静态库），而是编译成一个独立的动态库；
- UNITY_BUILD	  : merge all source files and build this module as a single compilation unit

例如PX4提供的示例模块中`src/examples/hello/CMakeLists.txt`使用了该函数：

```cmake
px4_add_module(
	MODULE examples__hello
	MAIN hello
	COMPILE_FLAGS
	SRCS
		hello_main.cpp
		hello_start.cpp
		hello_example.cpp
	DEPENDS
	)
```

编译hello模块后，会在`build/px4_sitl_default/src/examples/hello`下生成静态库（.a文件）。



将模块名称添加至属性PX4_MODULE_LIBRARIES，

```cmake
target_link_libraries(${MODULE} PRIVATE parameters_interface px4_layer uORB)
set_property(GLOBAL APPEND PROPERTY PX4_MODULE_LIBRARIES ${MODULE})
```



## px4_config.cmake

根据变量CONFIG，在项目目录下的boards文件夹内查找与之相同的`.px4board文件`。在仿真编译是变量CONFIG=px4_sitl_default，字符串以下划线分割，含义为vendor_model_label，即包含了供应商、型号、标签信息。

> CONFIG是定义在`.vscode/cmake-variants.yaml`中的，在编译的时候由cmake-tool调用cmake命令通过编译选项传递给CMakeLists.txt。

如果首次运行构建命令，则没有PX4_CONFIG_FILE变量，则会定义该变量。

如果找到`.px4board文件`则对应设置以下变量：

- PX4_CONFIG_FILE: PX4-Autopilot/boards/px4/sitl/default.px4board；
- PX4_BOARD_DIR: PX4-Autopilot/boards/px4/sitl；
- VENDOR: px4;
- MODEL: sitl;
- LABEL: default;
- PX4_BOARD: px4_sitl
- PX4_BOARD_NAME: PX4_SITL
- PX4_CONFIG: px4_sitl_default

增加编译目录，即`PX4-Autopilot/boards/px4/sitl/src`；

添加cmake子模块，即`PX4-Autopilot/boards/px4/sitl/sitl.cmake`。

## kconfig.cmake

根据选择的板子（这里是px4 sitl），将其对应目录下的所有*.px4board文件生成boardconfig文件。

- 输入：*.px4board文件位于`boards/px4/sitl`目录下；其中default.px4board文件指定了选择编译的模块。
- 输出：生成的文件位于`build/px4_sitl_default/boardconfig`文件中；



这里：

- PX4_SOURCE_DIR是项目源码根目录，即PX4-Autopilot；
- PX4_BOARD_DIR是板子目录，对于仿真则为PX4-Autopilot/boards/px4/sitl；
- COMMON_KCONFIG_ENV_SETTINGS：PYTHON_EXECUTABLE=/usr/bin/python3;KCONFIG_CONFIG=/home/amovlab/Desktop/repositories/PX4-Autopilot/build/px4_sitl_default/boardconfig;PLATFORM=posix;VENDOR=px4;MODEL=sitl;LABEL=default;TOOLCHAIN=;ARCHITECTURE=;ROMFSROOT=px4fmu_common
- DEFCONFIG_PATH：/usr/bin/python3;-m;defconfig；
- BOARD_DEFCONFIG：PX4-Autopilot/boards/px4/sitl/default.px4board。

```cmake
if(${LABEL} MATCHES "default" OR ${LABEL} MATCHES "recovery" OR ${LABEL} MATCHES "bootloader" OR ${LABEL} MATCHES "canbootloader")
    # Generate boardconfig from saved defconfig
    execute_process(
    COMMAND ${CMAKE_COMMAND} -E env ${COMMON_KCONFIG_ENV_SETTINGS}
    ${DEFCONFIG_PATH} ${BOARD_DEFCONFIG}
    WORKING_DIRECTORY ${PX4_SOURCE_DIR}
    OUTPUT_VARIABLE DUMMY_RESULTS
	)
```



生成px4_boardconfig.h文件（`build/px4_sitl_default/px4_boardconfig.h`），模块裁剪后生成的宏定义头文件。

```cmake
# Generate header file for C/C++ preprocessor
execute_process(
	COMMAND ${CMAKE_COMMAND} -E env ${COMMON_KCONFIG_ENV_SETTINGS}
		${GENCONFIG_PATH} --header-path ${PX4_BINARY_DIR}/px4_boardconfig.h
	WORKING_DIRECTORY ${PX4_SOURCE_DIR}
	OUTPUT_VARIABLE DUMMY_RESULTS
)
```

确定变量PLATFORM，后续会将PLATFORM赋值给变量PX4_PLATFORM。

```cmake
		if(Board)
			string(REPLACE "CONFIG_BOARD_" "" ConfigKey ${Name})
			if(Value)
				set(${ConfigKey} ${Value})
				message(STATUS "${ConfigKey} ${Value}")
			endif()
		endif()
```



# 附录

## PX4_MODULE_LIBRARIES

每一个模块编译为静态链接库之后，会将模块名称添加至属性<a id="PX4_MODULE_LIBRARIES">PX4_MODULE_LIBRARIES</a>中，这样在构建可执行文件时就方便将这些链接库链接进来。

例如在编译`src/modules/mc_att_control`时，生成libmodules__mc_att_control.a静态库。

当前仿真目标编译过程中，相关模块链接库如下：

```bash
lib__cdev__test__cdev_test;lib__controllib__controllib_test;lib__rc__rc_tests;modules__uORB__uORB_tests;lib__work_queue__test__wqueue_test;drivers__camera_trigger;drivers__gps;drivers__osd__msp_osd;drivers__tone_alarm;modules__airship_att_control;modules__airspeed_selector;modules__attitude_estimator_q;modules__camera_feedback;modules__commander;modules__control_allocator;modules__dataman;modules__ekf2;modules__events;modules__flight_mode_manager;modules__fw_att_control;fw_autotune_attitude_control;modules__fw_pos_control;modules__fw_rate_control;drivers__gimbal;modules__gyro_calibration;modules__gyro_fft;modules__land_detector;modules__landing_target_estimator;modules__load_mon;modules__local_position_estimator;modules__logger;modules__mag_bias_estimator;module__manual_control;modules__mavlink;modules__mavlink__mavlink_tests;modules__mc_att_control;mc_autotune_attitude_control;modules__mc_hover_thrust_estimator;modules__mc_pos_control;modules__mc_rate_control;modules__navigator;modules__payload_deliverer;modules__rc_update;modules__replay;modules__rover_pos_control;modules__sensors;modules__simulation__battery_simulator;modules__simulation__gz_bridge;modules__simulation__pwm_out_sim;modules__simulation__sensor_airspeed_sim;modules__simulation__sensor_baro_sim;modules__simulation__sensor_gps_sim;modules__simulation__senosr_mag_sim;modules__simulation__simulator_mavlink;modules__simulation__simulator_sih;modules__temperature_compensation;modules__uuv_att_control;modules__uuv_pos_control;modules__uxrce_dds_client;modules__vtol_att_control;systemcmds__actuator_test;systemcmds__bsondump;systemcmds__dyn;systemcmds__failure;systemcmds__led_control;systemcmds__param;systemcmds__perf;systemcmds__sd_bench;systemcmds__shutdown;systemcmds__system_time;systemcmds__tests;systemcmds__tests__hrt_test;systemcmds__topic_listener;systemcmds__tune_control;systemcmds__uorb;systemcmds__ver;systemcmds__work_queue;modules__fake_gps;modules__fake_imu;examples__fake_magnetometer;examples__hello;examples__px4_mavlink_debug;examples__px4_simple_app;examples__work_item
```

