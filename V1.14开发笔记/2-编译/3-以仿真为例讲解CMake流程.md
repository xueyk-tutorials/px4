# 构建流程



# 重要变量

以下是VSCode通过cmake-tools进行仿真程序构建过程中CMake相关变量。

| 名称            | 值                                             | 所在文件         | 说明                                                         |
| --------------- | ---------------------------------------------- | ---------------- | ------------------------------------------------------------ |
| PX4_SOURCE_DIR  | PX4-Autopilot                                  | CMakeLists.txt   | 即${CMAKE_CURRENT_SOURCE_DIR}，由cmake-tools指定             |
| PX4_BINARY_DIR  | PX4-Autopilot/build/px4_sitl_default           | CMakeLists.txt   | 即${CMAKE_CURRENT_BINARY_DIR}，由cmake-tools指定，根据setting.json内"cmake.buildDirectory"确定，根据开发者在cmake工具视图下variant的选择确定。 |
| PX4_CONFIG_FILE | PX4-Autopilot/boards/px4/sitl/default.px4board | px4_config.cmake |                                                              |
| PX4_CONFIG      | px4_sitl_default                               | px4_config.cmake |                                                              |
| PLATFORM        | posix                                          | kconfig.cmake    |                                                              |
| PX4_PLATFORM    | posix                                          | kconfig.cmake    |                                                              |
|                 |                                                |                  |                                                              |



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
- platforms/posix: Ubuntu下仿真就是posix；

- platforms/posix/src/px4: 

- boards/px4/sitl: 板子；

- src/module/???: 根据kconfig确定哪些模块编译；
- src/lib/events
- src/lib/metadata
- src/lib/parameters



## 生成uORB图

方便开发者更快速理清各模块通信和交互关系。

# cmake模块

## px4_parse_function_args.cmake

定义px4_parse_function_args()函数，在其他函数体头部被调用，用了解析其他函数被调时传入的参数。

重写cmake_parse_arguments函数。

## px4_add_module.cmake

### 作用

定义px4_add_module()函数，用于将指定模块编译成静态库/动态库。

### 函数用法

#### 格式

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

#### 参数

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

#### 示例

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

## px4_config.cmake

根据变量CONFIG，在项目目录下的boards文件夹内查找与之相同的`.px4board文件`。在仿真编译是变量CONFIG=px4_sitl_default，字符串以下划线分割，含义为vendor_model_label，即包含了供应商、型号、标签信息。

> CONFIG是定义在`.vscode/cmake-variants.yaml`中的，在编译的时候由cmake-tool传递给CMakeLists.txt。

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

根据选择的板子对应目录下default.px4board生成`build/px4_sitl_default/boardconfig`文件，这里default.px4board文件指定了选择编译的模块。

这里：

- PX4_SOURCE_DIR是项目源码根目录，即PX4-Autopilot；

- PX4_BOARD_DIR是板子目录，对于仿真则为PX4-Autopilot/boards/px4/sitl。

```cmake
# Generate boardconfig from default.px4board and {label}.px4board
execute_process(
    COMMAND ${CMAKE_COMMAND} -E env ${COMMON_KCONFIG_ENV_SETTINGS}
    ${PYTHON_EXECUTABLE} ${PX4_SOURCE_DIR}/Tools/kconfig/merge_config.py Kconfig ${BOARD_CONFIG} ${PX4_BOARD_DIR}/default.px4board ${BOARD_DEFCONFIG}
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



