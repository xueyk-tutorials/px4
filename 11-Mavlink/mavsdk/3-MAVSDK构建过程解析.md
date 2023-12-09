# 构建过程解析

## 说明

​		本篇文章主要记录下在对MAVSDK进行源码编译时对构建过程的一些理解和说明。

## cmake结构

​		MAVSDK通过cmake构建，故先简单介绍下MAVSDK构建文件和目录结构。

​		由于模块较多为了维护、开发方便，顶层CMakeLists.txt通过add_subdirectory()将一个个子级模块目录包含。

```shell
|-- mavsdk/
|-- src/
	# 核心代码（一定编译）包括接口驱动、mavsdk实现、各种插件实现
	|-- mavsdk/
		|-- core/    # 核心，Mavsdk、System类等
			|-- CMakeLists.txt
		|-- plugins/ # 各种插件，如telemetry、param、offboard
			|-- CMakeLists.txt
		|-- CMakeLists.txt
	# server，通过编译选项BUILD_MAVSDK_SERVER控制是否编译
	|-- mavsdk_server/       
		|-- CMakeLists.txt
	|-- third_party/ # 依赖库的下载、构建、编译 
    	|-- cmake/   # 用于编译各依赖库
    		|-- build_target.cmake 
    	|-- mavlink/ # mavlink库（一堆头文件）
    		|-- CMakeLists.txt
    	|-- jsoncpp/
    		|-- CMakeLists.txt
		|-- CMakeLists.txt
	|-- CMakeLists.txt
```

## 生成头文件

​		在构建过程中会生成两个文件，一个是版本信息、一个是mavlink头文件。

​		在`src/mavsdk/core/CMakeLists.txt`中，使用`configure_file()`函数生成。

```cmake
configure_file(version.h.in version.h)
configure_file(mavlink_include.h.in mavlink_include.h)
```

## mavlink构建

​		MAVSDK既然需要与无人机通信，其通信协议必然离不开mavlink，那么在编译过程中一定会用到mavlink库（一堆头文件而已）。提供mavlink库有两种方式，一种是从mavlink仓库下载（默认），一种是指定mavlink库目录路径。

### 使用头文件时用到的相关变量

​		构建mavlink时有两个变量需要注意：

- MAVLINK_HEADERS

  指定mavlink库目录路径（也就是那堆头文件放到哪了）；

- MAVLINK_DIALECT

  指定dialect（“方言”）；mavlink设计的很精巧，开发者可以定义消息、命令并生成自己的头文件，也就是自己的一套通信消息（称为dialect），默认的是common；

#### MAVLINK_DIALECT变量

​		在`MAVSDK/CMakeLists.txt`中定义了`MAVLINK_DIALECT`变量用于选择包含的mavlink库内的头文件，默认为MAVLINK_DIALECT=common。

```cmake
if (NOT MAVLINK_DIALECT)
    set(MAVLINK_DIALECT common)
endif()
```

#### MAVLINK_HEADERS变量

​		在`MAVSDK/third_party/CMakeLists.txt`中，定义了`MAVLINK_HEADERS`变量，用于指定mavlink头文件路径。

```cmake
if(NOT MAVLINK_HEADERS)
    build_target(mavlink)
    set(MAVLINK_HEADERS "${CMAKE_CURRENT_BINARY_DIR}/mavlink/mavlink/src/mavlink/include/")
    set(MAVLINK_HEADERS "${MAVLINK_HEADERS}" PARENT_SCOPE)
    message(STATUS "MAVLink headers for dialect ${MAVLINK_DIALECT} generated in ${MAVLINK_HEADERS}")
endif()
```

> 如果编译时指定了自定义的头文件，也就是命令行设置了MAVLINK_HEADERS的值，则会使用指定头文件路径。

### 构建方式一：下载	

1. 构建步骤

   在`third_party/CMakeLists.txt`构建时通过`build_target(mavlink)`进入mavlink的构建。build_target()函数定义在`third_party\cmake\build_target.cmake`中，该函数会通过`execute_process()`创建构建线程，对mavlink模块进行构建。

   启动构建线程后会调用cmake命令去构建`third_party\mavlink\CMakeLists.txt`，在其中通过`ExternalProject_add()`进行下载、编译、安装。

2. 下载和编译

​	下载mavlink仓库，然后编译。

​	最终编译后生成头文件路径为：

`MAVSDK/build/default/third_party/mavlink/mavlink/src/mavlink/include`

3. 添加头文件搜索路径

​		在`MAVSDK/src/mavsdk/CMakeLists.txt`中添加mavlink头文件路径

```cmake
target_include_directories(mavsdk
    SYSTEM PRIVATE ${MAVLINK_HEADERS}
)
```

4. 生成一个头文件

​		在`MAVSDK/src/mavsdk/core/CMakeLists.txt`中

```cmake
configure_file(mavlink_include.h.in mavlink_include.h)
```

​		其中`MAVSDK/src/mavsdk/core/mavlink_include.h.in`

```cpp
#pragma once
#include "mavlink/v2.0/@MAVLINK_DIALECT@/mavlink.h"
```

> 其中`MAVLINK_DIALECT`是一个CMake中定义的变量。

5. 包含mavlink头文件

​		需要使用mavlink库时只需要包含mavlink_include.h即可

```cpp
#include "mavlink_include.h"
```

### 构建方式二：指定头文件

- MAVLINK_DIALECT：指定用户dialect，默认MAVLINK_DIALECT=common，也就是使用common/mavlink.h头文件；
- MAVLINK_HEADERS：指定mavlink头文件所在路径与MAVSDK工程目录的相对路径，例如`-DMAVLINK_HEADERS=../mavlink-headers`表示头文件所在文件夹mavlink-headers是与MAVSDK文件夹同级。

```shell
$ cmake -Bbuild/default -DMAVLINK_DIALECT=mydialect -DMAVLINK_HEADERS=../mavlink-headers -H.
```

## 第三方库的构建

​		MAVSDK需要依赖一些第三方的库，MAVSDK在编译过程中会从仓库中下载这些库的源码然后编译成静态库，并且将这些静态库链接到MAVSDK生成的目标中。

### 依赖说明

- jsoncpp（必须）

  mission_raw插件会用到jsoncpp库，用于xx.plan文件（json格式）的加载和解析。在`src/npsdk/plugins/mission_raw/mission_import.h`中会包含<json/json.h>头文件。

- tinyxml2（必须）

  camera插件会用到tinyxml2库，用于加载和解析xml文件。在`src/npsdk/plugins/camera/camera_definition.h`中会包含<tinyxml2.h>头文件。

- curl（必须）

  相关文件为curl_include.h、curl_wrapper.h、http_loader.h。

Curl是一个命令行工具和库，用于进行数据传输。它支持多种协议，如HTTP、HTTPS、FTP、SMTP等，并可以通过URL进行数据传输。

Curl提供了一个简单的语法来发送HTTP请求和接收服务器响应。它可以执行各种操作，如发送GET请求、POST请求、上传文件、下载文件等。

另外Curl库会执行很多测试，包括检测环境等。

- zlib

> 其他大部分是mavsdk_server的依赖，这里不罗列了。

### 依赖库如何编译

1. 定义依赖库编译和安装路径

​		首先定义依赖库构建和安装路径（在主CMakeLists.txt中）。

```cmake
set(DEPS_BUILD_PATH "${PROJECT_BINARY_DIR}/third_party" CACHE PATH "Install path for the dependencies. Ignored if SUPERBUILD=OFF.")
set(DEPS_INSTALL_PATH "${DEPS_BUILD_PATH}/install" CACHE PATH "Install path for the dependencies. Ignored if SUPERBUILD=OFF.")
```

- DEPS_BUILD_PATH		

  这里`DEPS_BUILD_PATH`是依赖库构建时的工作路径，在MAVSDK构建路径`build/default`下创建`third_party`目录，并在该目录下创建与依赖库同名的文件夹做为该依赖库构建路径。

  对于jsoncpp来说，编译路径为：`build/default/third_party/jsoncpp`。

- DEPS_INSTALL_PATH

  在`DEPS_BUILD_PATH`路径下再创建install目录做为安装目录。

2. 编译依赖库

   **以编译jsoncpp为例**，通过`build_target(jsoncpp)`函数运行编译。该函数会通过两个`execute_process()`函数完成依赖库的构建、下载、编译、安装操作。

   **第一个**`execute_process()`函数启动一个cmake构建线程，该函数参数为：

   - COMMAND：设置bash命令，即/usr/bin/cmake；
   - WORKING_DIRECTORY：设置工作目录，即`npsdk/build/default/third_party/jsoncpp`目录；

   再根据为bash命令传入的参数，这个线程启动后就相当于运行了如下命令：

   ```shell
   $ cd third_party/jsoncpp
   $ cmake -DCMAKE_INSTALL_PREFIX:PATH=mavsdk/build/default/third_party/install \
   		-DCMAKE_PREFIX_PATH:PATH=${CMAKE_PREFIX_PATH} \
   		
   		mavsdk/third_party/jsoncpp
   ```

   > 注意：上面cmake后的编译参数没有写完，仅做为理解参考。

   其中cmake命令最后一个参数`mavsdk/third_party/jsoncpp`内包含CMakeLists.txt，这样就可以构建`third_party/jsoncpp/CMakeLists.txt`了。

   **第二个**`execute_process()`函数也是启动了一个cmake编译线程，参数与第一个一样，只不过是传入了`--build`命令，也就是相当于`make`命令。

   ```shell
   $ cd third_party/jsoncpp
   $ cmake --build .
   ```

3. 下载依赖库

   **以编译jsoncpp为例**，构建`third_party/jsoncpp/CMakeLists.txt`时，该CMakeLists.txt通过ExternalProject_Add()函数指定依赖库下载地址。由于步骤2启动编译线程时指定的工作路径为`npsdk/build/default/third_party/jsoncpp`目录，故依赖库源码也会下载到该目录下。

4. 编译安装

## 其他工具

### 测试

- gmock

gmock是google公司推出的一款开源的白盒测试工具

### 包管理工具

- hunter

Hunter 是 CMake 驱动的 C/C++ 跨平台包管理项目。

如果编译选项HUNTER_ENABLED=OFF（默认）则不会使用hunter，其`hunter_add_package()`函数不会执行任何操作。

## 附录

### cmake

#### get_filename_component()

获取文件路径