# MAVSDK（二）源码下载编译安装

## 说明

​		这篇文档讲解下如何下载MAVSDK源码并进行编译和安装，与官网不同，针对国内开发者，讲解了下载和编译过程中的问题和解决方法。更多内容请参考MAVSDK官网源码编译说明：https://mavsdk.mavlink.io/main/en/cpp/guide/build.html。

​		这里使用的开发环境为：WSL2（Ubuntu20.04）。

## 下载

​		MAVSDK托管在GitHub，如果开发者有稳定可靠的网络可以直接在下载GitHub上的仓库。git命令如下：

```shell
# 下载MAVSDK源码
$ git clone https://github.com/mavlink/MAVSDK.git
# 下载子仓库
$ git submodule update --init --recursive
# 检出需要的版本（这里我们使用v1.4.10）
$ git checkout -b v1.4.10
```

​		如果网络条件不好，可能很长时间才能下载完成或者无法下载，这时为了保证下载可靠、提高下载速度可以使用gitee上的仓库镜像，这里我已经将MAVSDK依赖的子仓库都放到gitee上了，git命令如下：

```shell
# 下载MAVSDK源码
$ https://gitee.com/github-mirror-alex/MAVSDK.git
```

​		由于MAVSDK依赖一些子仓库，也是托管在GitHub上的，这里我也将这些依赖仓库放到了gitee上面，故下载了MAVSDK仓库代码后需要修改子仓库的下载链接。故修改.gitmodules文件如下：

```shell
  [submodule "gtest"]
  	path = src/third_party/gtest
  	url = https://gitee.com/github-mirror-alex/googletest
  [submodule "mavsdk-proto"]
  	path = proto
  	url = https://gitee.com/github-mirror-alex/MAVSDK-Proto.git
  [submodule "src/third_party/mavlink"]
  	path = src/third_party/mavlink
  	url = https://gitee.com/github-mirror-alex/mavlink
```

​		然后就可以更新子仓库了，运行git命令如下：

```shell
# 下载子仓库
$ git submodule update --init --recursive
```

​		最后检出开发者需要使用的版本，这里我们以v1.4.10为例：

```shell
$ git checkout -b v1.4.10
```

## 编译

### 修改第三方工具库下载链接

​		由于在编译过程中会下载依赖的第三方工具库，而这些依赖仓库也是托管在GitHub上的，同样，我也将这些依赖放到了gitee上，为了提高下载稳定性和速度，首先修改下载地址。

​		这些第三方工具库都是放到third_party目录下的，故修改third_party内的各工具的CMakeLists.txt文件。主要是修改`ExternalProject_Add`命令中`GIT_REPOSITORY`对应的链接地址，将其改为gitee仓库地址即可。

- jsoncpp

  修改`third_party/jsoncpp/CMakeLists.txt`如下：

  ```cmake
  ExternalProject_Add(
      jsoncpp
      # GIT_REPOSITORY https://github.com/open-source-parsers/jsoncpp
      GIT_REPOSITORY https://gitee.com/github-mirror-alex/jsoncpp
      GIT_TAG ed1ab7ac452b0fe51f3b0a8364770774175a060e
      PREFIX jsoncpp
      CMAKE_ARGS "${CMAKE_ARGS}"
      )
  ```

- tinyxml2

  修改`third_party/tinyxml2/CMakeLists.txt`如下：

  ```cmake
  ExternalProject_add(
      tinyxml2
      # GIT_REPOSITORY https://github.com/leethomason/tinyxml2
      GIT_REPOSITORY https://gitee.com/github-mirror-alex/tinyxml2
      GIT_TAG 55716da04f2de307e01d84ed695bed86114f9136
      PREFIX tinyxml2
      PATCH_COMMAND git checkout . && git apply ${PROJECT_SOURCE_DIR}/cmake-3.10.2.patch
      CMAKE_ARGS "${CMAKE_ARGS}"
      )
  ```

- zlib

  修改`third_party/zlib/CMakeLists.txt`如下：

  ```cmake
  ExternalProject_add(
      zlib
      # GIT_REPOSITORY https://github.com/madler/zlib
      GIT_REPOSITORY https://gitee.com/github-mirror-alex/zlib
      GIT_TAG v1.2.11
      PREFIX zlib
      # PATCH_COMMAND git checkout . && git apply ${PROJECT_SOURCE_DIR}/build_shared_libs.patch
      PATCH_COMMAND git checkout .
      CMAKE_ARGS "${CMAKE_ARGS}"
      )
  ```

- curl

  修改`third_party/curl/CMakeLists.txt`如下：

  ```cmake
  ExternalProject_add(
      curl
      # GIT_REPOSITORY https://github.com/curl/curl.git
      GIT_REPOSITORY https://gitee.com/github-mirror-alex/curl.git
      GIT_TAG curl-7_78_0
      GIT_SHALLOW ON
      PREFIX curl
      # PATCH_COMMAND git checkout . && git apply ${PROJECT_SOURCE_DIR}/curl.patch
      PATCH_COMMAND git checkout .
      CMAKE_ARGS "${CMAKE_ARGS}"
      )
  ```

- mavlink

  修改`third_party/mavlink/CMakeLists.txt`如下：

  ```cmake
  ExternalProject_add(
      mavlink
      # GIT_REPOSITORY https://github.com/mavlink/mavlink
      GIT_REPOSITORY https://gitee.com/github-mirror-alex/mavlink
      # GIT_TAG 3b52eac09c2e37325e4bc49cd2667ea37bf1d7d2
      GIT_TAG 3e303e54a7f8b9df960348a55218cc8a459525e0
      PREFIX mavlink
      CONFIGURE_COMMAND Python3::Interpreter
          -m pymavlink.tools.mavgen
          --lang=C
          --wire-protocol=2.0
          --output=<BINARY_DIR>/include/mavlink/v2.0/
          message_definitions/v1.0/${MAVLINK_DIALECT}.xml
      BUILD_COMMAND ""
      INSTALL_COMMAND ""
      BUILD_IN_SOURCE TRUE
  )
  ```

  

> 注意，如果在编译过程中，当下载第三方工具库后出现patch错误，就将该工具库内的CMakeLists.txt文件内的`git apply xxxx.patchh`注释即可。

### 默认编译

debug版本

```shell
cmake -DCMAKE_BUILD_TYPE=Debug -Bbuild/default -H.
cmake --build build/default -j8  # 相当于make
```

release版本

```shell
cmake -DCMAKE_BUILD_TYPE=Release -Bbuild/default -H.
cmake --build build/default -j8  # 相当于make
```

### 不编译test（可选）

​		有时为了提高编译进度，可以将不重要的模块取消编译。这里我们取消测试相关的模块编译。

​		修改`MAVSDK/src/CMakeLists.txt`文件，将`BUILD_TESTS`编译选项设置为OFF。

```cmake
option(BUILD_TESTS "Build tests" OFF)
```

​		这样就不会编译src/third_party/gtest、src/integration_tests、src/mavsdk/core/mocks文件夹的内容。

### 使用自定义的mavlink头文件

​		MAVSDK默认编译过程中，会下载mavlink，并且使用默认的common，如果开发者本地已经有mavlink不想重复下载，或者希望使用自定义的dialog，则可以通过设置MAVLINK_DIALECT和MAVLINK_HEADERS编译选项实现。

- MAVLINK_DIALECT：指定开发者自定义的dialog；
- MAVLINK_HEADERS：指定开发者本地的mavlink头文件目录；

​		需要开发者创建`mavlink-headers`文件夹，并将`mavlink-headers`文件夹放到与MAVSDK同级目录下，然后将生成好的mavlink头文件按照如下目录结构存放：

```shell
mavlink-headers              # <-- This is the directory referenced
└── mavlink
    └── v2.0
        ├── checksum.h
        ├── common/
        ├── standard/
        ├── minimal/
        ├── mydialect/       # 这是用户的自定义消息、命令等
        │   ├── mydialect.h
        │   ├── mavlink.h
        │   ├── mavlink_msg_...
```

​		运行编译命令：

```shell
$ cd MAVSDK
$ cmake -DCMAKE_INSTALL_PREFIX=install -Bbuild/default -DMAVLINK_DIALECT=nextpilot  -DMAVLINK_HEADERS=../mavlink-headers -H.
$ cmake --build build/default --target install
```

> 如果使用了自定义的mavlink头文件，那么就不需要再编译默认的mavlink模块（third_party/mavlink）。这样就节省了下载mavlink仓库、编译的步骤。

## 安装

### 安装在局部路径
```shell
$ cd MAVSDK
$ cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=install -Bbuild/default -H.
$ cmake --build build/default --target install
```

​		这样会在MAVSDK目录下生成install安装文件夹。

​		安装后，install目录结构如下：

```shell
.
├── include
│   └── mavsdk
│       └── plugins
│           ├── action
│           ├── log_files
│           ├── manual_control
│           ├── mavlink_passthrough
│           │   └── mavlink
│           │       └── v2.0
│           │           ├── common
│           │           ├── minimal
│           │           └── standard
│           ├── mission
│           ├── offboard
│           ├── param
│           ├── telemetry
└── lib
    └── cmake
    │   └── MAVSDK
    ├──libmavsdk.so
    ├──libmavsdk.so.1
    ├──libmavsdk.so.1.4.10
```

> 注意：这里没有将所有的plugins文件夹列出来。

### 安装在系统目录		

​		如果不指定安装目录，将会安装至系统默认安装目录，Linux系统默认目录是`/usr/local`。安装命令如下：

```shell
$ cd MAVSDK
$ sudo cmake --build build/default --target install
```



### 编译测试样例

​		使用mavsdk给的example进行测试，在example文件夹下面有多个测试样例。
​		当安装到局部路径下后，可以通过设置`CMAKE_PREFIX_PATH`为安装路径，这样就可以使用编译安装后的库和头文件了。

```shell
$ cd MAVSDK/examples
$ cmake -DCMAKE_PREFIX_PATH=$(pwd)/../install -Bbuild -H.
$ cmake --build build -j8
```

## 附录

### 生成mavlink头文件命令

```shell
$ cd mavlink-master
$ python3 -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 --output=generated/include/mavlink/v2.0 message_definitions/v1.0/common.xml
```

