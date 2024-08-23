# 源码编译问题记录

## Ubuntu编译

### 编译

#### 内存不足

- 描述

  在WSL2 ubuntu20.04下编译，报错有`c++: fatal error: Killed signal terminated program cc1plus`。

- 解决

  创建交换分区

```shell
# 创建分区路径
sudo mkdir -p /var/cache/swap/
# 设置分区的大小
# bs=64M是块大小，count=64是块数量，所以swap空间大小是bs*count=4096MB=4GB
sudo dd if=/dev/zero of=/var/cache/swap/swap0 bs=64M count=64
# 设置该目录权限
sudo chmod 0600 /var/cache/swap/swap0
# 创建SWAP文件
sudo mkswap /var/cache/swap/swap0
# 激活SWAP文件
sudo swapon /var/cache/swap/swap0
# 查看SWAP信息是否正确
sudo swapon -s
```

如果编译完了，不想要交换分区可以删除，命令为：

```shell
sudo swapoff /var/cache/swap/swap0
sudo rm /var/cache/swap/swap0
```



### gazebo相关

1.  gz/common/profiler/Export.hh: No such file or directory

   问题：

   ubuntu20.04+gazebo11，编译PX4仿真时报错`gz/common/profiler/Export.hh: No such file or directory`

   解决：

```shell
$sudo apt install libignition-common3-dev
```

