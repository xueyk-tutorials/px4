# 使用wireshark进行mavlink调试

## 配置

### 安装wireshark

```shell
sudo apt-add-repository ppa:wireshark-dev/stable
sudo apt-get install wireshark

## 添加到用户组
sudo adduser $USER wireshark
```

输入命令启动

```shell
wireshark
```



### 插件生成与配置

#### 生成插件

下载mavlink仓库，进入mavlink-master根目录，输入如下命令：

```shell
python3 -m pymavlink.tools.mavgen --lang=WLua --wire-protocol=2.0 --output=mavlink_2_common message_definitions/v1.0/common.xml
```

插件**mavlink_2_common.lua**.会在当前目录下创建

#### 配置插件

为了监控mavlink消息，必须指定监控端口，在**mavlink_2_common.lua**文件默认添加了相关配置信息，如下是默认的UDP监控端口

```lua
-- bind protocol dissector to USER0 linktype

wtap_encap = DissectorTable.get("wtap_encap")
wtap_encap:add(wtap.USER0, mavlink_proto)

-- bind protocol dissector to port 14550 and 14580

local udp_dissector_table = DissectorTable.get("udp.port")
udp_dissector_table:add(14550, mavlink_proto)
udp_dissector_table:add(14580, mavlink_proto)
```

仿真环境下，GCS连接端口为：

18570<---->14550

### 导入插件

命令行输入`wireshark`打开软件，在Help->Folders，查看Global Lua Plugins的路径，并将生成的插件**mavlink_2_common.lua**文件拷贝至该路径下，例如：

```shell
$ sudo cp mavlink_2_common.lua /usr/lib/x86_64-linux-gnu/wireshark/plugins/
```

重启wireshark软件。





## 使用

### 过滤

- 只查看地面站发送的消息

地面站的comp_id=0xbe=190

```shell
mavlink_proto.msgid==0
```





```shell
mavlink_proto.msgid==0 && mavlink_proto.compid == 1 && 
(ip.addr == 10.0.115.155 && ip.dst == 10.0.115.141)
```



