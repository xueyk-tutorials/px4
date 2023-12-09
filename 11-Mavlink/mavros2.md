# mavros



## 软件框架

### 启动

整个ros的启动位于`mavros_node.cpp`中。

#### 添加节点

一共添加了三个ROS节点：rclcpp::Node、mavros::router::Router、mavros::uas::UAS。

- Router

  mavlink消息路由器，用于通信消息的路由。

- UAS

  UAS

#### 给Router添加Endpoint

​		通过指定ROS参数的方式，创建Endpoint，由`mavros_node.cpp`创建参数并调用`router_node->set_parameters()`函数发布节点参数设置事件，通过`Router::on_set_parameters_cb()`函数响应参数设置，读取参数并创建Endpoint，并将创建好的放到`Router::endpoints`“数组”中。

​		默认情况下，`mavros_node.cpp`设置的参数包括["fcu_urls", "gcs_urls", "uas_urls"]，也即会创建如下三个Endpoint：

- fcu

  用于飞控连接，通过udp/serial建立连接，属于MAVConnEndpoint类型

- gcs

  用于地面站连接，通过udp/serial建立连接，属于MAVConnEndpoint类型

- uas

  用于UAS连接，通过ROS消息通信机制与UAS节点建立连接，属于ROSEndpoint类型

#### MAVConnEndpoint接收处理

​		通过MAVConnEndpoint::recv_message()函数处理接收到的数据，那么该函数如何被调用的呢？

​		MAVConnEndpoint可以通过udp、serial等方式建立与外界的mavlink连接，也就是创建MAVConnUDP、MAVConnSerial类的实例，这两个类继承于MAVConnInterface，根据多态的性质，在MAVConnEndpoint类中定义了如下指针成员变量：

```c
mavconn::MAVConnInterface::Ptr link;
```

​		创建MAVConnInterface实例时，其open_url()函数用于创建mavlink通信连接，向该函数中传入两个回调函数：ReceivedCb、ClosedCb，其中ReceivedCb在连接接收到mavlink消息时调用。

​		在MAVConnEndpoint::open()函数中创建了MAVConnUDP或MAVConnSerial实例，并且向open_url()函数传入了MAVConnEndpoint::recv_message()函数指针，这样串口或UDP接收到数据后会自动调用endpoint的接收处理函数。

#### ROSEndpoint接收处理

​		通过ROSEndpoint::ros_recv_message()函数用于处理接收。

​		与MAVConnEndpoint不同，ROSEndpoint接收是通过订阅其他ROS节点实现的。在ROSEndpoint::open()函数中，创建了订阅"mavlink_sink"并绑定了ROSEndpoint::ros_recv_message()函数用于处理接收。

### Endpoint

​		endpoint用于建立与某个“外部终端”的连接。对应的连接终端可以是实体或者虚拟的，例如fcu endpoint与gcs endpoint对应的连接终端是实体，也就是对应了外部的一个设备，而uas endpoint对应的连接终端是虚拟的一个ROS节点。

​		fcu endpoint通过serial、UDP方式连接一个飞控，gcs endpoint通过serial、UDP方式连接一个地面站，也就是这两个endpoint需要操作串口或网口实现与具体设备的通信；而uas endpoint是通过ROS订阅与发布的机制与UAS节点进行通信（UAS节点在程序启动时创建）。

### Router

​		Router中定义了endpoints数组，包含了多个endpoints，每个endpoints用于与一个终端建立通信连接。

#### 转发

​		所谓转发即当前endpoint接收到mavlink消息后，获取并遍历Router中的endpoints数组，调用数组内的符合条件的endpoint的发送函数，将mavlink消息发送到endpoint的连接端。

​		**也就是一个endpoint的接收，通过Router规则后，触发其他endpoints的发送！**

​		例如uas endpoint接收到消息后，可以向fcu endpoint或者gcs  endpoint转发该消息（即调用这两个endpoint的发送函数），那么飞控或地面站就会收到该消息。

​		由Endpoint实例调用Router::route_message()函数实现。

​		**消息转发流程如下：**

FCU -> Router(fcu endpoint接收) -> Router(uas endpoint发送) -> UAS

UAS -> Router(uas endpoint接收) -> Router(fcu endpoint发送) -> FCU

#### 转发规则

​		首先获取消息目的地址(用sys_id与comp_id表示)，规则如下：

- 不向自身转发；
- 遍历endpoints数组，如果endpoint的地址与目的地址一致，则向该endpoint转发；
- 不向同类型的endpoint转发消息；

### UAS

​		UAS是一个ROS节点。

- 发送消息

  plugins node -> UAS -> Router(uas endpoint)。

  各插件接收到用户程序指令后，会调用UAS::send_message(const mavlink::Message & msg)，然后通过`this->sink`发布消息。

- 接收消息

  Router(uas endpoint) -> UAS -> plugins node。

  通过订阅"mavlink_source"地址上的消息并调用UAS::recv_message()接收函数处理，在接收处理中调用UAS::plugin_route()函数，轮询所有的插件。



### 用户程序

​		用户程序一般是由用户编写并创建的ROS节点。

#### 无人机通信流程

- 发送

  source: FCU -> Router -> UAS -> plugins node -> offboard Node

- 接收

  sink: offboard Node -> plugins node -> UAS -> Router -> FCU



## Plugin

所在文件`plugin.hpp`

### 类型

#### HandlerInfo

```c++
using HandlerInfo = std::tuple<mavlink::msgid_t, const char *, size_t, HandlerCb>;
```

#### Subscriptions

```c++
using Subscriptions = std::vector<HandlerInfo>;
```

## Plugin子类

包括IMUPlugin等



### plug导出

在每个plug，如imu、global_position等，其对应的.cpp末尾通过**MAVROS_PLUGIN_REGISTER**进行注册和导出，这样就能通过pluginlib相关函数，使用这些插件了。

例如在imu.cpp中，对**IMUPlugin**进行插件注册：

```c++
#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::IMUPlugin)
```

其中**MAVROS_PLUGIN_REGISTER**在mavros_plugin_register_macro.hpp中声明：

```c++
#define MAVROS_PLUGIN_REGISTER(PluginClass) \
  CLASS_LOADER_REGISTER_CLASS( \
    mavros::plugin::PluginFactoryTemplate<PluginClass>, \
    mavros::plugin::PluginFactory)
```



>根据**pluginlib源码**，其中CLASS_LOADER_REGISTER_CLASS与PLUGINLIB_EXPORT_CLASS等价！
>
>在`pluginlib/pluginlib/include/pluginlib/class_list_macros.hpp`有：
>
>```c++
>#include <class_loader/class_loader.hpp>
>
>
>#define PLUGINLIB_EXPORT_CLASS(class_type, base_class_type) \
>  CLASS_LOADER_REGISTER_CLASS(class_type, base_class_type)
>
>#endif  // PLUGINLIB__CLASS_LIST_MACROS_HPP_
>```
>
>





## UAS

### 成员变量（属性）

#### plugin_factory_loader

插件加载器！

```c++
pluginlib::ClassLoader<plugin::PluginFactory> plugin_factory_loader;
```

通过成员函数plugin_factory_loader.getDeclaredClasses()可以加载该ClassLoader对应基类下的所有插件！

#### plugin_subscriptions

存放msg_id对应的所有订阅信息！

```c++
//! UAS link -> router -> plugin handler
std::unordered_map<mavlink::msgid_t, plugin::Plugin::Subscriptions> plugin_subscriptions;
```

每个插件都会接收特定message（由msg_id定义），插件中包含了每个msg_id对应的处理函数，多个插件可能都会需要接收同一个message，也就是一个msg_id可能会对应很多个处理函数（这些处理函数都是属于不同的插件）。



### 成员函数（方法）

#### UAS::add_plugin

添加插件，将所有可用插件加载，并根据黑名单、白名单判断插件是否可用，并将可用插件的shared_ptr存放到loaded_plugins中。

#### connect_to_router

创建mavlink message的发送（this->sink发布者）与接收（this->source订阅者）。

##### 发布者sink

##### 订阅者source

话题名：/uas<tgt_sysstem>/mavlink_source

回调函数：UAS::recv_message

#### recv_message

mavlink消息接收回调函数。

订阅消息后，将订阅的`mavros_msgs::msg::Mavlink`转成`mavlink_msg_t`标准类型消息；

调用`UAS::plugin_route()`进行消息分发。

#### plugin_route

跟传入的message的msg_id，遍历并调用所有注册的插件中的回调函数，进而实现各插件对message的接收和处理！

```c++
void UAS::plugin_route(const mavlink_message_t * mmsg, const Framing framing)
{
  auto it = plugin_subscriptions.find(mmsg->msgid);
  if (it == plugin_subscriptions.end()) {
    return;
  }
  for (auto & info : it->second) {
    std::get<3>(info)(mmsg, framing);    // 调用插件回调函数，实现message接收和处理
  }
}
```





# C++相关

### 编译属性[[]]

[[maybe_unused]]用于描述暂时没有被使用的函数或变量，以避免编译器对此发出警告

```c++
[[maybe_unused]] int f()        //没有被使用的函数
{
	return 1;
}
int main()
{
	[[maybe_unused]] int i = 0; //没有被使用的变量
    return 0;
}
```

