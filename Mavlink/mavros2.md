# mavros



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

