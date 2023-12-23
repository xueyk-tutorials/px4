# MAVSDK（五）源码解析2

## 如何超时检测

调用插件函数，添加要执行的“工作”到工作列表；

周期性调用插件do_work()，遍历工作列表，并为工作注册超时；

如果未超时前收到“响应”，则注销对应的超时注册cookie；



周期性判断time_out列表内注册的超时事件，如果时间已经超时则调用响应的回调函数。



### 示例：heartbeat超时检测



## 如何获取飞控数据

用户调用`subscribe_position()`函数设置回调函数，当位置数据更新后会自动调用用户的函数。

```c++
void TelemetryImpl::subscribe_position(Telemetry::PositionCallback& callback)
{
    std::lock_guard<std::mutex> lock(_subscription_mutex);
    _position_subscription = callback;
}
```

这里设置用户回调函数，只需要将用户定义的回调函数赋值给`_position_subscription`。

TelemetryImpl类提供了mavlink消息处理函数，例如`process_global_position_int()`函数处理位置消息，这里将`_position_subscription`回调（也就是用户的回调函数）添加至`MavsdkImpl::_user_callback_queue`用户回调队列中。

```c++
void TelemetryImpl::process_global_position_int(const mavlink_message_t& message) {
    if (_position_subscription) {
        auto callback = _position_subscription;
        auto arg = position();
        _parent->call_user_callback([callback, arg]() { callback(arg); });
    }
}

```

在`TelemetryImpl`实例初始化函数中，将注册mavlink消息（GLOBAL_POSITION_INT）处理函数，一旦接收到GLOBAL_POSITION_INT消息后，将调用`process_global_position_int()`函数。

```c++
void TelemetryImpl::init(){
	_parent->register_mavlink_message_handler(
        MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        [this](const mavlink_message_t& message) { process_global_position_int(message); },
        this); 
}
```



## 如何获取参数

MAVLinkParameters类实现参数操作，get_param_int_async()函数用于获取参数。

### 基本流程

​		调用MAVLinkParameters::get_param_int_async()函数，传入要获取的参数名和完成操作后执行的回调函数；

​		MAVLinkParameters::get_param_int_async()将创建获取参数的工作项（WorkItem）；

​		MAVLinkParameters::do_work()处理工作项，并注册超时处理函数；

​		MAVLinkParameters::receive_timeout()处理超时；

> 其中：
>
> SystemImpl::system_thread()会调用MAVLinkParameters::do_work()；
>
> 使用SystemImpl::register_timeout_handler()注册超时处理函数，将超时处理添加到MavsdkImpl::timeout_handler；
>
> MavsdkImpl::work_thread()会周期性调用timeout_handler.run_once()处理超时；
>
> 由`NPsdk::DEFAULT_TIMEOUT_S`定义默认超时时间；

### 代码说明

```c++
void MAVLinkParameters::get_param_int_async(
    const std::string& name,
    const GetParamIntCallback& callback,
    const void* cookie,
    std::optional<uint8_t> maybe_component_id,
    bool extended)
{
    // LogDebug() << "getting param " << name << ", extended: " << (extended ? "yes" : "no");

    if (name.size() > PARAM_ID_LEN) {
        LogErr() << "Error: param name too long";
        if (callback) {
            callback(MAVLinkParameters::Result::ParamNameTooLong, 0);
        }
        return;
    }

    // Otherwise, push work onto queue.
    auto new_work = std::make_shared<WorkItem>(_parent.timeout_s());
    new_work->type = WorkItem::Type::Get;
    new_work->callback = callback;
    new_work->maybe_component_id = maybe_component_id;
    new_work->param_name = name;
    new_work->param_value = {};
    new_work->extended = extended;
    new_work->cookie = cookie;

    _work_queue.push_back(new_work);
}
```



## 时间处理

npsdk_time.h

```c++
typedef std::chrono::time_point<std::chrono::steady_clock> dl_time_t;
typedef std::chrono::time_point<std::chrono::system_clock> dl_system_time_t;
typedef std::chrono::time_point<std::chrono::system_clock> dl_autopilot_time_t;
```



## 如何发送mavlink command

### 代码分析

#### 发送命令

SystemImpl::send_command()

MavlinkCommandSender::send_command()



```c++
MavlinkCommandSender::Result SystemImpl::send_command(MavlinkCommandSender::CommandInt& command)
{
    if (_target_address.system_id == 0 && _components.empty()) {
        return MavlinkCommandSender::Result::NoSystem;
    }
    command.target_system_id = get_system_id();
    return _command_sender.send_command(command);
}
```



#### 调用回调函数

等待飞控反馈控制命令后，会发送ACK消息。

SystemImpl::process_mavlink_message

MAVLinkMessageHandler.process_message



### 示例：设置消息更新频率(async方式)

Telemetry::set_rate_position_async()函数设置频率和回调函数。

TelemetryImpl::set_rate_position_async()函数设置频率和回调函数。

SystemImpl::set_msg_rate_async()函数设置消息id、频率、回调函数、comp_id。

> - 首先生成控制指令
>
>   生成MavlinkCommandSender::CommandLong类型数据
>
> - 发送控制指令并注册回调
>
>   调用SystemImpl::send_command_async()
>
>   调用MavlinkCommandSender::queue_command_async()
>
> - 生成工作事项
>
>   生成MavlinkCommandSender::Work类型的工作事项并放入工作队列
>
> - 工作
>
>   在MavlinkCommandSender::do_work()中遍历工作队列，一项项进行处理



