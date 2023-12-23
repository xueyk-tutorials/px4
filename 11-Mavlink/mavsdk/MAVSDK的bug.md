# MAVSDK v1.4.10版本bug合集

## telemetry模块

### 姿态消息订阅<a id="姿态消息订阅">#Bug-01</a>

#### 描述

不论飞控没有发送attitude_quaternion消息或者发送频率是多少，MAVSDK订阅回调却一直以25Hz运行（约40ms/25Hz）。

#### 分析

源码中对ATTITUDE和ATTITUDE_QUATERNION两个消息进行了合并处理，由于欧拉角与四元数可以相互转换，任何一个消息收到后都会自动更新另一个消息。也就是如果飞控发送ATTITUDE频率为10，那么即使飞控不发送ATTITUDE_QUATERNION，mavsdk在收到ATTITUDE后更新四元数。这样会造成重复更新！











### 频率设置bug

#### 描述

set_rate_attitude_async()函数设置attitude_quaternion消息，飞控端能够正常响应，但MAVSDK订阅的消息频率仍然很高。

#### 分析

由于MAVSDK只提供了设置ATTITUDE_QUATERNION消息更新频率函数，没有提供更新ATTITUDE消息频率函数，这样只要有ATTITUDE消息过来仍然会更新ATTITUDE_QUATERNION消息。

参考[姿态消息订阅](#姿态消息订阅)









