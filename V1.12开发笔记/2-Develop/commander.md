# commander





## 各模块说明

### ArmAuthorization（解锁授权）



### HealthFlags（健康标志）

为了表示机载端有关子模块（如各传感器）状态，在vehicle_status.msg中定义了的三个成员：

```c
uint32 onboard_control_sensors_present
uint32 onboard_control_sensors_enabled
uint32 onboard_control_sensors_health
```

每个成员都是32位，每一位表示一个对应子模块，也就是最多可以记录32个子模块的状态。具体怎么对应由`HealthFlags.hpp`中`struct subsystem_info_s`结构体中定义的常量决定，例如：

```c
    static constexpr uint64_t SUBSYSTEM_TYPE_GYRO                  = 1 << 0;
    static constexpr uint64_t SUBSYSTEM_TYPE_ACC                   = 1 << 1;
    static constexpr uint64_t SUBSYSTEM_TYPE_MAG                   = 1 << 2;
	// 其他省略
```

也就是说位0表示陀螺仪状态，位1对应加速度计，位2对应磁力计等。

> 注意这里的子模块概念比较模糊，主要包括了各传感器如陀螺仪、加速度计，以及GEOFENCE、AHRS等。

根据mavlink消息SYS_STATUS ([ #1 ](https://mavlink.io/en/messages/common.html#SYS_STATUS))中的解释：

- present：表示对应子模块是否存在，判断条件一般为orb中是否有该模块对应的消息发布，例如如果陀螺仪存在，则会发布sensor_gyro_s类型消息。
- enabled：表示对应模块是否使能，对于各传感器来说，如果参与了估计器融合（作为了主传感器）则其是使能状态。
- health：表示对应模块是否工作正常。

#### 设置health flag

设置各模块的健康标志所在代码位置有多处：

- 在起飞前检查模块`PreFlightCheck`中，会对各传感器进行检查并设置health flag。
- 在commander主循环中调用检查，例如调用avoidance_check()设置避障相关传感器的health flag。



### PreFlightCheck起飞前检查

起飞前检查主要包括如下内容：

- 检查传感器如加速度计、陀螺仪、磁力计；
- 检查手动控制，manualControlCheck；
- 检查遥控器校准情况，rcCalibrationCheck；

commander主循环启动前进行一次起飞前检查，但不会打印任何检查信息。

commander主循环启动后，若没有解锁则会进行起飞检查，如果有检查失败则会打印相关信息。
