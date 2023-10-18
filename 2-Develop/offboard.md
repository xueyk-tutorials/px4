# offboard

## 逻辑流程

### onboard计算机到PX4 mavlink模块

要发送的消息主要有：

- MAV_CMD_DO_SET_MODE ([176](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE) )

  设置模式，用于开启offboard飞行模式。

- SET_POSITION_TARGET_LOCAL_NED ([ #84 ](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED))

  设置local或body下的期望位置、速度、加速度、偏航角等。

- SET_POSITION_TARGET_GLOBAL_INT ([ #86 ](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT))

  设置global下的期望位置、速度、加速度、偏航角等。



### mavlink模块

mavlink接收线程，对接收的期望SET_POSITION_TARGET_xxx消息进行解析，如果是GLOBAL期望则会根据无人机当前位置转成LOCAL期望，然后计算出对应的vehicle_local_position_setpoint_s和offboard_control_mode_s消息，最后通过如下主题发布：

- ORB_ID(offboard_control_mode)发布offboard_control_mode_s消息。

- ORB_ID(trajectory_setpoint)发布vehicle_local_position_setpoint_s消息。

### 各控制模块

订阅mavlink模块发布的期望，做为控制器输入。

## 常用功能控制

### 进入offboard模式

机载计算机通过发送MAV_CMD_DO_SET_MODE消息至飞控，实现offboard切换。

```c
// base_mode:
129
// custom_mode:
	MANUAL = 0x10000
    ALTITUDE = 0x20000
    POSITION = 0x30000
    OFFBOARD = 0x60000
    STABILIZED = 0x70000
    TAKEOFF = 0x2040000
    HOLD = 0x3040000
    RETURN = 0x5040000
    LAND = 0x6040000
```

### VTOL模态切换

机载计算机通过发送MAV_CMD_DO_VTOL_TRANSITION消息至飞控，切换飞行模态为固定翼或多旋翼。

发布ORB消息时将vehicle_command_s.command置为vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION即可实现切换。

vtol_att_control模块通过订阅ORB_ID(vehicle_command)实现飞行模态切换。

### 改变无人机位置

机载计算机通过发送MAV_CMD_DO_REPOSITION消息至飞控，固定翼飞行至该位置后会盘旋，多旋翼飞行至该位置后会悬停。

### 查看状态

#### 垂起相关

vehicle_status_s.is_vtol：是否为垂起机型

vehicle_status_s.vehicle_type：成员表示当前飞行模态

- vehicle_status_s::VEHICLE_TYPE_ROTARY_WING = 1;

- vehicle_status_s::VEHICLE_TYPE_FIXED_WING = 2;

vtol_vehicle_status_s.vtol_in_trans_mode：判断是否处于模态转换阶段

## ORB



## 备注



### 相关mavlink消息

#### MAV_CMD_DO_SET_MODE ([176](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE) )



#### MAV_CMD_DO_VTOL_TRANSITION ([3000](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_VTOL_TRANSITION) )

控制命令：请求飞行模态切换

| Param (:Label) | Description                                                  | Values                                                       |
| -------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 1: State       | The target VTOL state. For normal transitions, only [MAV_VTOL_STATE_MC](https://mavlink.io/en/messages/common.html#MAV_VTOL_STATE_MC) and [MAV_VTOL_STATE_FW](https://mavlink.io/en/messages/common.html#MAV_VTOL_STATE_FW) can be used. | [MAV_VTOL_STATE](https://mavlink.io/en/messages/common.html#MAV_VTOL_STATE) |
| 2: Immediate   | Force immediate transition to the specified [MAV_VTOL_STATE](https://mavlink.io/en/messages/common.html#MAV_VTOL_STATE). 1: Force immediate, 0: normal transition. Can be used, for example, to trigger an emergency "Quadchute". Caution: Can be dangerous/damage vehicle, depending on autopilot implementation of this command. |                                                              |

#### MAV_CMD_DO_REPOSITION ([192](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_REPOSITION) )

控制命令：改变无人机位置，使无人机以一定速度和航向飞向一个指定的WGS84 global position。

| Param (:Label) | Description                                                  | Values                                                       | Units |
| -------------- | ------------------------------------------------------------ | ------------------------------------------------------------ | ----- |
| 1: Speed       | Ground speed, less than 0 (-1) for default                   | *min:* -1                                                    | m/s   |
| 2: Bitmask     | Bitmask of option flags.                                     | [MAV_DO_REPOSITION_FLAGS](https://mavlink.io/en/messages/common.html#MAV_DO_REPOSITION_FLAGS) |       |
| 3: Radius      | Loiter radius for planes. Positive values only, direction is controlled by Yaw value. A value of zero or NaN is ignored. |                                                              | m     |
| 4: Yaw         | Yaw heading. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.). For planes indicates loiter direction (0: clockwise, 1: counter clockwise) |                                                              | deg   |
| 5: Latitude    | Latitude                                                     |                                                              |       |
| 6: Longitude   | Longitude                                                    |                                                              |       |
| 7: Altitude    | Altitude                                                     |                                                              | m     |

#### SET_GPS_GLOBAL_ORIGIN ([ #48 ](https://mavlink.io/en/messages/common.html#SET_GPS_GLOBAL_ORIGIN))

设置无人机局部坐标系原点对应的经纬度坐标。

| Field Name                                                   | Type     | Units | Description                                                  |
| ------------------------------------------------------------ | -------- | ----- | ------------------------------------------------------------ |
| target_system                                                | uint8_t  |       | System ID                                                    |
| latitude                                                     | int32_t  | degE7 | Latitude (WGS84)                                             |
| longitude                                                    | int32_t  | degE7 | Longitude (WGS84)                                            |
| altitude                                                     | int32_t  | mm    | Altitude (MSL). Positive for up.                             |
| time_usec[ **](https://mavlink.io/en/messages/common.html#mav2_extension_field) | uint64_t | us    | Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number. |

#### SET_POSITION_TARGET_LOCAL_NED ([ #84 ](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED))

| Field Name       | Type     | Units | Values                                                       | Description                                                  |
| ---------------- | -------- | ----- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| time_boot_ms     | uint32_t | ms    |                                                              | Timestamp (time since system boot).                          |
| target_system    | uint8_t  |       |                                                              | System ID                                                    |
| target_component | uint8_t  |       |                                                              | Component ID                                                 |
| coordinate_frame | uint8_t  |       | [MAV_FRAME](https://mavlink.io/en/messages/common.html#MAV_FRAME) | Valid options are: [MAV_FRAME_LOCAL_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_LOCAL_NED) = 1, [MAV_FRAME_LOCAL_OFFSET_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_LOCAL_OFFSET_NED) = 7, [MAV_FRAME_BODY_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_NED) = 8, [MAV_FRAME_BODY_OFFSET_NED](https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_OFFSET_NED) = 9 |
| type_mask        | uint16_t |       | [POSITION_TARGET_TYPEMASK](https://mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK) | Bitmap to indicate which dimensions should be ignored by the vehicle. |
| x                | float    | m     |                                                              | X Position in NED frame                                      |
| y                | float    | m     |                                                              | Y Position in NED frame                                      |
| z                | float    | m     |                                                              | Z Position in NED frame (note, altitude is negative in NED)  |
| vx               | float    | m/s   |                                                              | X velocity in NED frame                                      |
| vy               | float    | m/s   |                                                              | Y velocity in NED frame                                      |
| vz               | float    | m/s   |                                                              | Z velocity in NED frame                                      |
| afx              | float    | m/s/s |                                                              | X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N |
| afy              | float    | m/s/s |                                                              | Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N |
| afz              | float    | m/s/s |                                                              | Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N |
| yaw              | float    | rad   |                                                              | yaw setpoint                                                 |
| yaw_rate         | float    | rad/s |                                                              | yaw rate setpoint                                            |

#### SET_POSITION_TARGET_GLOBAL_INT ([ #86 ](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT))

| Field Name       | Type     | Units | Values                                                       | Description                                                  |
| ---------------- | -------- | ----- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| time_boot_ms     | uint32_t | ms    |                                                              | Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency. |
| target_system    | uint8_t  |       |                                                              | System ID                                                    |
| target_component | uint8_t  |       |                                                              | Component ID                                                 |
| coordinate_frame | uint8_t  |       | [MAV_FRAME](https://mavlink.io/en/messages/common.html#MAV_FRAME) | Valid options are: [MAV_FRAME_GLOBAL_INT](https://mavlink.io/en/messages/common.html#MAV_FRAME_GLOBAL_INT) = 5, [MAV_FRAME_GLOBAL_RELATIVE_ALT_INT](https://mavlink.io/en/messages/common.html#MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) = 6, [MAV_FRAME_GLOBAL_TERRAIN_ALT_INT](https://mavlink.io/en/messages/common.html#MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) = 11 |
| type_mask        | uint16_t |       | [POSITION_TARGET_TYPEMASK](https://mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK) | Bitmap to indicate which dimensions should be ignored by the vehicle. |
| lat_int          | int32_t  | degE7 |                                                              | X Position in WGS84 frame                                    |
| lon_int          | int32_t  | degE7 |                                                              | Y Position in WGS84 frame                                    |
| alt              | float    | m     |                                                              | Altitude (MSL, Relative to home, or AGL - depending on frame) |
| vx               | float    | m/s   |                                                              | X velocity in NED frame                                      |
| vy               | float    | m/s   |                                                              | Y velocity in NED frame                                      |
| vz               | float    | m/s   |                                                              | Z velocity in NED frame                                      |
| afx              | float    | m/s/s |                                                              | X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N |
| afy              | float    | m/s/s |                                                              | Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N |
| afz              | float    | m/s/s |                                                              | Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N |
| yaw              | float    | rad   |                                                              | yaw setpoint                                                 |
| yaw_rate         | float    | rad/s |                                                              | yaw rate setpoint                                            |



### 相关orb消息

#### offboard_control_mode.msg

```shell
# Off-board control mode

uint64 timestamp		# time since system start (microseconds)

bool position
bool velocity
bool acceleration
bool attitude
bool body_rate
```





#### vehicle_local_position_setpoint.msg

```shell
# Local position setpoint in NED frame
# setting something to NaN means the state should not be controlled

uint64 timestamp	# time since system start (microseconds)

float32 x		# in meters NED
float32 y		# in meters NED
float32 z		# in meters NED
float32 yaw		# in radians NED -PI..+PI
float32 yawspeed	# in radians/sec
float32 vx		# in meters/sec
float32 vy		# in meters/sec
float32 vz		# in meters/sec
float32[3] acceleration # in meters/sec^2
float32[3] jerk # in meters/sec^3
float32[3] thrust	# normalized thrust vector in NED

# TOPICS vehicle_local_position_setpoint trajectory_setpoint

```



