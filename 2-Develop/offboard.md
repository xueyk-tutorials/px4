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





## 备注



### 相关mavlink消息

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



