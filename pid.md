# PID调参

参考：

https://docs.px4.io/main/zh/config_mc/pid_tuning_guide_multicopter_basic.html

https://logs.px4.io

## 参数基本意义

### P参数

P参数影响响应速度

P过大：超调或发散，产生高频震荡

P过小：响应慢，控制感不足

### I参数

I参数可以消除稳态误差

I过大：产生低频震荡

I过小：需要分析log

### D参数

D参数可以减小超调量

D过大：产生高频抖动

D过小：产生衰减振荡，需要分析log

## PX4控制器

https://docs.px4.io/main/zh/flight_stack/controller_diagrams.html

## 姿态环

### 参数

| 说明                   | 参数           | 描述 |
| ---------------------- | -------------- | ---- |
| 内环Roll rate control  | MC_ROLLRATE_P  |      |
|                        | MC_ROLLRATE_I  |      |
|                        | MC_ROLLRATE_D  |      |
| 内环Pitch rate control | MC_PITCHRATE_P |      |
|                        | MC_PITCHRATE_I |      |
|                        | MC_PITCHRATE_D |      |
| 内环Yaw rate control   | MC_YAWRATE_P   |      |
|                        | MC_YAWRATE_I   |      |
|                        | MC_YAWRATE_D   |      |
| 外环Roll control       | MC_ROLL_P      |      |
| 外环Pitch control      | MC_PITCH_P     |      |
| 外环Yaw control        | MC_YAW_P       |      |



## 位置环

MPC_XY_VEL_P_ACC

## 调参

先调内环再调外环

内环为姿态环，外环为位置环

P不断增加，发生震荡后慢慢增多D，直到消除震荡，如果D过大就又会导致震荡。
