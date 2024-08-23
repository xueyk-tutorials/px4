

### 控制模式

无人机下发的heartbeat中包括了mode说明，含义为

- base_mode: 

  订阅ORB_ID(vehicle_control_mode)主题，根据orb消息内容，通过与运算，得到base mode。

- main_mode:

  订阅ORB_ID(vehicle_status)主题，根据orb消息nav_state成员变量进行判断，得到main_mode和sub_mode。nav_state取值可参考vehicle_status.h内的枚举定义。

- sub_mode: