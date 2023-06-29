## PX4使用

## QGC

Ref: https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html

### ubuntu

Before installing *QGroundControl* for the first time:

1. On the command prompt enter:

   ```sh
   sudo usermod -a -G dialout $USER
   sudo apt-get remove modemmanager -y
   sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
   ```

2. Logout and login again to enable the change to user permissions.

 To install *QGroundControl*:

1. Download [QGroundControl.AppImage](https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage).

2. Install (and run) using the terminal commands:

   ```sh
   chmod +x ./QGroundControl.AppImage
   ./QGroundControl.AppImage  (or double click)
   ```

### 飞行错误提示

#### 起飞前检查

commander模块执行多个传感器飞行前的质量检查和EKF检查，这些检查由COM _ARM<>参数控制。如果这些检查失败，电机将无法解锁，并产生以下错误信息

- PREFLIGHT FAIL: EKF HGT ERROR（高度错误）
  - 当IMU和高度检测数据不一致时,会产生此错误。
  - 校准加速度计和陀螺仪后重启飞行器, 如果这个错误任然存在,
     检查高度传感器数据的问题。
  - 此检查由参数COM_ARM_EKF_HGT控制。
- PREFLIGHT FAIL: EKF VEL ERROR（速度错误）
  - 当IMU和GPS速度检测数据不一致时,会产生此错误。
  - 检查GPS速度数据是否存在不切合实际的数据跳跃,如果GPS品质良好，执行加速机和陀螺仪校准，并重新启动飞行器。
  - 此检查由参数COM_ARM_EKF_VEL控制。
- PREFLIGHT FAIL: EKF HORIZ POS ERROR（水平位置错误）
  - 当IMU和位置测量数据（GPS或外部视觉设备）不一致时，会产生此错误。
  - 检查位置传感器数据是否存在不切实际的数据跳转。如果数据品质良好，执行加速和陀螺仪校准，并重新启动飞行器。
  - 此检查由参数COM_ARM_EKF_POS控制
- PREFLIGHT FAIL: EKF YAW ERROR（偏航角错误）
  - 当使用陀螺仪数据估计的偏航角和来自磁力计或外部视觉系统的偏航角不一致时，会产生此错误。
  - 检查IMU数据是否存在大的偏航速率偏移量，并检查磁力计调整和校准。
  - 此检查由参数COM_ARM_EKF_POS控制。
- PREFLIGHT FAIL: EKF HIGH IMU ACCEL BIAS（加速度计错误）
  - 当由EKF估计的IMU加速度计偏差过大时，会产生此错误。
  - 此检查由参数COM_ARM_EKF_AB控制。
- PREFLIGHT FAIL: EKF HIGH IMU GYRO BIAS（陀螺仪错误）
  - 当由EKF估计的IMU陀螺仪偏差过大时，会产生此错误。
  - 此检查由参数COM_ARM_EKF_GB控制
- PREFLIGHT FAIL: ACCEL SENSORS INCONSISTENT - CHECK CALIBRATION（加速度不一致）
  - 当来自不同IMU单元的加速度测量值不一致时，会产生此错误信息。
  - 此检查仅适用于具有多个IMU的电路板。
  - 此检查由参数COM_ARM_IMU_ACC控制。
- PREFLIGHT FAIL: GYRO SENSORS INCONSISTENT - CHECK CALIBRATION（角速度不一致）
  - 当来自不同IMU单元的角速率测量不一致时，会产生此错误信息。
  - 此检查仅适用于具有多个IMU的电路板。
  - 此检查由参数COM_ARM_IMU_GYR 控制。

## PX4固件烧写

### 通过QGC烧写

QGC会下载最新的稳定版本固件，下载地址例如：http://px4-travis.s3.amazonaws.com/Firmware/stable/px4fmu-v5_default.px4

## PX4参数

使用QGC获取PX4参数列表，重要的参数解释如下表：

| QGC分组                   | 参数             | 意义                                                         |
| ------------------------- | ---------------- | ------------------------------------------------------------ |
| MAV                       | MAV_SYS_ID       | 无人机的system ID（即对应mavlink消息中的system ID），地面站将根据system ID区分多机通信 |
|                           | MAV_COMP_ID      | 无人机的component ID                                         |
| MPC                       | MC_ROLL_P        | 俯仰角外环PID参数P                                           |
|                           | MC_ROLLRATE_P    | 俯仰角内环PID参数P                                           |
|                           | MC_ROLLRATE_I    | 俯仰角内环PID参数I                                           |
|                           | MC_ROLLRATE_D    | 俯仰角内环PID参数D                                           |
|                           | MC_PITCH_P       | 横滚角外环PID参数P                                           |
|                           | MC_PITCHRATE_P   | 横滚角内环PID参数P                                           |
|                           | MC_PITCHRATE_I   | 横滚角内环PID参数I                                           |
|                           | MC_PITCHRATE_D   | 横滚角内环PID参数D                                           |
|                           | MC_YAW_P         | 偏航角外环PID参数P                                           |
|                           | MC_YAWRATE_P     | 偏航角内环PID参数P                                           |
|                           | MC_YAWRATE_I     | 偏航角内环PID参数I                                           |
|                           | MC_YAWRATE_D     | 偏航角内环PID参数D                                           |
|                           | MC_ROLLRATE_MAX  | 限制除acro模式下的最大roll角速度率                           |
|                           | MPC_TILTMAX_AIR  | 在空中无人机最大倾角                                         |
|                           | MPC_XY_CRUISE    | 设置AUTO模式（RTL/HOLD/MISSION）下水平速度                   |
|                           | MPC_XY_VEL_MAX   | 设置无人机水平飞行的最大速度，注意这个速度会作为MPC_XY_CRUISE的上限，如果MPC_XY_VEL_MAX<MPC_XY_CRUISE，那么会自动的将MPC_XY_CRUISE设置为MPC_XY_VEL_MAX |
|                           | MPC_Z_VEL_MAX_DN | AUTO模式和stablized模式(ALT/POS CTRL)下最大下降速度          |
|                           | MPC_Z_VEL_MAX_UP | AUTO模式和stablized模式(ALT/POS CTRL)下最大上升速度          |
|                           | MPC_XY_P         | 水平位置PID控制P参数                                         |
|                           | BRD_SAFETYENABLE | 0:禁用安全开关 1:启用安全开关                                |
| RTL                       | RTL_RETURN_ALT   |                                                              |
| COM                       | COM_ARM_EKF_AB   |                                                              |
|                           | COM_ARM_SWISBTN  |                                                              |
|                           | COM_ARM_MAG_ANG  | 起飞ARM检查磁场角度的一致性所允许的最大偏差角度，如果角度设置太小，很容易解锁时提示磁场需要校准 |
|                           | COM_RC_IN_MODE   | 1：启动使用虚拟摇杆(virtual joystick/thumb joystick)         |
| Developer:Circuit Breaker | CBRK_IO_SAFETY   | 0：启用安全开关<br>22027：禁用安全开关                       |
|                           | MAV_0_CONFIG     | MAVLink实例0的串口配置，这个默认配置到TELEM1                 |
|                           | MAV_1_CONFIG     | MAVLink实例1的串口配置，可以用来配置TELEM2                   |
|                           | SER_TEL1_BAUD    | telem1串口波特率设置                                         |
| RC                        | RC_MAP_AUX1      | PX4飞控A1通道映射，选择一个RC通道即可自动映射至飞控A1通道输出 |
|                           | CAL_MAG0_EN      |                                                              |
|                           | CAL_MAG_PRIME    | 设置外部或内部某个磁罗盘为主磁罗盘，如果主磁罗盘没有检测到则无法解锁。若外部磁罗盘没有连接，则应将主磁罗盘ID设置为内部对应的ID值。 |
|                           | CAL_MAG0_ID      | 内部磁罗盘ID                                                 |
|                           |                  | 卡曼滤波器使用光流数据 EK2_GPS_TYPE = 3 # 使用声呐作为主要高度数据来源 EK2_ALT_SOURCE = 1 |
| CBRK                      | CBRK_SUPPLY_CHK  | Pixhawk不使用Power Maneger也可以解锁使用，将其设置为894281即可在无电流计情况下解锁 |



### onboard配置

Ref:https://dev.px4.io/v1.10/zh/companion_computer/pixhawk_companion.html

要在`TELEM 2` 上配置默认的配套计算机消息流，请设置以下参数：

- [MAV_1_CONFIG](https://dev.px4.io/v1.10/zh/advanced/parameter_reference.html#MAV_1_CONFIG) = `TELEM 2` (`MAV_1_CONFIG`总是配置为 `TELEM 2` 端口)
- [MAV_1_MODE](https://dev.px4.io/v1.10/zh/advanced/parameter_reference.html#MAV_1_MODE) = `Onboard`
- [SER_TEL2_BAUD](https://dev.px4.io/v1.10/zh/advanced/parameter_reference.html#SER_TEL2_BAUD) = `921600`（建议在像日志流或Fast RTPS之类的应用，使用 921600 或更高）



## 飞控MAVLink消息

连接地面站，可以看到飞控的外发消息包括如下：

| ALTITUDE           | GLOBAL_POSITION_INT | PARAM_VALUE         |
| ------------------ | ------------------- | ------------------- |
| ATTITUDE           | HEARTBEAT           | PROTOCOL_VERSION    |
| ATTITUDE_TARGET    | HOME_POSITION       | RC_CHANNELS         |
| AUTOPILOT_VERSION  | LOCAL_POSITION_NED  | SERVO_OUTPUT_RAW    |
| BATTERY_STATUS     | MISSION_COUNT       | STATUSTEXT          |
| COMMAND_ACK        | MISSION_CURRENT     | SYS_STATUS          |
| ESTIMATOR_STATUS   | MISSION_ITEM_INT    | UTM_GLOBAL_POSITION |
| EXTENDED_SYS_STATE | PING                | VFR_HUD             |
| GPS_RAW_INT        |                     | VIBRATION           |



## 虚拟摇杆

要在PX4中启用游戏杆支持，您需要将参数设置COM_RC_IN_MODE为1 - 游戏杆/无RC校验。如果此参数未设置，则游戏杆不会作为设置选项提供。

对于PX4 SITL构建，这是默认启用的。

### 参考

http://www.yoyojacky.com/?p=137

https://docs.qgroundcontrol.com/master/en/SettingsView/VirtualJoystick.html



### 配置游戏杆

1. 关闭QGroundControl
2. 将游戏杆或游戏手柄连接到USB端口
3. 启动QGroundControl并连接到车辆
4. 选择顶部工具栏中的齿轮图标（车辆设置），然后选择边栏中的操纵杆。下面的屏幕将会出现。游戏杆设置 PlayStation
5. 按校准按钮，然后按照屏幕上的说明校准/移动棒。
6. 选中启用游戏杆输入复选框
7. 确保您的游戏杆在主动游戏杆下拉菜单中被选中。
8. 选择每个操纵杆按钮激活的飞行模式/车辆功能。最多可以设置16个操纵杆按钮操作。
9. 通过按下按钮测试按钮和粘贴杆，并在轴/按钮监视器中查看结果。

### 支持的摇杆

索尼Playstation 3/4控制器

罗技极限3D专业版

罗技F710游戏手柄



```python
import os, struct, array
from fcntl import ioctl
'''
Event reading:
struct js_event{
    __u32 time,    # event timestamp in milliseconds
    __s16 value,   # value [-32768 to +32767]
    __u8  type,    # button:0x01, axis:0x02, initial state:0x80
    __u8  number   # axis or button number
    }
'''

JSIOCGAXES = 0x80016a11
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS   = 0x02
JS_EVENT_INIT   = 0x08

class Joysticker(object):
    def __init__(self):
        fn_js = None
        for fn in os.listdir('/dev/input'):
            if fn.startswith('js'):
                print('/dev/input/%s' % fn)
                fn_js = '/dev/input/' + fn
    self.jsdev = open(fn_js,'rb')

    buf = array.array('B',[0])
    ioctl(self.jsdev,JSIOCGAXES,buf)
    print(buf)
    num_axis = buf[0]

    buf = array.array('B',[0]*num_axis)
    ioctl(self.jsdev,0x80406a32,buf)
    print(buf)

def read(self):
    info = None
    evbuf = self.jsdev.read(8)
    if evbuf:
        time,value,ttype,number = struct.unpack('IhBB',evbuf)
        info = [time,value,ttype,number]
        #print('time:{0},value:{1},ttype:{2},number:{3}'.format(time,value,ttype,number))
    else:
        #print('nothing')
        info = None
    return info
    
if __name__ == '__main__':
    js = Joysticker()
    while True:
        info = js.read()
        if info is not None:
            print(info)

```


​        



