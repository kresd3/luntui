# E4_05_imu660rc_demo 代码阅读文档

本文档用于快速理解 `E4_05_imu660rc_demo` 的整体框架和 `code` 目录下各模块逻辑。当前例程已经不只是 IMU660RC 数据显示，而是围绕 IMU 姿态、电调速度反馈、轮腿机构逆运动学和 TFT 菜单调参构成的轮腿平衡控制程序雏形。

## 1. 工程入口与运行核心

主要运行在 `user/main_cm7_0.c`，`user/main_cm7_1.c` 基本保持空模板。

`main_cm7_0.c` 的初始化顺序：

1. `clock_init(SYSTEM_CLOCK_250M)`：系统时钟和底层初始化。
2. `debug_init()`：调试串口初始化。
3. `tft180_init()` 相关配置：初始化 TFT180 屏幕，用于菜单显示。
4. `gpio_init(KEY1..KEY4, ...)`：初始化 4 个按键，上拉输入。
5. `leg_control_init()`：初始化四个腿部舵机 PWM，并设置初始腿部坐标。
6. `small_driver_uart_init()`：初始化 UART1，与无刷电调/驱动板通信。
7. 初始化各个 PID 参数：
   - `PID_leg`
   - `PID_out`
   - `PID_L`
   - `PID_R`
   - `PID_balance`
   - `PID_gyro`
   - `PID_dir`
   - `PID_pitch`
   - `PID_roll`
8. `imu660rc_init(IMU660RC_QUARTERNION_120HZ)`：IMU660RC 以 120Hz 四元数模式工作。
9. 初始化三个 PIT 周期中断：
   - `PIT_CH0`：10ms
   - `PIT_CH1`：5ms
   - `PIT_CH2`：2ms

主循环只做一件事：

```c
while(true)
{
    menu_Display();
}
```

也就是说，高频控制闭环主要放在 PIT 中断里，主循环负责屏幕菜单交互。

## 2. 中断调度框架

核心控制逻辑在 `user/cm7_0_isr.c`。

### 2.1 IMU 数据采集

`gpio_6_exti_isr()` 中判断 `P06_7` 外部中断：

```c
if(exti_flag_get(P06_7))
{
    imu660rc_callback();
}
```

`P06_7` 对应 `IMU660RC_INT2_PIN`。IMU 产生数据就绪中断后，回调读取四元数，并更新：

- `imu660rc_roll`
- `imu660rc_pitch`
- `imu660rc_yaw`
- `imu660rc_gyro_x`
- `imu660rc_gyro_y`
- `imu660rc_gyro_z`
- `imu660rc_acc_x/y/z`

因此主循环中不再主动调用 `imu660rc_get_acc()` 或 `imu660rc_get_gyro()`。

### 2.2 UART1 电调数据接收

`uart1_isr()` 中：

```c
uart_control_callback();
Lowpass();
```

作用：

- `uart_control_callback()` 从 UART1 解析驱动板返回的左右轮速度。
- `Lowpass()` 对速度做一阶低通滤波，得到 `Lowpass_left_speed_data` 和 `Lowpass_right_speed_data`。

注意：当前 PIT 控制里计算 `car_speed` 使用的是 `motor_value.receive_left_speed_data` 和 `motor_value.receive_right_speed_data` 原始速度，而不是低通后的速度。

### 2.3 PIT_CH0：10ms 外环/按键/方向

```c
Get_KeyNum();
car_speed = (float)(motor_value.receive_left_speed_data - motor_value.receive_right_speed_data) / 2.0;
B_X = 18.48 + 1.6 * PosionPID_realize(&PID_leg, car_speed);
PosionPID_realize(&PID_dir, imu660rc_gyro_z);
```

职责：

- 扫描按键，更新 `Num_Key`。
- 根据左右轮速度差计算 `car_speed`。
- `PID_leg` 以速度为反馈，输出腿部 X 坐标修正量，改变 `B_X`。
- `PID_dir` 以 `imu660rc_gyro_z` 为反馈，输出左右轮差速修正。

### 2.4 PIT_CH1：5ms 平衡角速度目标

```c
PID_gyro.target_val = PosionPID_realize(&PID_balance, imu660rc_pitch);
```

职责：

- `PID_balance` 以车身 pitch 角为反馈。
- 输出作为 `PID_gyro.target_val`，也就是期望俯仰角速度。

这相当于平衡控制的角度环：姿态角误差 -> 目标角速度。

### 2.5 PIT_CH2：2ms 内环/腿部逆解/电机输出

```c
small_driver_get_speed();
PID_R.target_val = PosionPID_realize(&PID_gyro, -imu660rc_gyro_y);
leg_position_set(B_X, B_H + E_H, B_H - E_H);
inverseKinematics();
motor_Loop();
```

职责：

- 向驱动板请求速度数据。
- `PID_gyro` 以 `-imu660rc_gyro_y` 为反馈，输出目标轮速 `PID_R.target_val`。
- 根据 `B_X`、`B_H`、`E_H` 设置左右腿末端坐标。
- `inverseKinematics()` 将腿部坐标解算为四个舵机角度/PWM。
- `motor_Loop()` 根据目标轮速和方向差速输出左右电机占空比。

这相当于角速度内环：目标角速度 - 实际角速度 -> 目标轮速/电机输出。

## 3. 控制数据流总览

整体闭环可以按下面理解：

```text
IMU INT2 外部中断
    -> imu660rc_callback()
    -> 更新 pitch / gyro_y / gyro_z

UART1 接收中断
    -> uart_control_callback()
    -> 更新左右轮速度 motor_value.receive_*_speed_data

PIT_CH0 10ms
    -> 按键扫描
    -> 速度外环 PID_leg
    -> 方向 PID_dir
    -> 更新 B_X 与差速输出

PIT_CH1 5ms
    -> 姿态角 PID_balance
    -> 更新 PID_gyro.target_val

PIT_CH2 2ms
    -> 请求速度
    -> 角速度 PID_gyro
    -> 设置腿坐标
    -> 逆运动学输出舵机 PWM
    -> motor_Loop 输出电机占空比

main while
    -> menu_Display()
    -> TFT 菜单显示和 PID 参数调节
```

## 4. code 目录模块说明

### 4.1 `pid.c/.h`

定义通用 PID 结构体和两个算法：

- `PosionPID_realize()`：位置式 PID。
- `addPID_realize()`：增量式 PID。

全局 PID 对象：

- `PID_leg`：速度外环，输出腿部 X 坐标修正。
- `PID_balance`：pitch 角度环，输出目标角速度。
- `PID_gyro`：角速度环，输出目标轮速。
- `PID_dir`：yaw 角速度方向环，输出左右差速。
- `PID_L`、`PID_R`：左右/轮速相关 PID，其中当前主要使用 `PID_R.target_val`。
- `PID_pitch`、`PID_roll`、`PID_out`：已经初始化，但当前主控制链里使用较少或暂未接入。

`PosionPID_realize()` 流程：

1. `Error = target_val - actual_val`
2. 积分累加 `integral += Error`
3. 计算 `Kp * Error + Ki * integral + Kd * (Error - LastError)`
4. 更新 `LastError`
5. 按 `max/min` 限幅

注意：代码中积分限幅被注释掉了，长时间运行时积分项可能累积。

### 4.2 `motor_control.c/.h`

负责 PID 参数初始化、速度滤波和电机输出。

主要全局变量：

- `B_X` 在 `leg_control.c` 中定义，但由 `PIT_CH0` 根据速度外环更新。
- `car_speed`：由左右轮速度计算得到。
- `Lroll`、`Rroll`：滚转补偿入口，当前默认值为 0。
- `Middle_angle`、`stab_roll` 等变量当前未完整接入主控制链。

`motor_Loop()` 逻辑：

```c
l = PID_R.target_val - PID_dir.output_val + Lroll;
r = PID_R.target_val + PID_dir.output_val + Rroll;
```

随后将左右输出限制在 `[-50, 50]`，再调用：

```c
motor_set_pwm(l, r);
```

`motor_set_pwm` 宏内部会把百分比形式的 `l/r` 转为驱动板需要的占空比数据，并调用 `small_driver_set_duty()` 通过 UART 发送。

### 4.3 `small_driver_uart_control.c/.h`

负责与无刷驱动板的 UART1 协议通信。

串口配置：

- UART：`UART_1`
- 波特率：`460800`
- 引脚宏：`SMALL_DRIVER_RX`、`SMALL_DRIVER_TX`

通信帧长度固定 7 字节：

```text
[0] 0xA5 帧头
[1] 功能字
[2] 左数据高字节
[3] 左数据低字节
[4] 右数据高字节
[5] 右数据低字节
[6] 前 6 字节累加和校验
```

功能字：

- `0x01`：设置左右电机占空比。
- `0x02`：请求/接收左右轮速度。

主要函数：

- `small_driver_uart_init()`：初始化 UART1、开接收中断、清结构体、发送 0 占空比、请求速度。
- `small_driver_set_duty(left_duty, right_duty)`：发送占空比命令。
- `small_driver_get_speed()`：发送速度请求命令。
- `uart_control_callback()`：在 UART1 接收中断里解析速度反馈。

### 4.4 `leg_control.c/.h`

负责轮腿机构的坐标设置、逆运动学解算和四个舵机 PWM 输出。

四个舵机引脚：

- `LeftFront`
- `LeftRear`
- `RightFront`
- `RightRear`

机构尺寸参数：

- `L1 = 61`
- `L2 = 91`
- `L3 = 91`
- `L4 = 61`
- `L5 = 37`
- `L6 = 107`

核心变量：

- `B_H = 45`：基础高度。
- `B_X = 18.45`：基础 X 坐标，运行中会被速度外环修正。
- `E_H`：左右腿高度差，当前默认 0，接口已经预留。
- `IKParam`：保存左右腿末端坐标和解算出的角度。

`leg_position_set(X, yleft, yright)`：

- 将 `yleft/yright` 限制在 `[25, 150]`。
- 左腿坐标：`XLeft = X`，`YLeft = yleft`。
- 右腿坐标：`XRight = 37 - X`，`YRight = yright`。

`inverseKinematics()`：

1. 根据左右腿目标坐标分别计算 `alpha`、`beta` 两个关节角。
2. 将弧度转换为角度。
3. 映射到四个舵机的 PWM duty。
4. 对 duty 做上限保护。
5. 调用 `pwm_set_duty()` 输出到四个舵机。

注意：逆解中 `sqrt(...)` 内部没有做可达性保护。如果目标坐标超出机构可达范围，可能出现负数开方导致异常结果。当前主要靠 `leg_position_set()` 的高度限幅和控制参数维持在可达范围内。

### 4.5 `key.c/.h`

负责 4 个按键的扫描和消抖。

按键引脚：

- `KEY1 = P20_0`
- `KEY2 = P20_1`
- `KEY3 = P20_2`
- `KEY4 = P20_3`

`Get_KeyNum()` 每次调用会扫描四个按键。`KeyTriggerFunc()` 使用 3 次采样一致判断做简单消抖，并在下降沿产生一次键值：

- `KEY1 -> Num_Key = 3`
- `KEY2 -> Num_Key = 4`
- `KEY3 -> Num_Key = 2`
- `KEY4 -> Num_Key = 1`

当前 `Get_KeyNum()` 在 10ms 的 `PIT_CH0` 中调用，因此按键消抖窗口约为 30ms。

### 4.6 `menu.c/.h`

负责 TFT180 屏幕菜单和在线参数修改。

主菜单：

```text
task
PID_gyro
PID_angle
PID_speed
```

二级菜单：

- `task` -> `task1/taks2/task3`
- `PID_gyro` -> `P_g/I_g/D_g`
- `PID_angle` -> `P_a/I_a/D_a`
- `PID_speed` -> `P_s/I_s/D_s`

当前实际绑定可调参数在 `menu_num[]`：

```c
{1,4,1,&PID_leg.Kp},
{1,4,2,&PID_dir.Kp},
{1,4,3,&PID_leg.Kd},
```

也就是在第 1 层、第 4 页 `PID_speed` 页面中，三行分别调：

- `PID_leg.Kp`
- `PID_dir.Kp`
- `PID_leg.Kd`

按键逻辑：

- `Num_Key == 1`：菜单向下；进入数值编辑时数值减小 `0.001`。
- `Num_Key == 2`：菜单向上；进入数值编辑时数值增加 `0.001`。
- `Num_Key == 3`：确认/进入下一级/进入编辑。
- `Num_Key == 4`：返回/退出编辑。

注意：`menu_Display()` 内部会调用 `menu_Func()`，而 `menu_Func()` 使用 `while(1)` 阻塞式刷新菜单。由于控制闭环在中断中运行，主循环被菜单阻塞并不会停止中断控制，但屏幕菜单逻辑本身是阻塞式的。

## 5. 当前控制链中的关键参数

| PID | 目标 | 反馈 | 输出用途 | 主要参数 |
| --- | --- | --- | --- | --- |
| `PID_leg` | 目标速度 0 | `car_speed` | 修正 `B_X` | `Kp=-0.02`, 限幅 `[-8, 8]` |
| `PID_dir` | 目标 yaw 角速度 0 | `imu660rc_gyro_z` | 左右差速 | `Kp=-0.012`, 限幅 `[-30, 30]` |
| `PID_balance` | 目标 pitch 角 6 | `imu660rc_pitch` | `PID_gyro.target_val` | `Kp=280`, `Kd=2.2` |
| `PID_gyro` | 目标 pitch 角速度 | `-imu660rc_gyro_y` | `PID_R.target_val` | `Kp=0.012`, `Kd=0.0005` |
| `PID_R` | 目标轮速 | 当前未直接闭环实际轮速 | 电机基准输出 | 主要使用 `target_val` |

## 6. 需要特别留意的点

1. IMU 欧拉角依赖 `INT2 -> P06_7` 外部中断接线；未接 INT2 时，四元数/欧拉角模式不能正常更新。
2. `PIT_CH2` 每 2ms 调用 `small_driver_get_speed()`，而注释里驱动板速度输出默认 10ms，实际通信频率需要结合驱动板协议确认。
3. `PID_L`、`PID_pitch`、`PID_roll`、`PID_out` 已初始化，但当前主控制链基本未使用，后续可能是预留功能。
4. `Lowpass()` 计算了滤波速度，但主速度外环当前没有使用滤波值。
5. `motor_value.receive_left_speed_data` 和 `receive_right_speed_data` 是 `int16`，接收解析时直接拼接高低字节，需要确认编译器下符号扩展是否符合预期。
6. `inverseKinematics()` 对机构可达范围缺少数学保护，调参时不要让 `B_X`、`B_H`、`E_H` 超出可达范围。
7. `motor_Loop()` 最终把左右输出限幅到 `[-50, 50]`，再由宏换算为驱动板占空比。调 PID 时要注意这个限幅会掩盖更大的 PID 输出。

## 7. 推荐阅读顺序

1. `user/main_cm7_0.c`：看初始化顺序和主循环。
2. `user/cm7_0_isr.c`：看 IMU、UART、PIT 三类中断如何驱动整个程序。
3. `code/pid.c` 和 `code/motor_control.c`：看控制量如何计算。
4. `code/leg_control.c`：看腿部坐标如何变成舵机 PWM。
5. `code/small_driver_uart_control.c`：看电机驱动板协议。
6. `code/key.c` 和 `code/menu.c`：看 TFT 菜单如何调参数。

