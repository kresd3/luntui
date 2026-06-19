# E4_05 轮腿平衡与惯导代码阅读文档

本文档基于当前工程代码重新整理，用于快速理解 `luntui/code` 目录以及它和 `user/main_cm7_0.c`、`user/cm7_0_isr.c` 之间的关系。当前程序已经不是单纯的 IMU660RC 示例，而是一个围绕 IMU 姿态、无刷驱动板速度反馈、轮腿机构逆运动学、TFT 菜单和 Flash 惯导轨迹记录/复现构成的轮腿平衡控制程序雏形。

## 1. 程序入口与初始化流程

主入口在 `user/main_cm7_0.c`，`main_cm7_1.c` 基本保持模板状态。`main_cm7_0.c` 的主要初始化顺序如下：

1. `clock_init(SYSTEM_CLOCK_250M)`：系统时钟和底层初始化。
2. `debug_init()`：调试串口初始化。
3. `tft180_set_dir()`、`tft180_set_color()`、`tft180_init()`：初始化 TFT180 屏幕。
4. `gpio_init(KEY1..KEY4, ...)`：初始化 4 个上拉输入按键。
5. `leg_control_init()`：初始化 4 路舵机 PWM，并设置初始腿部坐标。
6. `small_driver_uart_init()`：初始化 UART1，与无刷驱动板通信。
7. `ble6a20_init()`：初始化 BLE6A20 蓝牙模块。
8. `flash_init()`：CYT 系列 Flash 初始化。
9. `Init_Nag()`：惯导结构体和 Flash 缓冲区初始化。
10. 初始化所有 PID 对象：`PID_leg`、`PID_out`、`PID_L`、`PID_R`、`PID_balance`、`PID_gyro`、`PID_dir`、`PID_pitch`、`PID_roll`。
11. `imu660rc_init(IMU660RC_QUARTERNION_120HZ)`：IMU660RC 以 120 Hz 四元数模式工作。
12. 初始化 3 个周期中断：
    - `PIT_CH0`：10 ms
    - `PIT_CH1`：5 ms
    - `PIT_CH2`：2 ms
13. 清零 `imu660rc_yaw` 和 `angle_n`。

主循环只执行屏幕菜单：

```c
while(true)
{
    menu_Display();
}
```

高频闭环控制主要放在 PIT 中断里，主循环负责 TFT 菜单显示、任务切换和参数触发。

## 2. 中断调度框架

核心调度在 `user/cm7_0_isr.c`。

### 2.1 IMU 数据采集

`gpio_6_exti_isr()` 判断 `P06_7` 外部中断：

```c
if(exti_flag_get(P06_7))
{
    imu660rc_callback();
}
```

`P06_7` 对应 IMU660RC 的 `INT2` 引脚。IMU 产生数据就绪中断后，回调会更新四元数、欧拉角、角速度和加速度相关全局量，例如 `imu660rc_pitch`、`imu660rc_yaw`、`imu660rc_gyro_y`、`imu660rc_gyro_z` 等。因此主循环里不再主动调用 `imu660rc_get_acc()` 或 `imu660rc_get_gyro()`。

### 2.2 UART 接收

当前使用到的串口中断主要有：

- `uart1_isr()`：调用 `uart_control_callback()` 解析无刷驱动板返回的左右轮速度，并调用 `Lowpass()` 更新滤波后的 Z 轴角速度。
- `uart2_isr()`：调用 `ble6a20_callback()` 处理蓝牙模块。
- `uart4_isr()`：调用 `uart_receiver_handler()`，保留给串口接收机。

无刷驱动板速度反馈最终进入：

- `motor_value.receive_left_speed_data`
- `motor_value.receive_right_speed_data`

### 2.3 PIT_CH0：10 ms 按键与速度外环

```c
Get_KeyNum();
car_speed = (float)(-motor_value.receive_left_speed_data
                  + motor_value.receive_right_speed_data) / 2.0;
B_X = 18.48 + 1.6 * PosionPID_realize(&PID_leg, car_speed);
```

职责：

- 扫描 4 个按键，更新 `Num_Key`。
- 由左右轮速度计算车体速度 `car_speed`。
- `PID_leg` 以速度为反馈，输出腿部 X 坐标修正量，改变 `B_X`。

### 2.4 PIT_CH1：5 ms 姿态外环与惯导调度

```c
Nag_System();
PID_gyro.target_val = PosionPID_realize(&PID_balance, imu660rc_pitch);

if(imu660rc_yaw <= 180) angle_n = imu660rc_yaw - error;
else if(imu660rc_yaw > 180) angle_n = (imu660rc_yaw - error) - 360;

PID_pitch.target_val = N.Angle_Run;
PID_dir.target_val = PosionPID_realize(&PID_pitch, angle_n);
```

职责：

- 调用 `Nag_System()`，根据菜单设置执行惯导记录或复现。
- `PID_balance` 以 `imu660rc_pitch` 为反馈，输出目标俯仰角速度到 `PID_gyro.target_val`。
- 上电后约 10 s 取一次 `imu660rc_yaw` 作为零偏 `error`，之后计算相对航向角 `angle_n`。
- `PID_pitch` 以 `angle_n` 为反馈，以 `N.Angle_Run` 为目标，输出目标水平角速度到 `PID_dir.target_val`。

### 2.5 PIT_CH2：2 ms 内环、逆解和电机输出

```c
small_driver_get_speed();
PID_R.target_val = PosionPID_realize(&PID_gyro, -imu660rc_gyro_y);

Lowpass();
PosionPID_realize(&PID_dir, Lowpass_imu660rc_gyro_z);

leg_position_set(B_X, B_H + E_H, B_H - E_H);
inverseKinematics();
motor_Loop();
```

职责：

- 向无刷驱动板发送速度请求。
- `PID_gyro` 以 `-imu660rc_gyro_y` 为反馈，输出电机基础控制量到 `PID_R.target_val`。
- `Lowpass()` 对 `imu660rc_gyro_z` 做一阶滤波。
- `PID_dir` 以滤波后的 Z 轴角速度为反馈，输出左右差速修正。
- 根据 `B_X`、`B_H`、`E_H` 设置左右腿末端坐标。
- `inverseKinematics()` 将腿部坐标解算为 4 路舵机 PWM。
- `motor_Loop()` 输出左右电机占空比。

## 3. 控制数据流总览

```text
IMU INT2 外部中断
    -> imu660rc_callback()
    -> 更新 pitch / yaw / gyro_y / gyro_z 等姿态数据

UART1 接收中断
    -> uart_control_callback()
    -> 更新 motor_value.receive_*_speed_data

PIT_CH0 10 ms
    -> Get_KeyNum()
    -> car_speed
    -> PID_leg 速度外环
    -> 更新 B_X

PIT_CH1 5 ms
    -> Nag_System()
    -> PID_balance 姿态角外环
    -> 更新 PID_gyro.target_val
    -> 计算 angle_n
    -> PID_pitch 航向角外环
    -> 更新 PID_dir.target_val

PIT_CH2 2 ms
    -> small_driver_get_speed()
    -> PID_gyro 俯仰角速度内环
    -> Lowpass 过滤 gyro_z
    -> PID_dir 水平角速度内环
    -> leg_position_set()
    -> inverseKinematics()
    -> motor_Loop()

main while
    -> menu_Display()
    -> TFT 菜单、任务切换、PID 预设、惯导记录/复现触发
```

## 4. code 目录模块说明

### 4.1 `pid.c/.h`

定义通用 PID 结构体和两个算法：

- `PosionPID_realize()`：位置式 PID。
- `addPID_realize()`：增量式 PID。

全局 PID 对象：

- `PID_leg`：速度外环，输出腿部 X 坐标修正。
- `PID_balance`：pitch 角度环，输出目标俯仰角速度。
- `PID_gyro`：俯仰角速度环，输出电机基础控制量。
- `PID_pitch`：航向角外环，输出目标水平角速度。
- `PID_dir`：水平角速度内环，输出左右差速。
- `PID_L`、`PID_R`：轮速相关 PID；当前主链路主要借用 `PID_R.target_val` 作为电机基础输出。
- `PID_roll`、`PID_out`：已初始化，当前主控制链路接入较少。

`PosionPID_realize()` 的基本流程：

1. `Error = target_val - actual_val`
2. 积分累加并按 `integralmax` 限幅
3. 计算 `Kp * Error + Ki * integral + Kd * (Error - LastError)`
4. 更新 `LastError`
5. 按 `max/min` 限制输出

注意：当前代码里有一段针对 `PID_pitch.Error` 的特殊处理，但写在通用 PID 函数中，不论传入哪个 PID 都会执行。另一个细节是负向积分限幅写成了 `pid->integral = pid->integralmax`，看起来更像应为 `-pid->integralmax`，后续若发现积分行为异常应优先检查这里。

### 4.2 `motor_control.c/.h`

负责 PID 参数初始化、角速度滤波和电机输出。

主要变量：

- `car_speed`：由左右轮速度计算得到。
- `angle_n`：相对航向角。
- `Lowpass_imu660rc_gyro_z`：Z 轴角速度一阶低通滤波结果。
- `Lroll`、`Rroll`：左右电机附加补偿入口，当前默认为 0。

`Lowpass()`：

```c
Lowpass_imu660rc_gyro_z =
    (int)(0.2 * (float)imu660rc_gyro_z + 0.8 * (float)last_imu660rc_gyro_z);
```

`motor_Loop()`：

```c
l = PID_R.target_val - PID_dir.output_val + Lroll;
r = PID_R.target_val + PID_dir.output_val + Rroll;
```

随后把 `l/r` 限幅到 `[-50, 50]`，再调用：

```c
motor_set_pwm(l, r);
```

`motor_set_pwm` 宏把百分比形式的左右输出换算为驱动板占空比，并调用 `small_driver_set_duty()` 通过 UART1 发送。

### 4.3 `small_driver_uart_control.c/.h`

负责与无刷驱动板的 UART1 通信。

串口配置：

- 串口：`UART_1`
- 波特率：`460800`
- 引脚宏：`SMALL_DRIVER_RX`、`SMALL_DRIVER_TX`

通信帧固定 7 字节：

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
- `0x02`：请求或接收左右轮速度。

主要函数：

- `small_driver_uart_init()`：初始化 UART1、打开接收中断、清结构体、发送 0 占空比、请求速度。
- `small_driver_set_duty(left_duty, right_duty)`：发送占空比命令。
- `small_driver_get_speed()`：发送速度请求命令。
- `uart_control_callback()`：在 UART1 接收中断里解析速度反馈。

注意：速度解析把高低字节先拼成 `int`，再赋给 `int16`。如果驱动板返回负数，需要确认补码和类型转换符合预期。

### 4.4 `leg_control.c/.h`

负责轮腿机构的坐标设置、逆运动学解算和四个舵机 PWM 输出。

四个舵机 PWM 引脚：

- `LeftFront = TCPWM_CH12_P05_3`
- `LeftRear = TCPWM_CH11_P05_2`
- `RightFront = TCPWM_CH10_P05_1`
- `RightRear = TCPWM_CH09_P05_0`

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
- `E_H`：左右腿高度差，当前默认 0。
- `IKParam`：保存左右腿末端坐标和逆解得到的角度。

`leg_position_set(X, yleft, yright)`：

- 把 `yleft/yright` 限制在 `[25, 150]`。
- 左腿坐标：`XLeft = X`，`YLeft = yleft`。
- 右腿坐标：`XRight = 37 - X`，`YRight = yright`。

`inverseKinematics()`：

1. 分别计算左右腿 `alpha/beta` 关节角。
2. 弧度转角度。
3. 映射为四路舵机 PWM duty。
4. 对 duty 做上限保护。
5. 调用 `pwm_set_duty()` 输出到舵机。

注意：逆解中的 `sqrt(...)` 没有做可达性保护，目标坐标超出机构可达范围时可能出现负数开方。当前主要依赖 `leg_position_set()` 的高度限幅和 PID 参数控制可达范围。

### 4.5 `key.c/.h`

负责 4 个按键扫描和简单消抖。

按键引脚：

- `KEY1 = P20_0`
- `KEY2 = P20_1`
- `KEY3 = P20_2`
- `KEY4 = P20_3`

`Get_KeyNum()` 每次扫描四个按键。`KeyTriggerFunc()` 使用 3 次采样一致判断做消抖，并在下降沿产生键值：

- `KEY1 -> Num_Key = 3`
- `KEY2 -> Num_Key = 4`
- `KEY3 -> Num_Key = 2`
- `KEY4 -> Num_Key = 1`

当前 `Get_KeyNum()` 在 10 ms 的 `PIT_CH0` 中调用，因此消抖窗口约为 30 ms。

### 4.6 `menu.c/.h`

负责 TFT180 菜单、任务变量修改和功能触发。

一级菜单：

```text
task
PID_gyro
PID_angle
PID_speed
```

当前 `menu[]` 中真正有二级页面的项目：

- `task` -> `task / nag_r / nag_e / nag_s / PID`
- `PID_gyro` -> `P_g / I_g / D_g`
- `PID_angle` -> `P_a / I_a / D_a`
- `PID_speed` -> `P_s / I_s / D_s`

当前 `menu_num[]` 实际绑定的可编辑变量都在 `task` 页面：

```c
{1,1,1,&task},
{1,1,2,&navigation_rec},
{1,1,3,&navigation_end},
{1,1,4,&navigation_start},
{1,1,5,&pid_flag}
```

功能触发逻辑：

- `pid_flag >= 1`：写入一组预设 PID 参数，包括 `PID_gyro`、`PID_balance`、`PID_leg`、`PID_dir`、`PID_pitch`。
- `navigation_rec >= 1`：设置 `N.Nag_SystemRun_Index = 1`，进入惯导记录。
- `navigation_end >= 1 && N.Nag_SystemRun_Index == 1`：设置 `N.End_f = 1`，结束记录并写入最后一页。
- `navigation_start >= 1`：设置 `N.Nag_SystemRun_Index = 2`，准备惯导复现。
- `N.Nag_SystemRun_Index == 2`：调用 `NagFlashRead()`，从 Flash 读取记录数据，读取完成后内部会自增到复现状态。

按键逻辑：

- `Num_Key == 1`：菜单向下；编辑模式下数值减 1。
- `Num_Key == 2`：菜单向上；编辑模式下数值加 1。
- `Num_Key == 3`：确认、进入下一级或进入编辑。
- `Num_Key == 4`：返回或退出编辑。

注意：`menu_Display()` 内部最终进入 `menu_Func()` 的 `while(1)`，菜单刷新是阻塞式的。由于控制闭环在中断里运行，主循环阻塞不会停止中断控制，但菜单代码本身不会自然返回到主循环。

### 4.7 `navigation.c/.h`

负责基于里程计的偏航角记录和复现，数据通过 Flash 保存。

关键宏：

- `MaxSize = 500`：单页缓冲中用于导航数据的最大数量。
- `Read_MaxSize = 10000`：复现数组最大读取数量。
- `Nag_Start_Page = 45`：Flash 起始页。
- `Nag_End_Page = 1`：Flash 终止页，同时用于保存 `N.Save_index`。
- `Nag_Set_mileage = 2100`：记录/复现的里程间隔。
- `Nag_Yaw = angle_n`：保存的偏航角来源。
- `L_Mileage = -motor_value.receive_left_speed_data`
- `R_Mileage = motor_value.receive_right_speed_data`

`N.Nag_SystemRun_Index` 是主要状态：

- `0`：惯导不运行。
- `1`：记录模式，`Nag_System()` 调用 `Nag_Read()`。
- `2`：菜单侧读取 Flash 的过渡状态，`menu_Func()` 调用 `NagFlashRead()`。
- `3`：复现模式，`Nag_System()` 调用 `Nag_Run()`。

记录流程：

1. `Run_Nag_Save()` 累加左右轮平均里程。
2. 每达到 `Nag_Set_mileage`，把 `Nag_Yaw * 100` 转成 `int32` 写入 `flash_union_buffer`。
3. 缓冲满 `MaxSize` 后调用 `flash_Nag_Write()` 写入当前 Flash 页，并让页索引递减。
4. 菜单触发结束时，`N.End_f = 1`，`Nag_Read()` 写入最后一页，并把 `N.Save_index` 写到 `Nag_End_Page`。

复现流程：

1. 菜单触发 `navigation_start` 后进入状态 2。
2. `NagFlashRead()` 读取 `Nag_End_Page` 得到 `N.Save_index`，再把各页记录读入 `Nav_read[]`。
3. 读取完成后 `N.Nag_SystemRun_Index++`，进入状态 3，并设置 `PID_leg.target_val = 300`。
4. `Run_Nag_GPS()` 按里程推进 `N.Run_index`，从 `Nav_read[]` 取出 `N.Angle_Run`。
5. `PIT_CH1` 中 `PID_pitch.target_val = N.Angle_Run`，使航向角外环跟随记录轨迹。

注意：`Nag_Run()` 里计算了 `N.Final_Out = imu660rc_yaw - N.Angle_Run`，但当前主控制链路实际使用的是 `N.Angle_Run` 作为 `PID_pitch.target_val`。

### 4.8 `flash.c/.h`

封装惯导数据的 Flash 读写。

- `flash_Nag_Write()`：必要时擦除页，把 `flash_union_buffer` 写入 `N.Flash_page_index`；结束记录时把 `N.Save_index` 写入 `Nag_End_Page`。
- `flash_Nag_Read()`：首次读取 `Nag_End_Page` 得到 `N.Save_index`，之后按 `N.Flash_page_index` 读取记录页到缓冲区。

注意：当前读函数内部有静态变量 `Index_R_f`，第一次读取后不会再次从 `Nag_End_Page` 刷新 `N.Save_index`。如果同一次上电期间需要重复读取不同记录，需要额外处理这个状态。

## 5. 当前控制链关键 PID

| PID | 目标 | 反馈 | 输出用途 | 初始化参数/运行时参数 |
| --- | --- | --- | --- | --- |
| `PID_leg` | 目标速度 | `car_speed` | 修正 `B_X` | 初始化 `Kp=0`，菜单预设后 `Kp=0.02`，限幅 `[-8, 8]` |
| `PID_balance` | 目标 pitch 角 `7` | `imu660rc_pitch` | `PID_gyro.target_val` | 初始化 `Kp=0`，菜单预设后 `Kp=200, Ki=0.5, Kd=1.2` |
| `PID_gyro` | 目标俯仰角速度 | `-imu660rc_gyro_y` | `PID_R.target_val` | 初始化 `Kp=0`，菜单预设后 `Kp=0.012, Kd=0.0005` |
| `PID_pitch` | 目标航向角 `N.Angle_Run` | `angle_n` | `PID_dir.target_val` | 初始化 `Kp=0`，菜单预设后 `Kp=120, Kd=0.5`，限幅 `[-2000, 2000]` |
| `PID_dir` | 目标水平角速度 | `Lowpass_imu660rc_gyro_z` | 左右差速 | 初始化 `Kp=0`，菜单预设后 `Kp=-0.008`，限幅 `[-30, 30]` |
| `PID_R` | 当前不直接闭环轮速 | 当前未接入实际轮速闭环 | 电机基础输出变量承载 | `Kp=0.019, Ki=0.00005`，但主链路主要使用 `target_val` |

## 6. 需要特别留意的点

1. IMU 欧拉角依赖 `INT2 -> P06_7` 外部中断接线；未接 INT2 时，四元数/欧拉角模式不能正常更新。
2. `PIT_CH2` 每 2 ms 调用 `small_driver_get_speed()`，而驱动板注释说明速度输出默认 10 ms，实际通信频率需要结合驱动板协议确认。
3. 菜单中 `PID_gyro/PID_angle/PID_speed` 页面目前没有实际绑定 PID 参数；真正绑定的是 `task` 页的 5 个任务变量。
4. 程序启动后 PID 参数初始多为 0，需要通过菜单 `PID` 项触发 `pid_flag` 后才会写入一组可运行的预设参数。
5. `N.Nag_SystemRun_Index == 2` 是 Flash 读取过渡状态，真正的惯导复现运行状态是 3。
6. `PosionPID_realize()` 中的 `PID_pitch.Error` 特殊处理和积分负限幅写法值得复查。
7. `inverseKinematics()` 缺少可达性数学保护，调参时不要让 `B_X`、`B_H`、`E_H` 超出机构可达范围。
8. `motor_Loop()` 最终把左右电机输出限幅到 `[-50, 50]`，调 PID 时需要注意这个限幅会掩盖更大的 PID 输出。
9. `small_driver_uart_control.h` 中 `SMALL_DRIVER_RX` 和 `SMALL_DRIVER_TX` 宏当前分别写为 `UART1_TX_P04_1`、`UART1_RX_P04_0`，命名和方向看起来容易混淆，接线时要按实际库宏含义确认。

## 7. 推荐阅读顺序

1. `user/main_cm7_0.c`：看初始化顺序和主循环。
2. `user/cm7_0_isr.c`：看 IMU、UART、PIT 如何驱动整个控制链。
3. `code/motor_control.c` 和 `code/pid.c`：看 PID 参数、速度/姿态控制量如何计算。
4. `code/leg_control.c`：看腿部坐标如何变成舵机 PWM。
5. `code/small_driver_uart_control.c`：看无刷驱动板通信协议。
6. `code/navigation.c` 和 `code/flash.c`：看偏航角轨迹如何记录、保存、读取和复现。
7. `code/key.c` 和 `code/menu.c`：看 TFT 菜单如何触发任务和参数预设。
