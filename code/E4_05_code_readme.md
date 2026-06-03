# E4_05 轮腿平衡与视觉巡线代码阅读文档

本文档基于当前 `luntui_3` 工程代码整理，用于快速理解 `code/` 目录与 `user/main_cm7_0.c`、`user/cm7_0_isr.c` 之间的关系。

当前程序已经不是单纯的 IMU660RC 示例，而是一个由轮腿平衡控制、MT9V03X 摄像头巡线、单边桥元素处理、TFT 菜单和无刷驱动板串口通信共同组成的轮腿车控制框架。

## 1. 工程入口与初始化流程

主入口在 `user/main_cm7_0.c`，`main_cm7_1.c` 基本保持模板状态。`main_cm7_0.c` 的初始化顺序如下：

1. `clock_init(SYSTEM_CLOCK_250M)`：系统时钟与底层初始化。
2. `debug_init()`：调试串口初始化。
3. `tft180_set_dir()`、`tft180_set_color()`、`tft180_init()`：初始化 TFT180 屏幕。
4. `gpio_init(KEY1..KEY4, ...)`：初始化 4 个上拉输入按键。
5. `leg_control_init()`：初始化 4 路舵机 PWM，并设置轮腿初始坐标。
6. `small_driver_uart_init()`：初始化 UART1，与无刷驱动板通信。
7. `flash_init()`：CYT 系列 Flash 初始化。
8. 初始化所有 PID 对象：`PID_leg`、`PID_out`、`PID_L`、`PID_R`、`PID_balance`、`PID_gyro`、`PID_dir`、`PID_pitch`、`PID_roll`。
9. `imu660rc_init(IMU660RC_QUARTERNION_120HZ)`：IMU660RC 以 120 Hz 四元数模式工作。
10. 初始化 3 个周期中断：
    - `PIT_CH0`：10 ms
    - `PIT_CH1`：5 ms
    - `PIT_CH2`：3 ms
11. 清零 `imu660rc_yaw` 和 `angle_n`。
12. `mt9v03x_init()`：初始化摄像头。

主循环只调用：

```c
while(true)
{
    menu_Display();
}
```

高频闭环控制放在 PIT 中断中，主循环负责 TFT 菜单、图像处理、巡线和元素状态机。

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

`P06_7` 对应 IMU660RC 的 `INT2` 引脚。IMU 产生数据就绪中断后，回调会更新四元数、欧拉角、角速度和加速度相关全局量，例如：

- `imu660rc_pitch`
- `imu660rc_roll`
- `imu660rc_yaw`
- `imu660rc_gyro_y`
- `imu660rc_gyro_z`

因此主循环里不再主动调用 `imu660rc_get_acc()` 或 `imu660rc_get_gyro()`。

### 2.2 UART 接收

当前使用到的串口中断主要有：

- `uart1_isr()`：调用 `uart_control_callback()` 解析无刷驱动板返回的左右轮速度，并调用 `Lowpass()` 更新 Z 轴角速度低通值。
- `uart2_isr()`：保留给 BLE6A20 蓝牙模块，当前 `main` 中 `ble6a20_init()` 被注释。
- `uart4_isr()`：调用 `uart_receiver_handler()`，保留给串口接收机。

无刷驱动板速度反馈最终进入：

- `motor_value.receive_left_speed_data`
- `motor_value.receive_right_speed_data`

### 2.3 PIT_CH0：10 ms 按键与速度外环

```c
time_ms += 10;
Get_KeyNum();
car_speed = (float)(-motor_value.receive_left_speed_data
                  + motor_value.receive_right_speed_data) / 2.0;
B_X = 18.48 + 1.6 * PosionPID_realize(&PID_leg, car_speed);
```

职责：

- 维护毫秒计时变量 `time_ms`。
- 扫描 4 个按键，更新 `Num_Key`。
- 由左右轮速度计算车体速度 `car_speed`。
- `PID_leg` 以速度为反馈，输出轮腿 X 坐标修正量，改变 `B_X`。

### 2.4 PIT_CH1：5 ms 姿态、方向与单边桥补偿

```c
PID_gyro.target_val = PosionPID_realize(&PID_balance, imu660rc_pitch);
angle_n = imu660rc_yaw - error;
```

职责：

- `PID_balance` 以 `imu660rc_pitch` 为反馈，输出目标俯仰角速度到 `PID_gyro.target_val`。
- 上电后一段时间取一次 `imu660rc_yaw` 作为零偏 `error`，之后计算相对航向角 `angle_n`。
- 将 `angle_n` 归一到 `[-180, 180]`。
- 普通巡线状态下，`PID_out` 以视觉误差 `error_dir` 为输入，输出目标水平角速度到 `PID_dir.target_val`。
- 单边桥状态下，使用进入单边桥时保存的 `dbq_yaw_ref` 做航向保持。
- 单边桥中段 `danbianqiao_flag == 2` 时，用 `PID_roll` 根据横滚角计算 `stab_roll`，再换算为左右腿高差 `E_H`。

### 2.5 PIT_CH2：3 ms 内环、逆解和电机输出

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
- `Lowpass()` 对 `imu660rc_gyro_z` 做一阶低通。
- `PID_dir` 以滤波后的 Z 轴角速度为反馈，输出左右差速修正。
- 根据 `B_X`、`B_H`、`E_H` 设置左右轮腿末端坐标。
- `inverseKinematics()` 将轮腿坐标解算为 4 路舵机 PWM。
- `motor_Loop()` 输出左右无刷电机占空比。

## 3. 主数据流

```text
IMU INT2 外部中断
    -> imu660rc_callback()
    -> 更新 pitch / roll / yaw / gyro_y / gyro_z

UART1 接收中断
    -> uart_control_callback()
    -> 更新 motor_value.receive_*_speed_data

PIT_CH0 10 ms
    -> time_ms
    -> Get_KeyNum()
    -> car_speed
    -> PID_leg 速度外环
    -> B_X

PIT_CH1 5 ms
    -> PID_balance 姿态角外环
    -> PID_gyro.target_val
    -> angle_n
    -> 普通状态: PID_out(error_dir) -> PID_dir.target_val
    -> 单边桥: PID_pitch / PID_roll -> PID_dir.target_val / E_H

PIT_CH2 3 ms
    -> small_driver_get_speed()
    -> PID_gyro 俯仰角速度内环
    -> Lowpass(gyro_z)
    -> PID_dir 水平角速度内环
    -> leg_position_set()
    -> inverseKinematics()
    -> motor_Loop()

main while
    -> menu_Display()
    -> mt9v03x_finish_flag 到达后处理图像
    -> scan() / zhaodian()
    -> element_judgment()
    -> element_perform()
    -> TFT 显示二值图与边线
```

## 4. 图像与巡线流程

图像相关代码主要在 `image.c/.h` 和 `scan.c/.h`。

### 4.1 `image.c/.h`

主要缓冲区：

- `Gray_filter[MT9V03X_H][MT9V03X_W]`：高斯滤波后的原分辨率灰度图。
- `Gray_zip[MT9V03X_H/2][MT9V03X_W/2]`：压缩后的灰度图。
- `image_sobel[MT9V03X_H/2][MT9V03X_W/2]`：二值图或边缘图，当前主流程使用二值化结果。
- `thresholds`：大津法阈值。

主流程在 `menu_Func()` 中，当 `mt9v03x_finish_flag` 置位后执行：

```c
fileOverview();
zip();
thresholds = otsuThreshold(Gray_zip[0], MT9V03X_W/2, MT9V03X_H/2);
erzhihua();
scan();
zhaodian();
element_judgment();
element_perform();
mt9v03x_finish_flag = 0;
```

各函数职责：

- `fileOverview()`：3x3 加权滤波。
- `zip()`：隔点采样，把图像压缩为一半宽高。
- `otsuThreshold()`：大津法计算二值化阈值。
- `erzhihua()`：根据 `thresholds` 生成黑白图。
- `sobel()`：Sobel 边缘检测函数保留，但当前主流程未调用。

### 4.2 `scan.c/.h`

核心数组：

- `Rline[]`、`Lline[]`、`Mline[]`：巡线使用的右边线、左边线、中线。
- `Rline_zhaodian[]`、`Lline_zhaodian[]`、`Mline_zhaodian[]`：寻找拐点使用的边线与中线。
- `White_Column[]`、`zchBailie`：统计中间区域白列长度，用于元素判断。

`scan()` 从图像下方往上搜索左右边界，并计算中线：

```c
Mline[i] = (int)(((Rline[i] + Lline[i]) / 2)
               + (r_empty - l_empty) * 0.8);
```

随后调用 `track()` 得到方向误差：

```c
error_dir = track();
last_error_dir = error_dir;
```

`track()` 根据近处前瞻 `qianzhan_jin` 和远处前瞻 `qianzhan_yuan` 取若干行中线，计算相对于 `zhongxian` 的偏差。这个偏差最终在 PIT_CH1 中进入 `PID_out`，形成方向控制目标。

`zhaodian()` 与 `linyu()` 用于寻找 A/B/C/D 四类拐点，并统计白列：

- A/C：左侧相关拐点。
- B/D：右侧相关拐点。
- `A_finish_flag` 等标志表示对应拐点是否找到。
- `A_kur`、`B_kur` 是拐点邻域峰度统计。

## 5. 特殊元素状态机

当前实际接入的特殊元素是单边桥：

```c
typedef enum
{
    Wu          = 0,
    PoDao       = 1,
    DanBianQiao = 2,
    Jump        = 3,
} element_type_enum;
```

`PoDao` 和 `Jump` 的判断/控制代码目前大部分被注释，`element_perform()` 中也未接入。

### 5.1 单边桥判断

`DanBianQiao_judgment()` 综合以下信息判断：

- A/B/C/D 拐点组合。
- 拐点之间的几何关系。
- 中间白列长度 `zchBailie`。
- `danbianqiao_flag == 0`，避免重复进入。
- `time_ms > 1100`，上电初期不判断。

满足条件后，`element_judgment()` 会：

```c
element_type = DanBianQiao;
danbianqiao_flag = 1;
DBQ_time1 = time_ms;
empty_flag = 1;
PID_out.Kp = 0;
dbq_yaw_ref = angle_n;
```

也就是进入单边桥时关闭视觉方向外环，并记录当前航向作为参考。

### 5.2 单边桥执行

`DanBianQiao_control()` 通过 `danbianqiao_flag` 分阶段执行：

- `flag == 1`：进入后的减速/过渡阶段，持续 `jiansu_time`。
- `flag == 2`：桥上阶段，PIT_CH1 会基于横滚角计算 `E_H`，让左右腿产生高度差。
- `flag == 3`：退出阶段，恢复部分参数，例如 `PID_out.Kp = -90.0`、`PID_leg` 限幅。

相关可调参数在 `scan.c` 顶部：

- `jichu_sudu`
- `tuibu_xianfu`
- `qianzhan`
- `jiansu_time`
- `dbqiao_time`
- `dbqiao_tuigao`
- `jump_qianzhan`
- `dbq_tbxianfu`
- `roll_kp`

其中部分参数当前没有完全接入主链路，但保留为调试入口。

## 6. 控制模块说明

### 6.1 `pid.c/.h`

定义通用 PID 结构体和两个算法：

- `PosionPID_realize()`：位置式 PID。
- `addPID_realize()`：增量式 PID。

全局 PID 对象：

- `PID_leg`：速度外环，输出轮腿 X 坐标修正。
- `PID_balance`：pitch 角度环，输出目标俯仰角速度。
- `PID_gyro`：俯仰角速度环，输出电机基础控制量。
- `PID_out`：视觉巡线方向外环，输入 `error_dir`。
- `PID_dir`：水平角速度内环，输出左右差速。
- `PID_pitch`：单边桥航向保持相关外环。
- `PID_roll`：单边桥横滚补偿。
- `PID_L`、`PID_R`：轮速相关 PID；当前主链路主要借用 `PID_R.target_val` 作为电机基础输出。

注意点：

- `PosionPID_realize()` 内部直接处理 `PID_pitch.Error`，这段逻辑写在通用 PID 函数里，不论传入哪个 PID 都会执行。
- 负向积分限幅当前写成 `pid->integral = pid->integralmax`，更像是笔误，通常应为 `-pid->integralmax`。若后续出现积分异常，应优先检查这里。

### 6.2 `motor_control.c/.h`

负责 PID 参数初始化、角速度滤波和电机输出。

主要变量：

- `car_speed`：由左右轮速度反馈计算得到。
- `angle_n`：相对航向角。
- `Lowpass_imu660rc_gyro_z`：Z 轴角速度一阶低通结果。
- `stab_roll`：单边桥横滚补偿角。
- `Lroll`、`Rroll`：左右电机附加补偿入口，当前默认未明显接入。

`Lowpass()`：

```c
Lowpass_imu660rc_gyro_z =
    (int)(0.2 * (float)imu660rc_gyro_z
        + 0.8 * (float)last_imu660rc_gyro_z);
```

`motor_Loop()`：

```c
l = PID_R.target_val - PID_dir.output_val + Lroll;
r = PID_R.target_val + PID_dir.output_val + Rroll;
```

随后将 `l/r` 限幅到 `[-50, 50]`，再调用 `motor_set_pwm(l, r)`。`motor_set_pwm` 宏会把百分比形式的左右输出换算为驱动板占空比，并通过 UART1 发送。

### 6.3 `leg_control.c/.h`

负责轮腿机构坐标设置、逆运动学解算和四个舵机 PWM 输出。

四个舵机 PWM 引脚：

- `LeftFront = TCPWM_CH12_P05_3`
- `LeftRear = TCPWM_CH11_P05_2`
- `RightFront = TCPWM_CH10_P05_1`
- `RightRear = TCPWM_CH09_P05_0`

主要变量：

- `B_H = 45`：基础高度。
- `B_X = 18.45`：基础 X 坐标，运行中会被速度外环修正。
- `E_H`：左右腿高度差，用于单边桥横滚补偿。
- `IKParam`：保存左右腿末端坐标和逆解得到的角度。

`leg_position_set(X, yleft, yright)`：

- 将 `yleft/yright` 限制在 `[25, 150]`。
- 左腿坐标：`XLeft = X`，`YLeft = yleft`。
- 右腿坐标：`XRight = 35.5 - X`，`YRight = yright`。

`inverseKinematics()`：

1. 分别计算左右腿 `alpha/beta` 关节角。
2. 弧度转角度。
3. 映射为四路舵机 PWM duty。
4. 对 duty 做上限保护。
5. 调用 `pwm_set_duty()` 输出到舵机。

注意：逆解中的 `sqrt(...)` 没有做可达性保护，目标坐标超出机构可达范围时可能出现负数开方。当前主要依赖 `leg_position_set()` 的高度限幅和 PID 参数控制可达范围。

### 6.4 `small_driver_uart_control.c/.h`

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

注意：速度解析把高低字节先拼成 `int`，再赋给结构体成员。若驱动板返回负数，需要确认结构体成员类型和补码转换符合预期。

### 6.5 `key.c/.h`

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

### 6.6 `menu.c/.h`

TFT 菜单使用两层结构：

- 第一层：`task`
- 第二层：`task`、`PID`

菜单通过 `Num_Key` 控制：

- `Num_Key == 1`：向下或数值减小。
- `Num_Key == 2`：向上或数值增大。
- `Num_Key == 3`：确认。
- `Num_Key == 4`：返回。

当前菜单主要承担两类工作：

- 当 `pid_flag >= 1` 时，写入一组比赛/调试用 PID 参数。
- 当 `mt9v03x_finish_flag` 置位时，处理一帧图像、巡线、元素判断，并在 TFT 上显示二值图和左右边线。

## 7. 当前需要特别注意的点

1. `E4_05_code_readme.md` 旧内容中提到的 `Nag_System()`、Flash 惯导记录/复现流程，在当前源码主链路中已经不存在或未接入。
2. `PIT_CH2` 当前周期是 3 ms，不是旧文档里的 2 ms。
3. 当前主方向控制来自视觉巡线 `error_dir`，不是单纯的 IMU 航向角目标。
4. `PoDao` 和 `Jump` 相关代码大多被注释，枚举还保留，但没有进入当前执行链路。
5. `PID` 通用函数里存在疑似笔误和跨对象副作用，调参或排查异常时要优先关注。
6. `menu.c` 中承担了图像处理与元素状态机调度，主循环若被菜单阻塞或显示耗时过长，会影响视觉更新频率。
7. `scan.c` 中多处数组访问依赖前置几何条件，例如 `DY+16`、`CY+16`、`Mline[...]` 等，若拐点未找到或坐标异常，存在越界风险。

## 8. 快速定位表

| 功能 | 主要文件 | 关键函数/变量 |
| --- | --- | --- |
| 主入口 | `user/main_cm7_0.c` | `main()` |
| 周期调度 | `user/cm7_0_isr.c` | `pit0_ch0_isr()`、`pit0_ch1_isr()`、`pit0_ch2_isr()` |
| IMU 更新 | `user/cm7_0_isr.c` | `gpio_6_exti_isr()`、`imu660rc_callback()` |
| 无刷驱动通信 | `small_driver_uart_control.c` | `small_driver_set_duty()`、`small_driver_get_speed()`、`uart_control_callback()` |
| 平衡控制 | `motor_control.c`、`pid.c` | `PID_balance`、`PID_gyro`、`PID_dir`、`motor_Loop()` |
| 轮腿逆解 | `leg_control.c` | `leg_position_set()`、`inverseKinematics()` |
| 图像预处理 | `image.c` | `fileOverview()`、`zip()`、`otsuThreshold()`、`erzhihua()` |
| 巡线 | `scan.c` | `scan()`、`track()`、`error_dir` |
| 拐点/元素 | `scan.c` | `zhaodian()`、`DanBianQiao_judgment()`、`element_perform()` |
| 菜单与显示 | `menu.c` | `menu_Display()`、`menu_Func()` |
| 按键 | `key.c` | `Get_KeyNum()`、`Num_Key` |
