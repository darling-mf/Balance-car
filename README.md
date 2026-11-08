# STM32 双轮自平衡车（四环串级 PID + 倒立摆逆解算）

> 基于 STM32F103C8T6 的双轮自平衡车完整实现：MPU6050 姿态解算、编码器测速、**四环串级控制 + 倒立摆逆动力学解算**、电池电量监测与串口遥控。
> 无 RTOS，采用 **app 应用层 / my_lib 驱动层分层 + 协作式时间片调度**；控制算法调试、参数整定与代码走查修复均由本人完成。

---

## 一、功能特性

| 模块 | 说明 |
|---|---|
| 姿态解算 | MPU6050（±2000 °/s，±2 g），**互补滤波**（α=0.952，等效 τ≈0.1 s），输出 pitch / roll；14 字节 **burst read**，姿态刷新仅 1 次 I²C 事务 |
| 编码器测速 | **T 法测速**（测相邻脉冲时间差），低速不丢精度；换向点特殊处理 |
| 平衡控制 | **四环串级**：速度环 PI → θ 环 P → θ̇ 环 PID → 电机速度环 PI |
| 倒立摆逆解算 | 由期望角加速度反解期望线加速度，再积分成轮速给定 |
| 转向控制 | 陀螺 gz 闭环，输出左右轮**差速** |
| 电池监测 | ADC 注入通道 + TIM2_TRGO 硬件触发（100 Hz），四档电量 LED 指示，<20% 闪烁报警 |
| 遥控 | USART3 @9600，`move <turn> <move>` 文本协议，摇杆百分比映射到 ±0.7 m/s / ±15 rad/s |
| 显示 | 0.96" OLED（SPI） |
| 数学库 | 自实现 `qmath`（sin/cos/atan2），替换 libm 以省 Flash 与运算周期 |

---

## 二、硬件

| 部件 | 型号 / 参数 | 接口 |
|---|---|---|
| 主控 | STM32F103C8T6（64 KB Flash / 20 KB RAM） | — |
| 姿态 | MPU6050 | **硬件 I2C1**（重映射 PB8=SCL / PB9=SDA，400 kHz） |
| 电机 | 双直流减速电机 | TB6612/AT8236 + TIM PWM |
| 编码器 | 22 线，减速比 30613/1500 ≈ 20.4 | EXTI（PB14/PB15 左，PB3/PB4 右） |
| 电池 | 8.4 V 锂电，分压采样 | ADC1 注入通道 8（PB0），TIM2_TRGO 触发 |
| 显示 | SSD1306 0.96" OLED | SPI |
| 遥控 | 蓝牙/串口模块 | USART3（PB10/PB11），9600 8N1 |

> ⚠️ 右编码器使用 PB3/PB4，需 `GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE)` 释放 JTAG 占用，否则读不到脉冲。

---

## 三、控制算法

### 3.1 环结构

```
                   ┌──────────────────────────────────────────┐
   x_dot →  [速度环 PI] → θ_ref = atan(u/g)
                              ↓
                   [θ 环 P] → θ̇_ref
                              ↓
                 [θ̇ 环 PID] → θ̈_ref
                              ↓
       倒立摆逆解算  ẍ_ref = (g·sinθ − θ̈·Lp) / cosθ
                              ↓
                  积分 → ω_ref += ẍ_ref / Rw · Δt
                              ↓
   转向环 [P(gz)] → ω_diff  →  左轮 = ω_ref + ω_diff
                                右轮 = ω_ref − ω_diff
                              ↓
                   [电机速度环 PI] → PWM 占空比
```

### 3.2 三个关键细节（也是这套代码最值得讲的地方）

**① 轮子转动会"伪造"车体速度，必须补偿**

编码器测到的轮速里，混有"车体绕轮轴倾倒"带来的分量。若不扣除，速度环会把"前倾"误判为"前进"，形成正反馈。

```c
float omega2 = -theta_dot * (Lp + Rw) / Rw;   // 倾倒引起的等效轮速
float omega1 = omega - omega2;                // 真实质心平动
float x_dot  = omega1 * Rw;                   // 线速度 m/s
```

**② 用 `atan` 而不是线性近似**

线性化 `a = g·θ` 只在小角度成立；这里直接 `θ_ref = atan(a/g)`，大角度下依然准确。

```c
float theta_ref = qatan(PID_Compute(&pid_velocity, x_dot) / g);
```

**③ Δt 用硬件定时器实测，不假设固定周期**

主循环 1 ms / 5 ms 时间片并不严格，PID 内部靠 `GetUs()` 实测 Δt：

```c
float deltaT = (t_k - PID->t_k_1) * 1.0e-6f;
err_dev = (err - err_k_1) / deltaT;
err_int = err_int_k_1 + (err + err_k_1) * deltaT * 0.5f;   // 梯形法积分
```

**物理参数**：`Lp = 0.062 m`（质心高度）、`Rw = 0.032 m`（轮半径）、`g = 9.8`。

---

## 四、工程实现要点

1. **PID 结构**（`my_lib/pid.c`）：位置式 PID + **梯形法积分** + 输出限幅 + **积分限幅（与输出限幅分开配置，量纲正确）** + 实测 Δt。
2. **T 法测速**（`app_encoder.c`）：记录相邻 A 相脉冲的时间戳，`ω = Δθ/Δt`；换向瞬间把方向标记为 `±2` 并返回 0，避免除零和符号抖动。
3. **临界区快照**：中断里更新的多字节变量（`direction/t0/t1`）读取前 `__disable_irq()` 打包拷贝，再 `__enable_irq()`。
4. **ADC 硬件触发**：TIM2 TRGO → ADC 注入通道，中断里读 JDR，不占 CPU 轮询。
5. **串口三缓冲**：中断写 `intBuf` → 收满一行拷到 `transBuf` → 主循环在 `procBuf` 解析，避免中断与主循环竞争。
6. **协作式调度**：`PERIODIC(T)` 宏用 `GetTick()` 做软定时，替代 RTOS。

---

## 五、目录结构

```
balance_car/
├─ user/                  应用层
│  ├─ main.c              初始化 + 主循环
│  ├─ app_control.c       ★ 四环串级 + 倒立摆逆解算
│  ├─ app_mpu6050.c       姿态读取 + 互补滤波
│  ├─ app_encoder.c       ★ T 法测速（EXTI）
│  ├─ app_motor.c         电机速度环 PI
│  ├─ app_bat.c           电池 ADC（注入通道 + 定时器触发）
│  ├─ app_rc.c            串口遥控协议
│  ├─ app_pwm.c / app_button.c / app_usart2.c
│  └─ stm32f10x_it.c      中断向量
├─ my_lib/                驱动与工具库
│  ├─ pid.c               PID 控制器（梯形积分 + 抗饱和）
│  ├─ qmath.c             自实现三角函数（替代 libm）
│  ├─ i2c.c / si2c.c      硬件 / 软件 I2C
│  ├─ oled.c / oled_font.h   （字模由 ttf2c 生成，源字体不入库）
│  ├─ spi.c / usart.c / delay.c / button.c / task.h
├─ test/                  分模块自测程序
│  ├─ mpu6050_test.c      陀螺仪/加速度计/欧拉角验证
│  ├─ encoder_test.c      编码器测速验证
│  ├─ pwm_test.c          电机 PWM 验证
│  ├─ bat_test.c          电池采样验证
│  └─ qmath_test.c        三角函数精度验证
├─ startup/               CMSIS 启动文件
├─ std_periph_driver/     ST 标准外设库
└─ balance_car.uvprojx    Keil MDK 工程
```

---

## 六、编译与烧录

1. **环境**：Keil MDK 5 + STM32F1xx 器件支持包；仿真器 ST-Link（SWD）
2. 打开 `balance_car.uvprojx` → Build（F7）→ Download（F8）
3. **清理编译产物**：双击 `tools/clean.bat`（等价 Keil 的 Project → Clean + 清掉 IDE 状态文件）
4. **单独验证某个硬件模块**：在 `main.c` 里取消对应测试的注释，注释掉 `App_*_Init()`：
   ```c
   //QMath_Test();          // 三角函数精度
   //MPU6050_EularAngleTest();  // 姿态角输出
   //Encoder_T_Method_Test();   // 编码器测速
   //PWM_Test();                // 电机 PWM
   ```

---

## 七、代码走查与修复记录（6 项）

交付前对代码做了一轮 review，发现并修复以下问题。修复后重新编译链接验证：

```
编译 49/49 个源文件通过，0 错误
ROM 占用 28,580 B / 65,536 B (43.6%)
初始 SP 0x200012E0（20 KB RAM 内）   Reset_Handler 0x0800403D   向量表 0x08000000
```

| # | 位置 | 问题 | 修复 |
|---|---|---|---|
| 1 | `app_motor.c` | 右轮 PID 误用了**左轮**的控制器与反馈：`PID_Compute(&pid_motor_l, omega_l)` | 改为 `pid_motor_r` / `omega_r`，左右轮各自独立闭环 |
| 2 | `app_encoder.c`（EXTI3） | 换向判断写成了 `direction_l` | 改为 `direction_r` |
| 3 | `app_control.c` | 常量 `Pi = 3.1435927f` 小数位打错 | 改为 `3.1415927f` |
| 4 | `my_lib/pid.c/h` | 积分限幅复用了**输出限幅**的值，两者量纲不同 | 新增 `IntUpperLimit/IntLowerLimit` 与 `PID_IntLimitConfig()`；默认仍随输出限幅同步，**不改变现有手感** |
| 5 | `app_mpu6050.c` | 逐寄存器读 14 次 I2C 事务 | 改为从 `0x3B` **burst read** 14 字节，1 次事务读完 |
| 6 | `my_lib/str_cmd.c/h` | 空文件且无任何引用 | 删除 |

**修复 1 的实际影响**：原代码右轮没有独立闭环，且左轮 PID 每周期被计算两次（积分累积翻倍）；
修正后**转向响应明显改善**。

**修复 5 的实际影响**：姿态读取的 I2C 事务从 14 次降到 1 次，采样抖动减小。
⚠️ 该项改动涉及 I²C 读时序，**建议先单独跑 `MPU6050_EularAngleTest()` 确认姿态角正常，再接电机**。
## 八、可继续改进的方向

- **姿态解算升级**：互补滤波 → Mahony 四元数（消除大角度下的耦合误差），或卡尔曼滤波
- **控制算法对比**：现有非线性逆解算 vs LQR vs MPC，量化对比倾角恢复时间与抗扰能力
- **无刷/轮毂电机 + FOC**：提升效率与静音性
- **速度观测器**：用编码器 + IMU 做融合，替代纯差分测速
- **ROS 2 接入**：串口输出 `/odom` + `/imu`，接入 Nav2

---

## 九、参考

- 公开平衡车教学项目的框架组织方式：分模块分层、`PERIODIC` 时间片调度宏、`qmath` 定点数学库
- MPU6050 / MPU6500 数据手册：寄存器配置与量程换算
- STM32F10x 标准外设库
> **仓库说明**
> - `my_lib/font/ttfttc/` 下的中文字体（约 45 MB）**不入库**——它们只用来生成 OLED 字模，
>   生成结果 `oled_font.h` 已经在仓库里。需要重新生成字模时，把 TTF 放回该目录，用 `my_lib/font/ttf2c.exe` 转换。
> - `*.uvoptx` 是 Keil 的**用户界面/调试状态**文件（窗口布局、断点），每台电脑不同，不入库；
>   工程文件是 `balance_car.uvprojx`，这个必须保留。
