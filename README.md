# 基于 RK3506 的 EtherCAT 三轴云台实时运动控制系统

## 项目简介

本项目基于 **RK3506** 芯片构建实时运动控制系统，运行 **Linux PREEMPT_RT** 实时内核，通过 **SOEM** 开源协议栈与 **EtherCAT** 工业总线，实现三轴伺服云台的高速实时通信与控制。

系统以 **MPU6050** 六轴 IMU 通过 I2C 采集姿态数据，结合**动态互补滤波**与 **PID 闭环控制**，实时解算三轴位姿并驱动伺服补偿，在振动环境下自动维持锁定姿态。通过 EtherCAT **DC 分布式时钟同步**与 **CPU 内核隔离**，将通信周期缩短至 4ms，cyclictest 验证最大调度抖动 ≤237µs（峰值降低 88%），满足工业级实时要求。

配套 **WiFi 遥测模块**（ESP8266）支持手机端实时监控与远程模式切换，适用于医疗救护、海上作业、车载稳定等高精度姿态保持场景。

开发采用 **WSL2 交叉编译 + SSH 远程部署**，实现全链路自动化。

---

## 硬件清单

| 硬件模块 | 型号规格 | 关键参数 |
|----------|----------|----------|
| 主控芯片 | **RK3506** | 三核 Cortex-A7，1.2GHz |
| 姿态传感器 | **MPU6050** | 加速度 ±2g / 陀螺仪 ±250°/s，I2C 接口 |
| 无线模块 | **ESP8266** | UART 115200bps，UDP 透传 |
| 伺服驱动器 | **汇川 SV630N** | EtherCAT CoE，CSP / PT / PP 多模式 |
| 伺服电机 | **MS1H4-40B30CB** | 400W，3000rpm，1.27Nm，18bit 绝对值编码器 |
| 丝杠导程 | 5mm/圈 | 电子齿轮比 1000 指令单位 = 1 圈 |
| 云台铰点半径 | R = 173mm | 三轴 120° 等边三角形布局 |
| 电缸行程 | 0 ~ 250mm | 机械中位 125mm，最大倾角 ±12° |
| 网络 | eth0 (EtherCAT) + USB 网卡 (SSH) | 双网口隔离 |

---

## 项目目录结构

```
rk3506-project/
├── src/
│   ├── main.c                 # 系统入口，线程创建与 CPU 绑核
│   ├── common.h               # 全局类型定义、硬件常量
│   ├── shared_data.h/c        # 线程间共享数据区（原子变量/互斥锁/信号量）
│   ├── ethercat_config.c/h    # SOEM 初始化、PDO 映射、DC 配置、电子齿轮比
│   ├── ethercat_thread.c/h    # EtherCAT 实时主循环、CiA402 状态机
│   ├── control_thread.c/h     # 运动控制线程（4ms 周期）
│   ├── motion_control.c/h     # 运动控制顶层调度
│   ├── modes.c/h              # 四种工作模式状态机
│   ├── kinematics.c/h         # 三轴逆运动学解算
│   ├── homing.c/h             # 扭矩堵转回零状态机
│   ├── sensor_thread.c/h      # IMU 传感器采集（1kHz）+ 动态互补滤波
│   ├── mpu6050.c/h            # MPU6050 I2C 底层驱动
│   ├── uart_thread.c/h        # UART 通信线程（ESP8266）
│   ├── json_parser.c/h        # JSON 指令解析与遥测打包
│   ├── ring_buffer.c/h        # 无锁环形缓冲区（SPSC）
├── Makefile                   # 交叉编译 Makefile
├── README.md                  # 本文件
```

---

## 软件架构

### 多线程实时调度

| 线程 | 绑定 CPU | 实时优先级 | 周期 | 核心任务 |
|------|----------|------------|------|----------|
| EtherCAT 主站 | CPU1 | 99 (SCHED_FIFO) | 4ms | PDO 收发、DC 同步、CiA402 状态机 |
| 运动控制 | CPU1 | 98 (SCHED_FIFO) | 4ms | 逆运动学、模式切换、回零调度 |
| IMU 传感器 | CPU0 | 97 (SCHED_FIFO) | 1ms | MPU6050 读取、动态互补滤波 |
| UART 通信 | 无绑定 | 0 (普通) | 事件触发 | JSON 解析、遥测下发 |

### 数据流

```
手机 APP ←WiFi UDP→ ESP8266 ←UART→ json_parser ←共享数据区→ 运动控制线程
                                                        │
                                              kinematics_solve()
                                                        │
                                              servo_cmd_t → EtherCAT PDO
                                                        │
                                              伺服驱动器 → 电机执行
```

---

## 四种工作模式

| 模式 | 说明 | 控制方式 |
|------|------|----------|
| **手动模式** (`manual`) | 手机直控 Roll / Pitch / Z | 目标姿态 → 逆运动学 → CSP 位置闭环 |
| **平衡模式** (`balance`) | 锁定当前姿态，PID 自动修正 | 传感器反馈 + PID Z 补偿 → 逆运动学 |
| **震动模式** (`vibration`) | Z 轴正弦升降，Roll/Pitch 保持水平 | `Z(t) = center_z + amp × sin(2π·freq·t)` |
| **翻滚模式** (`roll`) | Roll+Pitch 联动 360° 倾斜旋转 | `roll = -tilt×cos(θ)`, `pitch = -tilt×sin(θ)` |

---

## 环境搭建

### 开发环境

| 组件 | 版本/说明 |
|------|-----------|
| 宿主机 | Windows 11 + WSL2 |
| 交叉编译器 | `arm-buildroot-linux-gnueabihf-gcc` 12.4.0 |
| 工具链路径 | `~/3506-toolchain/host/bin/` |
| SOEM 库 | v1.3.1，静态编译 (`~/soem-1.3.1/`) |
| 目标系统 | Buildroot Linux 6.8 + PREEMPT_RT |
| Android Studio | 用于手机 APP 开发 |

### 环境变量

```bash
export PATH=~/3506-toolchain/host/bin:$PATH
export CC=arm-buildroot-linux-gnueabihf-gcc
```

---

## 编译步骤

```bash
# 1. 进入项目目录
cd ~/rk3506-project

# 2. 清理旧构建
make clean

# 3. 编译
make

# 4. 产物
# gimbal_control  (ARM 32-bit ELF, statically linked)
```

---

## 部署运行

```bash
# 1. 停止开发板上旧进程
ssh root@192.168.1.100 "killall gimbal_control; rm -f /tmp/gimbal_control"

# 2. 上传新固件
scp gimbal_control root@192.168.1.100:/tmp/

# 3. 远程运行
ssh root@192.168.1.100 "chmod +x /tmp/gimbal_control && /tmp/gimbal_control"
```

---

## 实时性能测试

### cyclictest 抖动测试

测试条件：PREEMPT_RT 内核 + CPU 绑核（EtherCAT/运动控制 → CPU1，IMU → CPU0）

| 测试指标 | 绑核方案 | 未绑核方案 | 优化幅度 |
|----------|----------|------------|----------|
| 最小抖动 | 4 µs | 4 µs | — |
| 平均抖动 | **74 µs** | 91 µs | ↓ 18% |
| 最大抖动（峰值） | **237 µs** | 1918 µs | ↓ **88%** |
| 占控制周期比 | 1.85% (74/4000) | 2.28% (91/4000) | — |
| 峰值占比 | 5.93% (237/4000) | 47.95% (1918/4000) | — |

> 结论：绑核后峰值抖动 237µs，仅占 4ms 控制周期的 5.93%，远低于工业伺服通常要求的 20% 安全阈值。

### 关键实时性保障措施

- `SCHED_FIFO` 实时调度 + RT 优先级 97~99
- `pthread_attr_setaffinity_np` CPU 亲和性绑定
- `clock_nanosleep(TIMER_ABSTIME)` 绝对时钟精确定时
- WKC（工作计数器）校验失败自动丢弃脏数据帧
- EtherCAT 总线 IRQ 与实时线程同核部署（避免跨核中断延迟）

---

## 核心技术要点

### 1. CiA402 伺服状态机

```
Init → Pre-Op → Safe-Op → Op
                            │
                    6 (Switch On Disabled)
                     → 7 (Ready to Switch On)
                     → 0xF (Operation Enabled)  ← CSP 位置闭环
```

- 每步状态切换后验证从站实际状态码，异常时返回 -1 中止
- 安全停机序列：0x000F → 0x0007 → 0x0006

### 2. 逆运动学

```
z_i = current_z_base + height_offset + balance_z_comp
    + R × (θ_plat × cosα_i + φ_plat × sinα_i)
```

- 基准为 `current_z_base`（当前高度），非固定中位——任意高度下倾斜补偿不失准
- 数学证明：三轴 Z 坐标均值 = 平台中心高度（`cos0°+cos120°+cos240° = 0`）

### 3. 扭矩堵转回零

| 阶段 | 操作 | 参数 |
|------|------|------|
| 1 | 切换 PT 力矩模式 | 力矩给定 20% 额定 |
| 2 | 堵转采样 | 速度 <2000 且 扭矩 >2% |
| 3 | 误判滤波 | 40 次连续采样累加器 |
| 4 | 静置缓冲 | 250 周期 = 1s |
| 5 | 写零点 | 0x607C = -current_pos |
| 6 | 恢复 CSP | CSP PDO 映射复原 |

### 4. 动态互补滤波

```
roll = ALPHA × (roll + gyro × dt) + (1-ALPHA) × roll_acc
```

- `ALPHA` 动态范围：0.94（静止）~ 0.99（剧烈振动）
- 振动量估算：`level = 0.95×历史 + 0.05×|acc_norm - 1g|`
- 系数平滑过渡速率：0.005/s，避免角度跳变

---

## Android 手机 APP

| 页面 | 功能 |
|------|------|
| `ManualControlFragment` | Roll/Pitch/Z 滑杆直控 |
| `BalanceFragment` | 一键锁定当前姿态，PID 自动保持 |
| `VibrateFragment` | 频率/幅度调节，Z 轴正弦震动 |
| `RollContinuousFragment` | 速度/倾斜幅度调节，360° 双轴翻滚 |

- 协议：JSON over UDP (`{"mode":"balance","lock":true}`)
- 遥测刷新率：30Hz（33ms 间隔）
- APP 源码：`MyStewartControl/`

---

## 关键依赖

- [SOEM](https://github.com/OpenEtherCATsociety/SOEM) — Simple Open EtherCAT Master v1.3.1
- Linux Kernel 6.8 + PREEMPT_RT patch
- Buildroot 交叉编译工具链 (arm-buildroot-linux-gnueabihf)
- 汇川 SV630N 伺服驱动器 (EtherCAT CoE)
- MPU6050 I2C 驱动
- ESP8266 AT 固件 (UDP 透传模式)

---

## 参考文献

1. EtherCAT Technology Group, *EtherCAT Specification*, 2020
2. Beckhoff, *ETG.6010 – CiA402 Drive Profile for EtherCAT*, 2019
3. CAN in Automation, *CiA 402 – CANopen Device Profile for Drives*, 2018
4. RT-Labs, *SOEM Library Documentation*, 2024
5. Linux Foundation, *PREEMPT_RT Wiki*, 2024
6. Rockchip, *RK3506 Technical Reference Manual*, 2024
7. Invensense, *MPU-6050 Product Specification*, 2013

---

## License

MIT License — 仅供学习与竞赛使用。
