# 基于 RK3506 的 EtherCAT 三轴云台实时运动控制系统

## 简介

本项目基于 **RK3506**（三核 Cortex-A7，1.2GHz）+ **Linux PREEMPT_RT** 实时内核，通过 **SOEM** 开源协议栈自研 EtherCAT 主站，实现三轴伺服云台的 4ms 实时同步控制。

系统以 **MPU6050** 六轴 IMU 采集姿态，经**动态互补滤波**与 **PID 闭环**实时解算三轴位姿，驱动三台汇川 SV630N 伺服（400W）完成补偿。通过 **CPU 内核隔离 + SCHED_FIFO 实时调度**，cyclictest 实测最大抖动 ≤237µs（峰值降低 88%）。

配套 ESP8266 WiFi 遥测 + Android APP，支持手动操控、自平衡锁定、震动测试、360° 翻滚四种模式。

硬件：RK3506 + 三台汇川 SV630N 伺服 + MS1H4-40B30CB 电机（400W/1.27Nm/18bit 绝对值编码器）+ MPU6050 + ESP8266。三轴 120° 等边并联云台，铰点半径 R=173mm，行程 0~250mm。

---

## 工程代码文件说明

### 入口与全局定义

| 文件 | 职责 |
|------|------|
| `main.c` | 系统入口：初始化共享数据区 → 配置 EtherCAT → 创建 4 个 RT 线程（Sensor/UART/EtherCAT/Control）并绑核 → 等 Ctrl+C → 优雅退出 |
| `common.h` | 全局常量与类型定义（硬件参数、枚举、结构体），所有 .c 文件共用 |

### 共享数据区

| 文件 | 职责 |
|------|------|
| `shared_data.h` | 线程间共享数据结构声明：原子变量（模式切换）、互斥锁（目标姿态/震动参数/翻滚参数）、信号量（EtherCAT→Control 同步）、自旋锁（伺服反馈） |
| `shared_data.c` | 共享数据区初始化，设置默认模式为手动 |

### EtherCAT 协议栈

| 文件 | 职责 |
|------|------|
| `ethercat_config.h` | SOEM 主站配置头文件：声明初始化、PDO 映射、DC 时钟同步、状态切换等接口 |
| `ethercat_config.c` | **SOEM 初始化核心**：ec_init 初始化总线 → 扫描从站 → 21 条 SDO 配置 PDO 映射（RxPDO 6 路 + TxPDO 5 路）→ 电子齿轮比 96:1 → 力矩斜率 1000ms → DC Sync0 同步 → SafeOp→OP |
| `ethercat_thread.h` | EtherCAT 主站线程头文件 |
| `ethercat_thread.c` | **最高优先级 RT 线程**（RT 99, CPU1）：绑定 PDO 指针 → 等待 OP → CiA402 启动（控制字 6→7→0xF）→ **4ms 主循环**收发 PDO → WKC 校验 → sem_post 唤醒控制线程 |

### 运动控制

| 文件 | 职责 |
|------|------|
| `kinematics.h` | 逆运动学头文件：定义机械参数（R=190mm、Z 行程 0~100mm、±15°限幅）和求解函数签名 |
| `kinematics.c` | **三轴逆运动学解算**：输入传感器 Roll/Pitch → 平台反向补偿 → 合成倾角限幅 → 三铰点 Z 位移公式 `z_i = z_base + R×(θ×cosα_i + φ×sinα_i)` → 输出限幅。数学证明三轴均值 = 中心高度 |
| `modes.h` | 四种工作模式头文件：手动/平衡/震动/翻滚 + 回零/回中/冻结/水平接口 |
| `modes.c` | **四模式状态机**：手动（手机直控 Roll/Pitch/Z）→ 平衡（锁姿态 + PID Z 补偿）→ 震动（Z 轴正弦升降）→ 翻滚（Roll+Pitch 联动 360° 旋转），含回中过渡逻辑 |
| `motion_control.h` | 运动控制顶层调度头文件 |
| `motion_control.c` | **运动控制顶层**：包装 modes.c 的执行函数，设置 CiA402 控制字和工作模式，管理回零/低功耗/急停等全局状态。低功耗状态机：ARMED → ACTIVE（无力矩倒计时）→ 超时自动回零 |
| `control_thread.h` | 运动控制线程头文件 |
| `control_thread.c` | **第二优先级 RT 线程**（RT 98, CPU1）：sem_wait 等 EtherCAT 线程信号量 → 读传感器姿态 → 调用 motion_control_step → 写伺服指令到共享区 |

### 传感器与滤波

| 文件 | 职责 |
|------|------|
| `mpu6050.h` | MPU6050 I2C 驱动头文件 |
| `mpu6050.c` | **MPU6050 I2C 底层驱动**：open I2C-2 总线 → ioctl 设从机地址 0x68 → 读写寄存器（0x3B 加速度/0x43 陀螺仪）→ 16 位大端解析 → 换算物理量 |
| `sensor_thread.h` | 传感器采集线程头文件 |
| `sensor_thread.c` | **IMU 采集 RT 线程**（RT 97, CPU0）：3s 陀螺仪偏置校准 → **1kHz 绝对时钟精确定时** → 读 MPU6050 原始数据 → **动态互补滤波**（α 系数 0.94~0.99 自适应振动）→ 角度写入共享区 |

### 回零控制

| 文件 | 职责 |
|------|------|
| `homing.h` | 回零控制头文件 |
| `homing.c` | **CSP 慢速走位 + 扭矩堵转回零**：三缸以极慢速度递减目标位置 → 撞机械限位 → 监测实际扭矩骤升 → 40 次连续采样累加器防误判 → 堵转确认 → 静置 1s 消弹跳 → 写零点 → 全程 CSP 模式无需切换 PDO |

### 通信

| 文件 | 职责 |
|------|------|
| `uart_thread.h` | UART 通信线程头文件 |
| `uart_thread.c` | **UART 收发线程**（普通优先级，无绑核，事件驱动）：串口 115200/8N1 → **非阻塞** O_NONBLOCK 读 → JSON 字节流 → json_parse_and_dispatch 解析指令。每 33ms 打包遥测 JSON → 环形缓冲区 → write 发送 ESP8266 → 手机 UDP 接收 |
| `json_parser.h` | JSON 解析头文件 |
| `json_parser.c` | **简易 JSON 解析**：strstr 定位 key → strtof 提取浮点数 → 解析 mode/roll/pitch/z_pos/lock/freq/amp/speed/tilt 字段 → 更新共享数据区。json_build_telemetry 打包姿态/高度/模式为 JSON |
| `ring_buffer.h` | 环形缓冲区头文件 |
| `ring_buffer.c` | **无锁环形缓冲区**（SPSC 单生产者/单消费者）：16 槽 × 256 字节，atomic_int 保证 head/tail 原子操作，无需互斥锁 |

### 测试程序（一次性，用完即弃）

| 文件 | 职责 |
|------|------|
| `z_max_test.c` | **Z 轴最大脉冲独立测试**：三缸同步下降→堵转检测→设零点→同步上升→堵转检测→打印最大行程与推荐软限位。不依赖多线程框架 |
| `max_tilt_safe_test.c` | **最大倾角安全测试**：以 1mm/s 慢速推动电缸倾斜，实时监测伺服扭矩反馈，扭矩 >30% 额定立刻停止，读 MPU6050 倾角作为安全限幅上限 |

---

## 多线程实时架构

```
EtherCAT 线程 (RT 99, CPU1)     Control 线程 (RT 98, CPU1)
────────────────────────        ────────────────────────
4ms PDO 收发                    等待信号量 sem_wait
更新伺服反馈 fb                 ↓ 被唤醒
sem_post() ───────────→         读 fb + 传感器姿态
(继续下一轮)                    逆运动学 → 写伺服指令 cmd
```

| 线程 | CPU | 优先级 | 周期 | 任务 |
|------|-----|--------|------|------|
| EtherCAT 主站 | CPU1 | 99 (SCHED_FIFO) | 4ms | PDO 收发、DC 同步、CiA402 状态机 |
| 运动控制 | CPU1 | 98 (SCHED_FIFO) | 4ms | 逆运动学解算、模式切换 |
| IMU 传感器 | CPU0 | 97 (SCHED_FIFO) | 1ms | MPU6050 读取、互补滤波 |
| UART 通信 | 无 | 0 (普通) | 事件触发 | JSON 解析、遥测下发 |

---

## 四种工作模式

| 模式 | 触发方式 | 行为 |
|------|----------|------|
| **手动** | APP 滑杆 | Roll/Pitch/Z 目标 → 逆运动学 → CSP 位置闭环 |
| **自平衡** | APP 锁定开关 | 锁当前姿态为基准，PID 自动纠偏维持水平 |
| **震动** | APP 频率/幅度 | 回中 → Z 轴正弦 `Z(t)=center+amp×sin(2πft)` |
| **翻滚** | APP 速度/倾角 | 回中 → Roll+Pitch 联动 360° 倾斜旋转 |

---

## 开发环境

- **宿主机**：Windows 11 + WSL2
- **交叉编译器**：`arm-buildroot-linux-gnueabihf-gcc` 12.4.0
- **SOEM**：v1.3.1（静态编译）
- **目标系统**：Buildroot Linux 6.8 + PREEMPT_RT
- **APP**：Android Studio（Java, Fragment+ViewPager）

---

## 编译 & 运行

```bash
cd ~/rk3506-project
make clean && make
scp gimbal_control root@192.168.1.100:/tmp/
ssh root@192.168.1.100 "/tmp/gimbal_control"
```

---

## 核心参数

| 参数 | 值 |
|------|-----|
| EtherCAT 周期 | 4ms (250Hz) |
| IMU 采样率 | 1kHz |
| 绑核峰值抖动 | 237µs（未绑核 1918µs，降低 88%）|
| 互补滤波 α | 0.94（静止）→ 0.99（振动）|
| 电子齿轮比 | 96:1（65536 指令单位/圈）|
| 电缸行程 | 0~250mm（中位 125mm）|
| 最大倾角 | ±12° |
| 铰点半径 | R = 173mm |
| 回零方式 | CSP 慢速走位 + 扭矩堵转检测 + 40 次累加防抖 |
