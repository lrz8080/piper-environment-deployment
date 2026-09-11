# Piper X 机械臂控制说明文档

## 概述

本项目提供两种方式控制 AgileX Piper X 六自由度机械臂：

| 控制方式 | 控制空间 | 适用场景 |
|---------|---------|---------|
| 键盘控制 | 关节空间 / 笛卡尔空间 | 调试、精确点位控制 |
| Pico 4 VR 遥操作 | 关节空间 / 笛卡尔空间 | 数据采集、示教、直觉操作 |

---

## 一、键盘控制

### 1.1 关节空间控制 (`piper_ctrl_moveJ_keyboard.py`)

直接控制 6 个关节角度，支持短按精确控制和长按连续运动。

**运行方式：**
```bash
python3 piper_sdk/piper_ctrl_moveJ_keyboard.py
```

**按键映射：**

| 按键 | 功能 | 按键 | 功能 |
|------|------|------|------|
| Q | 关节0 +1° (底座顺转) | A | 关节0 -1° (底座逆转) |
| W | 关节1 +1° (大臂上抬) | S | 关节1 -1° (大臂下放) |
| E | 关节2 +1° (小臂) | D | 关节2 -1° (小臂) |
| R | 关节3 +1° (腕部1) | F | 关节3 -1° (腕部1) |
| T | 关节4 +1° (腕部2) | G | 关节4 -1° (腕部2) |
| Y | 关节5 +1° (腕部3) | H | 关节5 -1° (腕部3) |
| Z | 夹爪 +5mm | X | 夹爪 -5mm |
| 空格 | 夹爪 开/关切换 | | |

**功能键：**

| 按键 | 功能 |
|------|------|
| 1 | 归零位姿 |
| 2 | Home 位姿 |
| 3 | 抓取位姿 |
| 0 | 紧急归零 |
| ESC | 退出 |

**长按机制：** 按住控制键超过 0.3 秒进入连续运动模式（15Hz），松开立即停止。短按保持精确 ±1° 步进。

**关节限位：**

| 关节 | 范围 |
|------|------|
| J0 | -150° ~ +150° |
| J1 | 0° ~ +180° |
| J2 | -170° ~ 0° |
| J3 | -100° ~ +100° |
| J4 | -70° ~ +70° |
| J5 | -120° ~ +120° |

**依赖：** `piper_sdk`, `pynput`

---

### 1.2 笛卡尔空间控制 (`piper_ctrl_moveP_keyboard.py`)

控制机械臂末端在笛卡尔空间的位姿（XYZ + RPY），使用机械臂内置逆运动学（IK）解算关节角。

**运行方式：**
```bash
python3 piper_sdk/piper_ctrl_moveP_keyboard.py
```

**按键映射：**

| 按键 | 功能 | 按键 | 功能 |
|------|------|------|------|
| W | X+ (末端前移) | S | X- (末端后移) |
| A | Y+ (末端左移) | D | Y- (末端右移) |
| Q | Z+ (末端上升) | E | Z- (末端下降) |
| I | RX+ | K | RX- |
| J | RY+ | L | RY- |
| U | RZ+ | O | RZ- |
| 空格 | 夹爪 开/关 | +/- | 夹爪微调 |
| R | 复位初始位姿 | ESC | 退出 |
| ↑/↓ | 调整步进速度 | | |

**控制参数：**
- XYZ 步进：5mm（可按 ↑/↓ 在 1-50mm 范围调整）
- 旋转步进：5°（可按 ↑/↓ 在 1-20° 范围调整）
- 控制频率：50Hz
- 内置 IK：机械臂内置 `EndPoseCtrl` 将笛卡尔目标解算为关节角

**工作空间限位：**

| 轴 | 范围 |
|----|------|
| X | -100 ~ 500 mm |
| Y | -300 ~ 300 mm |
| Z | 0 ~ 500 mm |
| RX | -180° ~ +180° |
| RY | -100° ~ +112° |
| RZ | -75° ~ +75° |

**依赖：** `piper_sdk`, `pynput`

---

## 二、Pico 4 VR 遥操作控制

整体架构：Pico 4 浏览器 → MQTT Broker（云端）→ 本机 Python → Piper SDK → 机械臂。

```
Pico 4 (浏览器 ao2car.ddt.dev/vr)
    │  WebXR 读取手柄 6-DOF 位姿
    │  MQTT over WebSocket
    ▼
MQTT Broker (120.79.156.21:8083/mqtt)
    │  订阅 arm0001/controller, arm0001/controller_data
    ▼
本机 Python 遥操作程序
    │  Piper SDK (CAN 总线)
    ▼
Piper X 机械臂
```

### 2.1 启动前准备

1. Pico 4 浏览器打开 `https://ao2car.ddt.dev/vr/` 并完成手柄标定
2. 确保控制 PC 与 Pico 4 在同一网络（或能访问公网 MQTT Broker）
3. 确保 CAN 总线已激活：`sudo bash piper_sdk/can_activate.sh`

### 2.2 关节空间遥操作 (`pico_piper_joint_teleop.py`)

将 VR 手柄的位移/旋转直接映射到 6 个关节增量。

**运行方式：**
```bash
python3 example/teleop/pico_piper_joint_teleop.py --episodes 1 
```

**映射关系：**

| VR 手柄动作 | 机械臂关节 | 缩放比例 |
|------------|-----------|---------|
| X 轴位移 | 关节1 (底座) | -20:1 |
| Y 轴位移 | 关节2 (大臂) | 20:1 |
| Z 轴位移 | 关节3 (肘部) | -20:1 |
| Pitch (RX) | 关节4 | 20:1 |
| Roll (RY) | 关节5 | -10:1 |
| Yaw (RZ) | 关节6 | 10:1 |
| 扳机 | 夹爪 | 直接控制 |

**控制参数：**
- 控制频率：200Hz
- 死区：0.08 rad
- 握把按下 → 移动/旋转手柄控制关节，松开保持

**命令行参数：**
```
--episodes N  采集轨迹数 (默认 50)
--freq N      控制频率 Hz (默认 200)
```

---

### 2.3 笛卡尔空间遥操作 (`pico_piper_teleop.py`)

VR 手柄绝对位姿映射到末端笛卡尔位姿（绝对映射，无累积误差）。

**运行方式：**
```bash
python3 example/teleop/pico_piper_teleop.py --episodes 1
```

**映射公式：**
```
target_pos = ref_arm_pos + (vr_pos - ref_vr_pos) * scale
target_rpy = ref_arm_rpy + (vr_rot - ref_vr_rot) * scale
```

握把按下时捕获参考帧（当前 VR 位姿 + 机械臂末端位姿），握持期间目标位姿每帧独立从参考帧计算，无累积漂移。

**控制参数：**
- 位置缩放：1.0
- 旋转缩放：1.0
- 速度限幅：0.5 m/s（线速度）、1.0 rad/s（角速度）
- EMA 平滑：位置 τ=0.08s，姿态 τ=0.12s
- 死区：位置 1mm，姿态 0.015rad
- 控制频率：100Hz

**三层平滑体系：**
1. **死区** — 微小位移/旋转忽略，消除手部抖动
2. **EMA 滤波** — 对目标位姿做指数平滑，消除高频噪声
3. **速度限幅** — 限制相邻帧变化量，保证运动平滑安全

**握把安全锁：** 握把按下时才驱动机械臂，松开立即保持，是 Dead Man's Switch 安全机制。

---

### 2.4 笛卡尔空间遥操作 + 轴映射 (`pico_piper_ik_teleop.py`)（需要两个相机）

与 `pico_piper_teleop.py` 类似，但增加了：
- **坐标系轴映射矩阵** — 可独立控制 VR 每个轴到机械臂每个轴的映射方向和对应关系
- **旋转增量用旋转矩阵计算** (axis × angle)，避免 RPY 直接相减的方向节锁问题
- 更强的诊断输出

**运行方式：**
```bash
python3 example/teleop/pico_piper_ik_teleop.py --episodes 50
```

**坐标系映射矩阵：**

VR 坐标系（WebXR）：X=右, Y=上, Z=-前  
机械臂坐标系：X=前, Y=左, Z=上

位置映射矩阵：
```
VR_TO_ROBOT_POS = [[ 0, 1, 0],   # Robot X(前) ← VR 上(+Y)
                   [-1, 0, 0],   # Robot Y(左) ← VR -右
                   [ 0, 0, 1]]   # Robot Z(上) ← VR 上(+Z)
```

旋转映射矩阵：
```
VR_TO_ROBOT_ROT = [[ 1, 0, 0],
                   [ 0,-1, 0],
                   [ 0, 0,-1]]
```

如某个方向运动方向反了，把对应矩阵中的数字取反即可。

**控制参数：**
- 位置/旋转缩放：1.2
- 速度限幅：线速度 4.0 m/s，角速度 8.0 rad/s
- EMA 平滑：位置 τ=0.02s，姿态 τ=0.03s
- 死区：位置 2mm，姿态 0.02rad
- 控制频率：100Hz

---

### 2.5 笛卡尔空间遥操作 + 轴映射（无相机版）(`pico_piper_ik_teleop2.py`)

与 `pico_piper_ik_teleop.py` 功能完全相同，但关闭了 RealSense 相机连接和数据采集，仅采集机械臂关节/位姿/夹爪数据。

**运行方式：**
```bash
python3 example/teleop/pico_piper_ik_teleop2.py --episodes 50
```

---

## 三、MQTT 配置

所有遥操作程序共用同一 MQTT Broker：

| 配置项 | 值 |
|-------|-----|
| Broker 地址 | 120.79.156.21 |
| 端口 | 8083 |
| 路径 | /mqtt |
| 用户名 | arm_server |
| 密码 | serverarm |
| 协议 | MQTT v3.1.1 over WebSocket |

VR 手柄数据 Topic：

| Topic | 内容 |
|-------|------|
| arm0001/controller | 18 元素数组：左臂(0-8) + 右臂(9-17) 位姿/夹爪 |
| arm0001/controller_data | 按钮/扳机/握把状态 |

右手柄数据格式（controller[9:18]）：
```
[x, y, z, w, dx, dy, dz, isMove, clamp]
 位置(xyz)  四元数(xyzw)  握把  夹爪
```

---

## 四、数据采集

所有遥操作程序均支持将控制过程中的机械臂状态保存为 HDF5 格式。

采集的内容包括：
- 关节角度 (joint)
- 末端位姿 (qpos: XYZ + RPY)
- 夹爪状态 (gripper)
- （可选）相机图像 (color)

数据保存路径：`./datasets/{task_name}/`，每个 episode 一个 `.hdf5` 文件。

---

## 五、文件索引

```
piper_sdk/
├── piper_ctrl_moveJ_keyboard.py    # 键盘 - 关节空间
└── piper_ctrl_moveP_keyboard.py    # 键盘 - 笛卡尔空间

example/teleop/
├── pico_piper_joint_teleop.py      # Pico VR - 关节空间
├── pico_piper_teleop.py            # Pico VR - 笛卡尔 (绝对映射)
├── pico_piper_ik_teleop.py         # Pico VR - 笛卡尔 (轴映射+IK)
└── pico_piper_ik_teleop2.py        # Pico VR - 笛卡尔 (轴映射+IK, 无相机)
```

---

## 六、BETAFPV/TITA 遥控器控制

`example/teleop/betafpv_piper_teleop.py` 参照 `DDTRobot/airbot_joy` 的 ROS 2 Joy
轴映射和 `piper_ctrl_moveJ_keyboard_ssh.py` 的反馈初始化、关节限位及控制接口，
直接控制 Piper，不依赖 RealSense 或 MQTT，并增加完整的末端 XYZ + RPY 控制。

完整实机测试流程见 [`BETAFPV_PIPER_TEST_README.md`](BETAFPV_PIPER_TEST_README.md)。

### 模式映射

| 右侧三段开关 | 右扳机 | 功能 |
|---|---|---|
| 下 | 松开 | 末端笛卡尔位置 XYZ |
| 下 | 按下 | 夹爪全开/全闭切换；保持按下时可用第 4 摇杆轴微调 |
| 中 | 松开 | 关节 1/2/3 |
| 中 | 按下 | 关节 4/5/6 |
| 上 | 松开 | 末端笛卡尔姿态 RX/RY/RZ |
| 上 | 按下并保持 0.8 秒 | 回零 |

### 运行

推荐从项目根目录一键启动：

```bash
./start_piper_teleop.sh
```

首次测试可使用 `./start_piper_teleop.sh --dry-run`。脚本会自动加载 ROS 2、D1
namespace，并使用对应 Joy Topic、`raw` 输入格式和 `can1`；LIVE 模式下会检查
`can1`，必要时请求 sudo 密码并配置为 1Mbps。其他命令行参数可直接追加。

手动启动方式：

```bash
cd piper_control
source /opt/ros/humble/setup.bash
source /opt/d1_ros2/namespace.sh

# 先确认 D1 发布的 Joy topic
ros2 topic list | grep joy

# 不连接机械臂，先检查映射
python3 example/teleop/betafpv_piper_teleop.py \
  --joy-topic "/${ROBOT_NS}/joy" --input-format raw --dry-run

# Piper 实机位于 can1
python3 example/teleop/betafpv_piper_teleop.py \
  --joy-topic "/${ROBOT_NS}/joy" --input-format raw --can can1
```

如果已经运行原仓库的 `airbot_joy_node`，使用其重映射输出：

```bash
python3 example/teleop/betafpv_piper_teleop.py \
  --joy-topic /airbot_play/joy --input-format airbot --can can1
```

运行前必须在遥控器菜单选择 `use-sdk mode`。程序默认包含 8% 死区及关节/
工作空间/姿态限位，并将线速度限制为 0.1m/s、角速度和关节速度限制为
0.5rad/s。控制由新到达的 Joy 消息驱动，停止发布时不会重放最后一条指令。
程序同时监控 `teleop_command.use_sdk`：只有该参数为 `true` 且收到格式正确的
Joy 消息后才会连接并使能 Piper；运行中变为 `false` 会立即暂停发送新目标。
首次联调应先使用 `--dry-run`。
