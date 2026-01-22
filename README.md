# Team2_project


# Team2_project: Nao Soccer Simulation

This is a **Nao Robot Soccer** control system based on the **Webots** simulation environment. The project adopts a **Centralized Decision-Making (Supervisor)** + **Distributed Execution (Player)** architecture, achieving multi-robot collaboration via UDP communication.

## 📂 Project Structure

```text
Team2_project/
├── controllers/
│   ├── team_supervisor/
│   │   ├── team_supervisor.py  (Main Entry: Referee & Decision Brain)
│   │   ├── utils.py            (Math Utilities)
│   │   ├── movement.py         (Movement & Obstacle Avoidance)
│   │   └── strategies/         (Strategy Folder)
│   │       ├── __init__.py
│   │       ├── goalie.py       (Goalie Logic)
│   │       └── striker.py      (Striker/Defender/Support Logic)
│   └── nao_player/
│       ├── nao_player.py       (Low-level Actuator)
│       └── motions/            (Motion Files Folder)
└── README.md
```

---

## 🛠️ Module Analysis

### 1. Core Controllers

#### 🧠 `controllers/team_supervisor/team_supervisor.py` (Brain/Referee)
The central command center. It utilizes Webots Supervisor privileges to acquire "God view" data and is responsible for frame-by-frame decision-making.

* **Full Synchronization**: Removed dependency on real-time (`time.time`) and strictly follows the Webots physical simulation step (32ms/step). This ensures a 1:1 lock between the physics engine and logic calculation, eliminating lag and desynchronization.
* **Communication Management**: Sends JSON-formatted commands to all robots (R1-R4, B1-B4) every frame.
* **State Monitoring & Fall Recovery**: Real-time detection of robot Z-axis height. Once a fall is detected, it immediately sends `INTERRUPT` commands to force the robot to interrupt the current action and execute a `GetUp` routine.
* **Minimap**: Renders a real-time tactical board on the screen, displaying player positions, ball position, score, and match time.

#### 🦾 `controllers/nao_player/nao_player.py` (Actuator/Player)
The low-level driver running on each Nao robot.

* **Zero-Latency Commands**: Implemented `poll_cmd` logic. It **clears the UDP buffer** in every frame loop and extracts only the command with the latest sequence number, eliminating command pile-up lag.
* **Emergency Interrupt**: Supports `INTERRUPT_` prefixed commands. Even if the robot is walking, it can instantly interrupt the action and switch to fall recovery.
* **Motion Management**: Dynamically loads `.motion` files, supporting walking, shooting, side-stepping, and getting up.

---

### 2. Strategy Modules

#### ⚡ `strategies/striker.py` (Field Player Logic)
Integrates tactical logic for Striker, Defender, and Support roles.

* **Striker**:
    * **Smart Dribbling**: Enables **high-precision obstacle avoidance** when far from goal (>1.2m); switches to shooting in the danger zone.
    * **Orbit Logic**: Automatically orbits to the back of the ball tangentially if positioned between the ball and the goal to avoid own goals.
    * **Left Foot Adaptation**: Uses `OFFSET_SIDE` to ensure the ball is positioned correctly for left-footed kicking.
* **Defender**:
    * **Positional Defense**: Calculates the line between the ball and own goal, intercepting on that line.
    * **Half-field Constraint**: Strictly limits movement to own half.

#### 🛡️ `strategies/goalie.py` (Goalie Logic)
* **Box Constraint**: Uses `clamp` to strictly limit the goalkeeper within the Goal Box.
* **Strafing Saves**: Enabled `can_strafe=True`. Prioritizes **Side Stepping** for lateral blocking instead of turning and running.

---

### 3. Algorithms (Utils & Movement)

#### 🧭 `movement.py` (Movement & Avoidance)
Converts tactical targets into specific commands (`FWD`, `TURN`, `SIDE`).

* **Artificial Potential Fields (APF)**: Generates smooth avoidance paths using attraction and repulsion forces.
* **Smart Priority**:
    1.  **Turn**: Priority #1. Ensure correct heading first.
    2.  **Forward**: Priority #2. Move straight if distance > 0.15m.
    3.  **Strafe**: Priority #3. Only used for fine-tuning when very close (<0.4m) and laterally misaligned.
* **Dynamic Precision**:
    * **Dribble Mode**: High precision (0.20 rad threshold).
    * **Free Run**: Lower precision (0.45 rad threshold) for fluidity.

---

## ⚽ Role Configuration

| Team | ID | Role | Function | Responsibility |
| :--- | :--- | :--- | :--- | :--- |
| **Blue** | **B1** | **Striker** | `run_striker` | Core offense, dribbling, shooting |
| | **B2** | **Defender** | `run_defender` | Own-half defense, interception |
| | **B3** | **Support** | `run_support` | Follows offense, receives passes |
| | **B4** | **Goalie** | `run_goalie` | Goalkeeping, lateral saves |
| **Red** | **R1** | **Defender** | `run_defender` | Own-half defense, interception |
| | **R2** | **Support** | `run_support` | Assists R3 in offense |
| | **R3** | **Striker** | `run_striker` | Core offense, dribbling, shooting |
| | **R4** | **Goalie** | `run_goalie` | Goalkeeping |

---

## 🚀 Getting Started

1.  **Prerequisites**:
    * Install **Webots** (R2023b or newer recommended).
    * Configure Python controller environment.

2.  **Check Motion Files**:
    Ensure `controllers/nao_player/motions/` contains:
    * `Forwards50.motion`, `Backwards.motion`
    * `TurnLeftSmall.motion`, `TurnRightSmall.motion`
    * `SideStepLeft.motion`, `SideStepRight.motion` (New!)
    * `Shoot.motion` (Left-foot)
    * `GetUpFront.motion`, `GetUpBack.motion`

3.  **Run**:
    * Open the Webots world file.
    * `TeamSupervisor` will automatically handshake.
    * Match starts automatically.

---

## ⚙️ Tuning

* **`strategies/striker.py`**:
    * `SHOOTING_RANGE` (Default `1.2`): Distance to switch from dribble to shoot.
    * `OFFSET_SIDE` (Default `0.05`): Positive for left foot, negative for right.
* **`movement.py`**:
    * `STRAFE_LIMIT_DIST` (Default `0.4`): Max distance to allow strafing.
    * `angle_threshold`: Turning sensitivity.
* **`nao_player.py`**:
    * `max_action_sec`: Action timeout.



controllers/
  team_supervisor/
    ├── team_supervisor.py  (主入口)
    ├── utils.py            (数学工具)
    ├── movement.py         (移动与避障)
    └── strategies/         (策略文件夹)
          ├── __init__.py
          ├── goalie.py     (守门员逻辑)
          └── striker.py    (前锋/后卫/支援逻辑)
  nao_player/
    ├── nao_player.py
    └── motions


    
# 🤖 Nao Soccer Simulation (Webots + Python)

这是一个基于 **Webots** 仿真环境的 **Nao 机器人足球** 控制系统。项目采用了 **集中式决策（Supervisor）** + **分布式执行（Player）** 的架构，通过 UDP 通信实现多机器人协作。

---

## 📂 项目结构与代码功能解析

### 1. 核心控制器 (Controllers)

#### 🧠 `controllers/team_supervisor/team_supervisor.py` (大脑/裁判)
这是整个系统的核心指挥中心。它利用 Webots 的 Supervisor 权限获取“上帝视角”数据，并负责每一帧的决策。

* **完全同步机制**：移除了对真实时间 (`time.time`) 的依赖，严格跟随 Webots 的物理仿真步长 (32ms/step)，确保物理引擎与逻辑计算 1:1 锁定，解决了机器人动作“波浪式”延迟的问题。
* **通信管理**：每一帧向所有机器人（R1-R4, B1-B4）发送 JSON 格式的指令。
* **状态监测 & 跌倒恢复**：实时检测机器人 Z 轴高度，一旦发现跌倒，立即发送 `INTERRUPT` 系列指令，强制机器人中断当前动作并执行起立（GetUp）。
* **小地图 (Minimap)**：在屏幕上实时绘制战术板，显示球员位置、球的位置、比分以及比赛时间。

#### 🦾 `controllers/nao_player/nao_player.py` (执行器/球员)
运行在每个 Nao 机器人身上的底层驱动程序，负责接收指令并驱动电机。

* **指令零延迟**：实现了 `poll_cmd` 逻辑，在每一帧循环中**清空 UDP 缓冲区**，只提取并执行最新的序列号指令，彻底消除了动作堆积导致的“慢半拍”现象。
* **紧急打断机制**：支持 `INTERRUPT_` 前缀指令。即使机器人正在走路，也能瞬间中断并切换到跌倒恢复状态。
* **动作库管理**：动态加载 `.motion` 文件，支持走路、射门、侧移、起立等动作。

---

### 2. 策略模块 (Strategies)

#### ⚡ `strategies/striker.py` (通用场上球员逻辑)
这个文件集成了除门将外所有角色的战术逻辑（包括前锋、后卫和支援）。

* **前锋 (Striker) 逻辑**：
    * **智能盘带**：在距离球门较远 (>1.2m) 时，开启**高精度避障**带球推进；进入危险区后自动切换为射门。
    * **绕行逻辑 (Orbit)**：如果机器人在球和球门之间，会自动沿切线方向绕行到球的后方，避免将球踢成乌龙。
    * **左脚适配**：针对左脚射门的机器人，计算站位时增加了 `OFFSET_SIDE` 偏移量，确保球位于左脚前方。
* **后卫 (Defender) 逻辑**：
    * **卡位防守**：始终计算球与自家球门的连线，站在连线上进行拦截。
    * **半场限制**：严格限制后卫只能在己方半场活动，防止跑位过深导致后场空虚。

#### 🛡️ `strategies/goalie.py` (门将逻辑)
* **禁区限制**：使用 `clamp` 函数将门将严格限制在小禁区（Goal Box）范围内，防止门将乱跑。
* **侧移扑救**：开启了 `can_strafe=True`，当门将面对来球时，会优先使用**左右横移 (Side Step)** 进行封堵，而不是转身跑。

---

### 3. 算法与工具 (Utils & Movement)

#### 🧭 `movement.py` (移动与避障算法)
负责将战术目标点转化为具体的动作指令 (`FWD`, `TURN`, `SIDE`)。

* **人工势场法 (APF)**：计算目标的引力和障碍物的斥力，生成平滑的避障路径。
* **智能动作优先级**：
    1.  **转向 (Heading)**：优先保证朝向正确，避免斜向移动。
    2.  **直行 (Forward)**：距离较远 (>0.15m) 时优先直行。
    3.  **侧移 (Strafe)**：只有当距离目标非常近 (<0.4m) 且需要微调左右偏差时，才启用侧移。
* **动态精度控制**：
    * **带球模式**：角度误差阈值极低 (0.20 rad)，防止把球踢飞。
    * **空跑模式**：角度误差阈值较宽 (0.45 rad)，追求移动流畅性。

---

## ⚽ 战术配置 (Role Configuration)

代码中使用了固定的角色分配方案：

| 队伍 | ID | 角色 | 代码对应函数 | 职责 |
| :--- | :--- | :--- | :--- | :--- |
| **蓝队 (Blue)** | **B1** | **前锋 (Striker)** | `run_striker` | 核心进攻，负责盘带和射门 |
| | **B2** | **后卫 (Defender)** | `run_defender` | 负责本方半场防守、卡位 |
| | **B3** | **支援 (Support)** | `run_support` | 跟随进攻，接应球 |
| | **B4** | **门将 (Goalie)** | `run_goalie` | 守门，左右横移扑救 |
| **红队 (Red)** | **R1** | **后卫 (Defender)** | `run_defender` | 负责本方半场防守、卡位 |
| | **R2** | **支援 (Support)** | `run_support` | 协助 R3 进攻 |
| | **R3** | **前锋 (Striker)** | `run_striker` | 核心进攻，负责盘带和射门 |
| | **R4** | **门将 (Goalie)** | `run_goalie` | 守门 |

---

## 🚀 快速开始 (Getting Started)

1.  **环境准备**：
    * 安装 **Webots** (建议 R2023b 或更新版本)。
    * 确保已配置好 Python 控制器环境。

2.  **动作文件**：
    确保 `controllers/nao_player/motions/` 目录下包含以下动作文件：
    * `Forwards50.motion`
    * `Backwards.motion`
    * `TurnLeftSmall.motion`, `TurnRightSmall.motion`
    * `SideStepLeft.motion`, `SideStepRight.motion` (用于门将和微调)
    * `Shoot.motion` (左脚射门)
    * `GetUpFront.motion`, `GetUpBack.motion`

3.  **运行仿真**：
    * 打开 Webots 世界文件。
    * 仿真开始后，`TeamSupervisor` 会自动进行握手 (Handshake)。
    * 握手完成后，比赛自动开始。

---

## ⚙️ 关键参数微调 (Tuning)

如果需要调整机器人行为，可修改以下关键常量：

* **`strategies/striker.py`**:
    * `SHOOTING_RANGE` (默认 `1.2`): 小于此距离时，前锋从带球切换为射门。
    * `OFFSET_SIDE` (默认 `0.05`): 击球点偏移量。正数适配左脚，负数适配右脚。
* **`movement.py`**:
    * `STRAFE_LIMIT_DIST` (默认 `0.4`): 距离目标小于此数值时，允许使用侧移微调。
    * `angle_threshold`: 控制转向的灵敏度。
* **`nao_player.py`**:
    * `max_action_sec`: 动作超时强制中断时间，防止死锁。