# BXI 机器人四肢关节检测软件

本项目是一套面向机器人手臂和腿部总成的台架检测软件，用于检查单条或左右两条肢体的各个关节能否正常运动。

测试时，可将两条手臂或两条腿分别固定在台架左右两侧。软件按照预定顺序逐个驱动关节，采集位置、速度和力矩反馈，自动判断关节运动方向、运动范围、跟踪误差和关节间串扰是否正常。

项目同时支持 MuJoCo 仿真与 Elf3 实机。所有测试都应先在仿真环境中验证，再连接真实台架。

## 主要功能

- 支持手臂和腿部总成检测。
- 支持左侧、右侧和左右对应关节同时测试。
- 同时测试模式把左右对应关节组成一组同步运动，并分别生成检测结果。
- 提供图形化操作界面。
- 默认使用面向操作员的生产模式，并可切换到完整调试模式。
- MuJoCo 仿真和实机反馈数字孪生画面直接显示在检测 UI 中。
- 调试模式可安全重启并切换仿真/实机运行环境。
- 调试模式支持可独立启停的 Xbox 类手柄流程遥控。
- 实时显示目标位置和实际位置曲线。
- 显示关节位置、速度和力矩反馈。
- 自动完成关节正向和负向运动测试。
- 自动判断运动量、跟踪误差、速度、力矩及其他关节串扰。
- 仿真初始化时自动将机器人抬升到台架悬空高度，避免脚与地面接触。
- 仿真运行期间预测脚底与地面的干涉，并在危险命令发出前停止。
- 支持正常停止和急停锁定。
- 自动生成 JSON、CSV 汇总报告和完整采样数据。
- 仿真与实机共用同一套检测流程。
- 提供可复制部署的 Linux 安装包。

## 支持的关节

### 手臂

每条手臂包含 7 个关节：

- 肩关节俯仰
- 肩关节侧摆
- 肩关节旋转
- 肘关节俯仰
- 腕关节侧摆
- 腕关节俯仰
- 腕关节旋转

左右两条手臂共 14 个关节。

### 腿部

每条腿包含 6 个关节：

- 髋关节俯仰
- 髋关节侧摆
- 髋关节旋转
- 膝关节俯仰
- 踝关节俯仰
- 踝关节侧摆

左右两条腿共 12 个关节。

## 检测过程

检测让每个关节按安全全行程依次执行：

```text
零位安全姿态 → 碰撞安全下限 → 碰撞安全上限 → 零位安全姿态
```

关节运动采用平滑的五次最小加加速度轨迹，减少突然启动和停止对台架的冲击。

选择“左右两侧（对应关节同时）”时，顺序为：

```text
手臂：左右腕同步 → 左右肘同步 → 左右肩同步
腿部：左右踝同步 → 左右膝同步 → 左右髋同步
```

具体运动链顺序：

```text
手臂：腕旋转 → 腕俯仰 → 腕侧摆 → 肘 → 肩旋转 → 肩侧摆 → 肩俯仰
腿部：踝侧摆 → 踝俯仰 → 膝 → 髋旋转 → 髋侧摆 → 髋俯仰
```

末端关节运动空间和惯量较小，优先检测可以更早发现电机、接线或反馈故障，避免在状态未知时先驱动肩、髋等大范围关节。

每一组的左右关节使用相同的归一化轨迹进度，单程时间按两侧行程中较长的一侧计算。结果仍按单个关节分别判定，正在同步运动的另一侧关节不会被误判为串扰。

双侧同时模式使用视觉镜像方向：俯仰轴左右使用相同符号，侧摆和旋转轴左右使用相反符号。为了避免两条腿同时向内时互相碰撞，默认 5° 碰撞余量下，双腿髋侧摆镜像范围约为左侧 `-3°～137°`、右侧 `-137°～3°`；髋旋转约为左侧 `-55.5°～160°`、右侧 `-160°～55.5°`。仅测试左侧或右侧时仍保留各自的单侧安全范围。

测试肩关节或髋关节时，软件会自动执行：

```text
中心姿态 → 平滑蜷缩肘/膝 → 主关节测试 → 主关节回中 → 平滑展开
```

默认紧凑姿态经过单侧全范围、双侧镜像范围、蜷缩和展开过渡的碰撞扫描：

| 主动关节 | 辅助蜷缩姿态 |
|---|---:|
| 肩俯仰、肩侧摆 | 肘关节 -45° |
| 肩旋转 | 肘关节 -15° |
| 髋俯仰 | 膝关节 30° |
| 髋侧摆 | 膝关节 135° |
| 髋旋转 | 膝关节 5° |

蜷缩关节不计入当前主关节的串扰，但其位置、速度、力矩、软件限位和碰撞保护始终有效。除当前被测关节和对应蜷缩关节外，其他关节保持检测中心位置。

安全全行程不是直接使用机械硬限位。软件同时扫描 MuJoCo 简化碰撞体和 STL 可见外壳，取两者中更保守的范围，再叠加默认 5° 模型碰撞余量和 2° 机械限位余量。运动期间还会持续检查可见网格侵入；预测到自碰撞时不会发送该命令，并立即锁定控制器。

大角度轨迹按设置的最高速度自动延长时间。例如约 324° 的肩或腕关节不会在 1.5 秒内完成全行程，而是自动延长到满足速度限制。

当前模型和默认余量计算出的目标范围如下：

| 关节 | 左侧 | 右侧 |
|---|---:|---:|
| 肩俯仰 | -160° ～ 160° | -160° ～ 160° |
| 肩侧摆 | -0.9° ～ 170° | -170° ～ 0.9° |
| 肩旋转 | -25° ～ 160° | -160° ～ 25° |
| 肘俯仰 | -50° ～ 62.9° | -50° ～ 62.9° |
| 腕侧摆 | -160° ～ 160° | -160° ～ 160° |
| 腕俯仰 | -70° ～ 70° | -70° ～ 70° |
| 腕旋转 | -40° ～ 40° | -40° ～ 40° |
| 髋俯仰 | -111.9° ～ 150.8° | -111.9° ～ 150.8° |
| 髋侧摆 | -12° ～ 137° | -137° ～ 12° |
| 髋旋转 | -160° ～ 160° | -160° ～ 160° |
| 膝俯仰 | 0° ～ 145° | 0° ～ 145° |
| 踝俯仰 | -43.2° ～ 40° | -43.2° ～ 40° |
| 踝侧摆 | -15° ～ 15° | -15° ～ 15° |

这些范围只覆盖机器人模型自身，不包含实际台架、夹具、外接线束和周边设备。实机范围必须根据台架再次收紧，不能把模型验证等同于现场无碰撞保证。

## 合格判定

软件根据以下数据判断每个关节是否正常：

- 正向实际运动量是否达到要求。
- 负向实际运动量是否达到要求。
- 实际位置与目标位置的最大误差是否超限。
- 其他被测关节是否出现异常联动或串扰。
- 最大关节速度是否超限。
- 最大反馈力矩是否超限。
- 测试过程中关节反馈是否连续、有效。

任意一项超过设置阈值，该关节会被标记为“不通过”，结果表中会显示具体原因。

速度安全保护使用相同的持续超限判定：仿真和实机速度连续超限 `10 ms` 都会锁定。
该滤波只忽略单个反馈采样的瞬时尖峰，持续超速仍会停止发送控制命令。

默认参数用于软件调试和仿真验证。正式检测前，必须根据实际电机型号、减速比、机械结构和台架姿态设置每个关节的安全阈值。

## 安全要求

机器人关节和台架具有较大的运动力和惯量。实机操作前必须满足以下要求：

- 台架及肢体固定牢靠。
- 关节运动范围内没有人员和其他物体。
- 台架配备可以切断电机动力的实体急停按钮。
- 夹具不会阻挡关节返回驱动定义的零位。
- 已确认机械限位和线缆不会在测试过程中发生干涉。
- 安全全行程前已确认线缆允许对应关节接近最大转角。
- 已关闭行走、振动或其他机器人控制程序。
- `hardware/actuators_cmds` 只能存在一个控制命令发布者。
- 首次实机测试使用较大的碰撞/机械余量和较慢速度。
- 实机参数必须先在 MuJoCo 中完成验证。

软件中的“紧急停止”会停止发送控制命令，使硬件驱动的命令丢失保护介入。软件急停不能代替独立的实体急停和动力断电回路。

急停或安全故障触发后，软件会进入锁定状态。排查机械和电气问题并确认安全后，必须重启软件才能重新初始化。

## 系统环境

推荐开发和运行环境：

- Ubuntu 22.04 x86_64
- ROS2 Humble
- Python 3.10
- MuJoCo 3.x
- MuJoCo Python 模块
- PyQt5
- pyqtgraph
- xserver-xephyr（为后台 MuJoCo 提供从不映射到桌面的嵌套显示）
- BXI ROS2 软件包

运行前需要存在：

```text
/opt/ros/humble/setup.bash
/opt/bxi/bxi_ros2_pkg/setup.bash
```

安装界面依赖：

```bash
sudo apt update
sudo apt install python3-pyqt5 python3-pyqtgraph xserver-xephyr
python3 -m pip install --user mujoco
```

## 构建项目

进入项目目录：

```bash
cd /home/bxi/BXI/robot_limb_testing/bxi_rl_controller_ros2_example
```

加载环境并构建：

```bash
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
colcon build --packages-select bxi_example_py_elf3
source install/setup.bash
```

## 启动 MuJoCo 仿真

推荐使用启动脚本：

```bash
./scripts/run_limb_inspection.sh simulation
```

也可以直接使用 ROS2 launch：

```bash
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
source install/setup.bash
ros2 launch bxi_example_py_elf3 limb_inspection_sim.launch.py
```

启动后只显示检测软件主界面，MuJoCo 实时画面位于界面右上区域。后台 MuJoCo 仍负责物理计算，但从创建时就在永不映射到桌面的 Xephyr 嵌套显示中运行，因此不会出现原生窗口闪现。

内嵌视图支持：

- 左键拖动旋转视角。
- 右键或中键拖动平移视角。
- 鼠标滚轮缩放。
- 双击恢复默认视角。

内嵌视图默认加载轻量银色 CAD，三角面约为原模型的 18%，蓝色/灰色碰撞调试体
不会出现在操作界面中。视图刷新限制为最高 15 FPS；Qt 尚未消费上一帧时不会继续
排队。渲染实际运行在独立进程，隐藏供应商窗口与内嵌视图分别限制 llvmpipe
工作线程，不会与 ROS 控制共享 GIL 或无上限占用 CPU。
该模式可在没有独立显卡的机器人主控上使用。若只用于离线外观检查并希望加载原始
完整 CAD，可在启动前设置：

```bash
BXI_MUJOCO_VIEW_DETAIL=full ./scripts/run_limb_inspection.sh simulation
```

隐藏供应商窗口默认使用 2 个软件渲染线程；内嵌视图按 CPU 核数选择 2～8 个线程。
低功耗主控可分别覆盖：

```bash
BXI_VENDOR_RENDER_THREADS=1 BXI_VIEW_RENDER_THREADS=2 \
  ./scripts/run_limb_inspection.sh simulation
```

仿真初始化时，软件会先通过 `simulation/sim_reset` 将机器人基座抬升到默认 `1.70 m`，模拟四肢固定在台架两侧、脚部悬空的测试状态，然后执行关节初始化。该高度可通过 launch 参数调整：

```bash
ros2 launch bxi_example_py_elf3 limb_inspection_sim.launch.py \
  simulation_bench_height_m:=1.7
```

模型中没有真实台架、夹具和线缆。悬空高度只用于避免完整机器人模型与 MuJoCo 地面接触，不能替代实机台架的机械干涉检查。

## 启动实机检测

只有在仿真测试完成、台架检查完成并准备好实体急停后，才能运行：

```bash
./scripts/run_limb_inspection.sh hardware
```

实机驱动需要管理员权限，启动脚本会请求 `sudo` 授权。

启动脚本和调试模式切换按钮会先通过 sysfs 检查 Xilinx PCI CAN-FD 设备
`10ee:7022`。未检测到 FPGA 时不会启动 `motor_pwr=True` 的硬件驱动：从仿真界面
切换时会弹出提示并留在当前界面；直接请求实机启动时会回退到 MuJoCo 仿真并在界面
弹出提示，不会因硬件节点启动失败直接退出整个软件。

也可以手动启动：

```bash
sudo -E bash -c '
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
source install/setup.bash
ros2 launch bxi_example_py_elf3 limb_inspection_hw.launch.py
'
```

不要在完整仿真测试和现场参数确认之前直接运行实机模式。

## 生产模式与调试模式

软件默认进入“生产模式”。生产模式只保留日常检测需要的信息和操作：

- 系统状态、反馈状态和检测进度
- 手臂/腿部及台架侧选择
- 台架安全确认和全行程安全确认
- 初始化、一键检测、平稳停止和紧急停止
- MuJoCo 实时仿真画面
- 醒目的总通过判定和逐关节通过情况

运动参数、合格阈值、目标/反馈曲线、详细结果、运行日志和手动报告导出只在“调试模式”中显示。点击窗口右上角的“进入调试模式”即可切换；调试完成后点击“进入生产模式”返回。两种界面使用同一个控制器、同一套安全保护和同一个 MuJoCo 渲染视图，切换不会启动第二个仿真进程。

调试模式保存的检测对象、运动参数和合格阈值会被生产模式直接使用，仿真与实机也
读取和保存同一份配置。运行模式只决定通信话题、仿真视图和安全校验，不再切换配置文件。
界面会记住上次选择的模式；首次运行默认使用生产模式。生产模式不会绕过安全确认、
反馈超时、命令间隔、关节限位、速度、力矩或碰撞保护。

### 调试模式切换仿真/实机

通过推荐启动脚本运行时，调试工具区提供“切换到实机模式/切换到 MuJoCo 仿真”
按钮。切换并不是在同一个控制器中热切话题，而是先停止当前 launch，等待驱动和
仿真进程完全退出，再由启动脚本启动目标模式。切换到实机时仍会请求 `sudo`，并明确
提示硬件驱动会设置 `motor_pwr=True`。从终端启动时通过 `sudo` 授权；从应用菜单等
无终端环境启动时通过系统 `pkexec` 图形授权框授权。

直接执行 `ros2 launch` 时没有外层启动脚本负责重启，因此模式切换按钮会禁用。

实机模式也显示 MuJoCo 只读数字孪生视图。该视图由
`hardware/joint_states` 驱动，只负责显示，不运行供应商物理仿真、不发布控制命令，
也不改变检测安全逻辑。渲染仍在独立进程中以不高于 15 FPS 运行。

### Xbox 类手柄流程遥控

调试工具区可以单独启用或关闭手柄，默认设备为 `/dev/input/js0`。可在启动前覆盖：

```bash
BXI_GAMEPAD_DEVICE=/dev/input/js1 \
  ./scripts/run_limb_inspection.sh simulation
```

采用 Elf3 当前配套 Xbox 类手柄的 Linux joydev 按键布局（该手柄的 X/Y
编号为 3/4）：

| 按键 | 操作 |
|---|---|
| A（joydev 0） | 初始化所选关节 |
| X（joydev 3） | 开始自动检测 |
| B（joydev 1） | 平稳停止 |
| Y（joydev 4） | 紧急停止并锁定 |
| 左方向键（joydev axis 6 负向） | 选择手臂测试 |
| 右方向键（joydev axis 6 正向） | 选择腿部测试 |

启用手柄功能会自动勾选“台架已固定”和“全行程无干涉”两项；关闭或掉线时自动取消。
机器人初始化真正完成后，对支持 Linux EV_FF 的手柄尝试短振约
300 ms；不支持时静默忽略。实测 PowerA Battle Dragon `20d6:4013`
无 EV_FF 能力，SDL、HID 输出报告和 universal-pidff 均无法触发振动，
该型号会跳过振动尝试。其他手柄必要时可用
`BXI_GAMEPAD_EVENT_DEVICE=/dev/input/event7` 显式指定力反馈事件设备。

初始化、检测或回中过程中会锁定方向键的手臂/腿切换。
初始化和开始检测不再弹出二次确认框，按键满足当前状态条件后会直接执行。手柄功能只
遥控检测流程，不用摇杆直接生成关节位置命令。启用时若设备不存在或无读取
权限会弹出提示并保持软件运行；使用中拔出手柄或读取失败会关闭手柄功能并提示，不会
产生控制动作或退出软件。不同厂商手柄的 joydev 按键编号可能不同，需要通过
`jstest /dev/input/js0` 核对映射后再使用。

## 界面操作

### 1. 选择检测对象

在左侧控制区选择：

- 手臂或腿
- 左侧、右侧或左右两侧

### 2. 设置运动参数（调试模式）

MuJoCo 仿真与实机使用相同的参数和安全阈值。界面输入框不设置业务最小值或
最大值，也不会在输入时自动截断。开始初始化或检测时，仍会拒绝会造成除零或
无意义轨迹的值；关节软件限位、碰撞预测和脚底地面保护也始终生效。

- 最短单程时间：轨迹允许使用的最短时间。
- 全行程最高速度：安全全行程轨迹的速度上限。
- 模型碰撞余量：在 MuJoCo 无碰撞边界内继续缩小的角度。
- 机械限位余量：与关节机械硬限位保持的角度距离。
- 端点保持：到达目标位置后的保持时间。

### 3. 设置合格阈值（调试模式）

- 最大跟踪误差
- 最小运动比例
- 最大关节串扰
- 最大速度
- 最大力矩

### 调试参数速查

仿真和实机读取、保存同一组参数。界面允许直接输入数值；点击初始化或开始检测时
才检查数值能否形成有效轨迹。建议先用默认值完成单侧测试，再逐项调整。

| 参数 | 单位 | 作用与调整影响 |
| --- | --- | --- |
| 部位 | — | 选择手臂或腿部关节集合。 |
| 台架侧 | — | 选择左侧、右侧或左右对应关节同步运动。实机首次测试建议单侧。 |
| 最短单程时间 | s | 每段全行程轨迹的最短时间，软件可能为满足速度上限自动延长。 |
| 全行程最高速度 | °/s | 安全全行程、蜷缩过渡和平稳回中使用的目标轨迹峰值上限。数值越大，运动越快。 |
| 模型碰撞余量 | ° | 从 MuJoCo 无碰撞边界向内收缩的角度。越大越保守；过大可能使可用范围为空。 |
| 机械限位余量 | ° | 从软件机械限位向内收缩的角度。越大越保守；设为零仍不会越过软件硬限位。 |
| 端点保持 | s | 到达每个目标后保持的时间，用于观察稳态误差。 |
| 最大跟踪误差 | ° | 目标角度与反馈角度允许的最大差值。越小判定越严格。 |
| 最小运动比例 | — | 实际运动幅度与目标幅度的最低比值。越大越严格，零表示不检查运动不足。 |
| 最大关节串扰 | ° | 当前测试关节运动时，其他受控关节允许的最大偏移。越小越严格。 |
| 最大速度 | °/s | 既用于结果判定，也用于运行中持续超速停机；连续超限 `10 ms` 会锁定控制器。 |
| 最大力矩 | Nm | 既用于结果判定，也用于运行中力矩超限停机。其物理含义取决于驱动反馈定义。 |

两个 launch 的公共运行参数也保持一致：控制频率 `100 Hz`、反馈超时 `0.20 s`、
初始化时间 `10 s`、最大命令间隔 `0.08 s`、持续超速判定时间 `0.01 s`。
`simulation_bench_height_m` 只描述 MuJoCo 台架高度，不属于实机运动参数。

项目启动脚本及仿真/实机 launch 都强制设置 `ROS_LOCALHOST_ONLY=1`，只接收本机仿真
或硬件反馈。外部环境中的 `ROS_LOCALHOST_ONLY=0` 会被覆盖，局域网其他开发机即使
使用相同 ROS domain，也不能向本程序注入同名 `joint_states` 或 `actuators_cmds`。
本机重复发布者仍会被单实例锁和发布者数量检查拦截。

### 4. 完成安全确认

确认台架固定、运动区域无人且实体急停可用，然后勾选安全确认。使用安全全行程时，还必须确认线缆、夹具和全行程范围无干涉。

### 5. 初始化机器人

点击“初始化机器人”。软件会执行 Elf3 两阶段复位并缓慢加载关节刚度。

仿真模式会在两阶段复位前先把机器人抬升到台架悬空高度；状态栏显示“仿真机器人已抬升到 1.70 m”后才继续初始化。

初始化第一阶段可能使关节向驱动定义的零位运动。必须提前确认该运动路径安全。

### 6. 开始检测

状态显示“就绪”后，点击“一键检测”。软件将按顺序完成所选关节的检测。

测试过程中可以观察：

- 当前关节名称
- 检测总进度
- 目标和反馈位置曲线
- 逐关节检测结果
- 运行事件和故障信息

### 7. 停止检测

“平稳停止”会中断当前检测，并使用平滑轨迹返回检测中心位置。

出现机械干涉、异响、剧烈振动、反馈异常或人员进入运动区域时，应立即按下实体急停，同时点击软件“紧急停止”。

## 默认参数

仓库默认参数保存在：

```text
src/bxi_example_py_elf3/config/limb_inspection_defaults.json
```

界面修改后的参数保存在当前用户目录：

```text
~/.config/bxi_limb_inspection/settings.json
```

仿真与实机共用这一个文件。旧版本的 `settings_hardware.json` 仅在上述公共文件不存在时
作为一次兼容读取来源，后续保存都会写入公共文件。通过项目启动脚本进入实机模式时，
原用户的配置目录会显式传给 root 进程，不会在 `/root/.config` 下另建一套。

首次启动使用适合实机验证的保守参数：仅左侧、单程至少 `2 s`、最高 `10°/s`。
确认单侧动作方向、反馈、实体急停和台架空间正确后，再在调试模式中逐步扩大范围或启用双侧。

默认设置包括：

```text
界面模式：生产模式
检测模式：安全全行程
最短单程时间：2.0 s
全行程最高速度：10°/s
模型碰撞余量：5°
机械限位余量：2°
端点保持：0.5 s
最大跟踪误差：2°
最小运动比例：60%
最大关节串扰：3°
最大速度：30°/s
最大力矩：80 Nm
仿真台架悬空高度：1.70 m
```

这些数值不是正式产品验收标准。必须根据真实硬件和台架重新标定。

## 检测报告

测试完成后，软件默认把报告保存到：

```text
~/BXI/limb_inspection_reports
```

每次检测生成三个文件：

```text
elf3_limb_inspection_日期_时间.json
elf3_limb_inspection_日期_时间.csv
elf3_limb_inspection_日期_时间_samples.csv
```

文件内容：

- JSON：测试模式、参数、通过数量和所有关节结果。
- CSV：每个关节的汇总判定结果。
- samples CSV：所选关节的完整目标位置、实际位置、速度和力矩数据。

## 软件打包

构建 Linux 发布包：

```bash
bash scripts/package_limb_inspection.sh
```

生成文件：

```text
dist/bxi-limb-inspection-linux-x86_64.tar.gz
dist/bxi-limb-inspection-linux-x86_64-installer.run
```

优先使用单文件安装包：

```bash
chmod +x bxi-limb-inspection-linux-x86_64-installer.run
./bxi-limb-inspection-linux-x86_64-installer.run --check
./bxi-limb-inspection-linux-x86_64-installer.run
```

安装位置：

```text
~/.local/share/bxi_limb_inspection
```

安装后可以从应用菜单启动仿真模式，也可以在终端运行：

```bash
~/.local/share/bxi_limb_inspection/scripts/run_limb_inspection.sh simulation
```

实机模式：

```bash
~/.local/share/bxi_limb_inspection/scripts/run_limb_inspection.sh hardware
```

卸载软件：

```bash
./bxi-limb-inspection-linux-x86_64-installer.run --uninstall
```

卸载不会删除用户配置和历史检测报告。

## 项目结构

```text
.
├── README.md
├── scripts/
│   ├── run_limb_inspection.sh
│   ├── package_limb_inspection.sh
│   ├── deploy_limb_inspection.sh
│   └── self_extract_limb_inspection.sh
├── src/bxi_example_py_elf3/
│   ├── bxi_example_py_elf3/
│   │   ├── limb_inspection_core.py
│   │   ├── limb_collision_guard.py
│   │   ├── limb_inspection_controller.py
│   │   ├── limb_inspection_config.py
│   │   ├── limb_inspection_report.py
│   │   ├── limb_inspection_posture.py
│   │   ├── limb_hidden_simulation.py
│   │   ├── limb_simulation_view.py
│   │   └── limb_inspection_ui.py
│   ├── config/
│   │   └── limb_inspection_defaults.json
│   └── launch/
│       ├── limb_inspection_sim.launch.py
│       └── limb_inspection_hw.launch.py
├── test/
│   ├── test_limb_inspection_core.py
│   ├── run_limb_inspection_sim_smoke.py
│   └── verify_collision_safe_ranges.py
└── dist/
```

核心模块说明：

- `limb_inspection_core.py`：关节定义、测试配置、轨迹和结果判定。
- `limb_collision_guard.py`：基于 MuJoCo 模型预测命令姿态的自碰撞。
- `limb_inspection_controller.py`：ROS2 通信、初始化、状态机和安全保护。
- `limb_inspection_config.py`：默认配置、用户配置加载和持久化。
- `limb_inspection_report.py`：独立的 JSON、CSV 与采样报告生成。
- `limb_inspection_posture.py`：紧凑姿态、双侧镜像方向和互碰安全范围规划。
- `limb_hidden_simulation.py`：管理不可见 X11 父窗口、Xephyr 和供应商 MuJoCo 生命周期。
- `limb_simulation_view.py`：独立线程的交互式 MuJoCo 视图。
- `limb_inspection_ui.py`：图形界面编排、控件、曲线和结果表。
- `limb_inspection_sim.launch.py`：启动 MuJoCo 和仿真检测界面。
- `limb_inspection_hw.launch.py`：启动 Elf3 硬件驱动和实机检测界面。

模块之间按“纯检测逻辑 → ROS2 控制 → UI 编排”单向依赖。后台物理仿真与 UI 渲染使用不同进程/线程：供应商 MuJoCo 负责物理和 ROS2 反馈，`SimulationViewport` 只根据反馈绘图，任何视角操作都不会修改机器人控制状态。

## 测试与验证

检查 Python 文件：

```bash
python3 -m compileall -q \
  src/bxi_example_py_elf3/bxi_example_py_elf3 \
  src/bxi_example_py_elf3/launch
```

运行核心逻辑测试：

```bash
PYTHONPATH=src/bxi_example_py_elf3 \
pytest -q test/test_limb_inspection_core.py
```

验证全部 26 个关节的碰撞安全区间：

```bash
source install/setup.bash
python3 test/verify_collision_safe_ranges.py
```

运行 MuJoCo 全链路冒烟测试前，应先启动仿真节点，再执行：

```bash
source install/setup.bash
python3 test/run_limb_inspection_sim_smoke.py
```

冒烟测试使用双腿 6 对关节同步安全全行程，只用于确认 ROS2 复位、命令发布、反馈接收、同步状态机和结果生成链路。

## 常见问题

### 界面显示没有关节反馈

检查对应话题：

```bash
ros2 topic echo --once /simulation/joint_states
```

实机模式：

```bash
ros2 topic echo --once /hardware/joint_states
```

### 初始化按钮无法使用

确认：

- 已收到所选关节反馈。
- 已勾选台架安全确认。
- 没有正在进行的测试或返回中心动作。
- 软件没有处于急停锁定状态。

### 检测到多个命令发布者

关闭其他机器人控制节点，并检查：

```bash
ros2 topic info /hardware/actuators_cmds --verbose
```

实机测试时只能由本软件发布该话题。

### 安装包启动失败

先执行环境检查：

```bash
./bxi-limb-inspection-linux-x86_64-installer.run --check
```

确认 ROS2、BXI 软件包、PyQt5、pyqtgraph、MuJoCo Python 模块和 `xserver-xephyr` 已安装。

### 只连接两条拆下来的手臂或腿时没有反馈

当前实机启动使用 `hardware_elf3`。需要确认硬件驱动允许未连接的其他电机离线，并仍能正常发布所选关节数据。

如果硬件驱动要求完整机器人电机拓扑，需要为台架增加专用的手臂或腿部硬件配置，包括 CAN 通道、电机 ID 和离线电机处理规则。

## 实机投入前需要确认

目前软件已经完成 MuJoCo 仿真链路验证，但尚未在真实台架上完成验收。正式使用前需要确认：

- 各关节台架安装中心角。
- 各关节允许的正向和负向运动范围。
- 手臂和腿部台架的 CAN 通道及电机 ID。
- 未连接关节在硬件驱动中的处理方式。
- 各关节正常速度和力矩范围。
- `JointState.effort` 的实际物理含义和单位。
- 是否需要采集电机温度及驱动器故障码。
- 是否需要记录机器人序列号、工位号和操作员。
- 是否需要与数据库或 MES 系统连接。

完成现场标定后，建议把统一参数升级为按机器人型号、肢体类型和关节分别管理的检测配方。
