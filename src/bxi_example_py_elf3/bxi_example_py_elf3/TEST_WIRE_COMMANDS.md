# Elf3 Test Wire 启动命令

工作区路径：

```text
/home/bxi/BXI/robot_limb_testing/bxi_rl_controller_ros2_example
```

## 1. 首次运行或源码修改后：构建工作区

```bash
cd "/home/bxi/BXI/robot_limb_testing/bxi_rl_controller_ros2_example"
bash build.sh
```

## 2. 终端 1：启动 MuJoCo 仿真器

```bash
cd "/home/bxi/BXI/robot_limb_testing/bxi_rl_controller_ros2_example"

source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
source install/setup.bash

ros2 launch bxi_example_py_elf3 elf3_dof29_sim.launch.py
```

保持此终端运行，不要关闭。

## 3. 终端 2：启动 Test Wire 控制节点

```bash
cd "/home/bxi/BXI/robot_limb_testing/bxi_rl_controller_ros2_example"

source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
source install/setup.bash

ros2 run bxi_example_py_elf3 bxi_example_py_elf3_test_wire \
  --ros-args -p /topic_prefix:=simulation/
```

程序会等待约 5 秒，然后依次打印：

```text
robot reset 1!
robot reset 2!
```

保持此终端运行，不要关闭。

## 4. 终端 3：开始播放 data.txt 关节轨迹

```bash
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
source "/home/bxi/BXI/robot_limb_testing/bxi_rl_controller_ros2_example/install/setup.bash"

ros2 topic pub --once /motion_commands \
  communication/msg/MotionCommands \
  "{btn_9: 1}"
```

发送命令后，`bxi_example_test_wire` 会循环读取：

```text
./src/bxi_example_py_elf3/data/data.txt
```

文件中的每一行应包含 29 个关节位置。

## 5. 检查节点和话题

```bash
ros2 node list
ros2 topic info /motion_commands
ros2 topic info /simulation/actuators_cmds
```

正常情况下应能看到 MuJoCo 仿真节点和 `bxi_example_py_elf3_test_wire` 控制节点。

## 6. 停止程序

分别在终端 2 和终端 1 中按：

```text
Ctrl+C
```

建议先停止控制节点，再停止 MuJoCo。

## 注意事项

- 必须从工作区根目录启动 Test Wire 节点，否则相对路径 `./src/bxi_example_py_elf3/data/data.txt` 可能找不到。
- Test Wire 不使用 `W/S/A/D` 速度命令，只通过 `btn_9` 开始关节轨迹播放。
- 当前代码使用 `self.robot_reset(2, False)`，仿真机器人会保留虚拟悬挂，适合振动或线束测试。
- 如需释放虚拟悬挂，将其改为 `self.robot_reset(2, True)`，修改后重新执行 `bash build.sh`。
- 当前 `example_launch_test_wire.py` 中存在未定义的 `onnx_file`，因此暂时采用“终端 1 启动仿真器、终端 2 启动控制节点”的方式。
