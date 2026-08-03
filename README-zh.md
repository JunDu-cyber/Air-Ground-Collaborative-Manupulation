# 空地协同排雷侦察与响应系统

*[English](README.md)*

一个基于 ROS Noetic / Gazebo Classic 的仿真研究工作区,模拟**无人机（UAV）—无人车（UGV）协同任务**：PX4 无人机携带下视 RGB-D 相机与 YOLO11 分割模型对雷区进行侦察；确认的目标位置被转换到 UGV 的自身坐标系（egocentric frame）中。Husky–UR5 无人车利用激光惯性里程计、地形感知规划器，以及（可选的）MoveIt/GPD 抓取流水线依次到达这些目标。

> **范围说明。** 本项目是仿真与课程/研究项目，并非真实世界的爆炸物处理（EOD）系统，仅可在配套的 Gazebo/PX4 仿真环境中运行。

## 一、:mortar_board:项目基本信息

| | |
|---|---|
| 组号 | `3` |
| 作业名称 | `空地协同户外操作系统` |
| 项目名称 | 空地协同排雷侦察与响应系统 |

## 二、软件包功能说明

**要解决的问题。** 在没有预建全局地图、没有 GPS 的前提下，从空中定位仿真地雷，并让地面机器人依次到达（可选地抓取）每个已确认目标。

**主要功能：**
- 无人机下视 RGB-D 相机侦察 + YOLO11s-seg 地雷检测器。
- 多帧空间关联确认，把带噪声的单帧检测整合为稳定的地雷地图。
- 以自身坐标系（egocentric）方式把确认目标交接到 UGV 的本地 `odom` 坐标系（任务本身不需要共享全局 `map` 坐标系，也不需要 GPS）。
- UGV 自主能力：激光惯性里程计、地形感知路径规划、按序访问目标。
- 可选的 MoveIt Task Constructor / GPD 抓取流水线，用于抓取仿真地雷道具。

**方法（简述——算法原理的完整调研见 [`docs/algorithm_research_references.md`](docs/algorithm_research_references.md)，此处不展开）：** UAV 飞行侧使用 PX4 SITL + EGO-Planner；检测侧使用 TensorRT/Ultralytics 版 YOLO11s-seg 模型；UGV 里程计使用 DLIO；UGV 导航使用基于 CMU 的局部规划器，代价源为 ANYbotics `elevation_mapping` 高程图层；可选抓取使用 MoveIt Task Constructor + GPD。

**输入：** 内置雷场的 `outdoor_city` Gazebo 世界、随包提供的 YOLO11s-seg 权重，以及运行时环境变量（见第八节）。

**输出：** 确认后的地雷地图（`mine_detection_output/mine_map.yaml`）、UGV 巡视/抓取状态（ROS 话题），以及可选的 RViz 可视化。

**适用场景：** 单机工作站上的 Gazebo Classic 仿真；未经大量额外工作不适用于真实硬件（见上方"范围说明"）。

### 任务流水线

```text
PX4 无人机 + 下视 RGB-D 相机
          │
          ▼
YOLO11s-seg 地雷定位
          │  /mine_detection/raw
          ▼
多帧关联与确认
          │  /mine_detection/map (MineMap)
          ▼
MineMap → WorldTarget 桥接
          │  /detected_targets
          ▼
UGV 目标巡视 + 地形感知规划
          │  /ugv/goal
          ▼
可选的 MoveIt / GPD 抓取-返回循环
```

空地坐标变换在启动时建立一次：DLIO 拥有 `odom → base_link`；一个锚定关系锁存 `odom → uav0/map_local`。因此主任务（egocentric 模式）运行期间不需要全局 `map` 坐标系，也不需要 GPS。

## 三、文件目录说明

| 路径 | 内容 |
|---|---|
| `airground_takeoff.sh` | 主一体化演示启动脚本。 |
| `src/uav_truth_tracker/` | 无人机 ROS 节点、侦察、地雷定位/融合、PX4 启动文件、自定义消息。 |
| `src/mobile_manipulator/` | Husky–UR5 仿真、DLIO/规划集成、高程滤波、目标巡视、桥接节点。 |
| `src/grasp_mtc/` | MoveIt Task Constructor 抓取流水线。 |
| `models/` | 无人机与地雷相机的 Gazebo 模型。 |
| `mine_seg_v2_delivery/` | 交付的 YOLO 分割权重与模型清单。 |
| `docs/` | 设计说明、操作手册、测试记录与答辩材料。 |
| `patches/` | 应用在外部 EGO-Planner 检出代码上的项目补丁。 |

## 四、运行环境

| 组件 | 测试目标版本 |
|---|---|
| 操作系统 | Ubuntu 20.04 |
| ROS | ROS Noetic |
| 仿真器 | Gazebo Classic 11 |
| 飞控栈 | PX4 v1.14.3 SITL + MAVROS + EGO-Planner |
| 构建工具链 | `catkin_make` 或 `catkin build`，C++17，Python 3.8（Noetic 自带系统 Python 3） |
| 感知（Python） | `ultralytics`、`torch`、`opencv-python`、`numpy`、`PyYAML`（未随包锁定具体版本——请安装与 Python 3.8 兼容的当前版本） |
| 可选 GPU 加速 | NVIDIA 驱动 + CUDA + TensorRT，以及导出的 ONNX 模型（仅 `UAV_DETECT_BACKEND=tensorrt`，即脚本默认值，才需要；CPU/Ultralytics 回退路径完全不需要这些） |
| 桌面会话 | 支持桌面 OpenGL/Gazebo 的图形会话 + `gnome-terminal`（启动脚本会为每个子系统各开一个终端标签，见第七节） |

本项目未记录、也未测试过其他操作系统/版本组合；如果你不在 Ubuntu 20.04 + ROS Noetic 上，需要自行适配包名。

## 五、依赖安装方法

将仓库克隆到 catkin 工作区，并从仓库根目录运行安装脚本：

```bash
git clone <repository-url> learning_ws
cd learning_ws
source /opt/ros/noetic/setup.bash
bash setup_uav.sh
```

`setup_uav.sh` 会安装 ROS/PX4 前置依赖、把 EGO-Planner 克隆到 `src/ego-planner/`、应用项目补丁、在 `${PX4_DIR:-$HOME/PX4-Autopilot}` 安装 PX4 SITL，并在首次运行时构建整个工作区。该脚本需要 `sudo` 权限，且耗时可能较长。

如尚未安装感知相关 Python 包，请安装：

```bash
python3 -m pip install --user ultralytics torch opencv-python numpy PyYAML
```

### 高程建图（elevation-mapping）依赖

ETH/ANYbotics 的源码依赖有意不随包内置（vendor）。构建高程建图相关功能前，请先导入锁定版本：

```bash
sudo apt install python3-vcstool ros-noetic-grid-map ros-noetic-grid-map-visualization
vcs import src < src/elevation_mapping.repos
catkin_make
source devel/setup.bash
```

如果工作区已经初始化过，拉取更新后重新构建即可：

```bash
catkin_make
source devel/setup.bash
```

除上述内容及 `setup_uav.sh` 中列出的项目外，本软件包不需要安装额外依赖。

## 六、运行前配置

- **环境变量**（完整列表见第八节）——唯一会改变运行**行为**（而非只是调参）的变量是 `UAV_DETECT_BACKEND`：如果没有可用的 NVIDIA/TensorRT 环境，务必设置 `UAV_DETECT_BACKEND=cpu UAV_DETECT_DEVICE=cpu`，因为脚本默认值是 `tensorrt`。
- **模型文件放置。** 提交模型必须位于 `mine_seg_v2_delivery/weights/best.pt`（本仓库已随包提供）。其清单见 [`mine_seg_v2_delivery/DEPLOYMENT.txt`](mine_seg_v2_delivery/DEPLOYMENT.txt)——任务类型、输入尺寸（960）、推荐置信度（清单中为 0.7337337337；启动文件自身默认值为 0.65，见第八节）以及验证指标。请使用 `best.pt`，不要用 `last.pt`。
- **`PX4_DIR`。** 如果 PX4 安装在 `~/PX4-Autopilot` 以外的位置，请在运行 `setup_uav.sh` 或 `airground_takeoff.sh` 之前导出 `PX4_DIR`。
- 默认运行不需要修改任何配置文件——所有任务参数都通过启动参数/环境变量暴露（见第八节），不存在写死的路径。

## :rocket:七、完整运行流程

在工作区根目录启动一体化仿真：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash

bash airground_takeoff.sh
```


**步骤 1。** 在工作区根目录，依次 `source` ROS 环境和工作区 overlay（顺序如上）。
**步骤 2。** 按上面的命令运行 `airground_takeoff.sh`。这是唯一的入口——默认演示不需要再手动启动任何其他内容。
**步骤 3。** 脚本会依次打开自己的 `gnome-terminal` 标签，顺序固定，且每一个都会在内部等待上一个就绪后才启动：`1_Gazebo` → `1b_Mine_Field` → `2_PX4_Spawn_UAV` → `3_UGV` → `3b_UGV_LIO` →（如启用则含 `3c_UGV_NAV`、`7a_MoveGroup`、`7b_Grasp`）→ `4_MAVROS` → `5_AirGround_ROS` → `6_Takeoff` → `8_UAV_Detect`。你不需要点开任何一个标签——脚本内部的 `sleep` 已经保证了这个顺序。
**步骤 4。** 等待约 30–60 秒让所有标签起来，然后进行第十节中的交互步骤（确认检测、启动巡视）。
**步骤 5。** 按第九节（输出）与第十节（成功判定标准）检查结果。

启动脚本会以暂停状态启动 Gazebo、生成五颗地雷的雷场、拉起 PX4、UGV、DLIO、规划模块、空地 TF/高程融合层、MAVROS、无人机起飞桥接节点，以及地雷检测。



## 八、输入说明

`airground_takeoff.sh` 接受的环境变量：

| 变量 | 默认值 | 作用 |
|---|---:|---|
| `UAV_DETECT` | `true` | 启动无人机地雷检测栈。 |
| `UAV_DETECT_BACKEND` | `tensorrt` | 设为 `cpu` 使用 Ultralytics/PyTorch 路径。 |
| `UAV_DETECT_DEVICE` | `cpu` | 传给检测器的 PyTorch 设备。 |
| `UAV_SURVEY` | `false` | 启用无人机自动侦察航线。 |
| `UGV_NAV` | `true` | 启动 CMU 规划与目标巡视节点。 |
| `UGV_GRASP` | `true` | 启动 MoveIt/GPD，并在到达目标时请求抓取。 |
| `FLIGHT_H` | `2.0` | 无人机飞行高度（米）。 |
| `START_RVIZ` | `true` | 启动高程图 RViz 配置。 |
| `PX4_DIR` | `~/PX4-Autopilot` | PX4 源码/构建目录。 |

检测器启动文件自身的 `confidence` 参数默认值为 **0.65**（`src/uav_truth_tracker/launch/uav_mine_detection.launch`）；只有在评估与随包默认值不同的工作点时才需要传入 `confidence:=...`。

流水线在运行时消费的话题输入：

| 话题 | 类型 | 含义 |
|---|---|---|
| `/mine_camera/rgb/image_raw`、`.../depth`、`.../camera_info` | `sensor_msgs` | 检测器消费的无人机下视 RGB-D 相机数据。 |
| `/ugv/start_tour` | `std_srvs/Trigger`（服务调用） | 冻结已收集目标并启动 UGV 巡视（手动调用，或由 `UAV_SURVEY=true` 自动触发）。 |

## 九、输出说明

每次任务都会把确认后的地雷地图写入：

```text
mine_detection_output/mine_map.yaml
```

该文件是**生成的输出，未纳入版本控制**；每次运行开始时都会被**覆盖**（而非追加），如需保留上一次结果，请先自行复制备份。

实时输出以 ROS 话题形式给出，而非文件：

| 接口 | 类型 | 含义 |
|---|---|---|
| `/mine_detection/raw` | `uav_truth_tracker/MineDetectionArray` | 逐帧、经深度定位的检测结果。 |
| `/mine_detection/map` | `uav_truth_tracker/MineMap` | 关联并确认后的地雷假设集合。 |
| `/detected_targets` | `mobile_manipulator/WorldTarget` | 交接给 UGV 的确认地雷位置。 |
| `/ugv/tour_status` | `std_msgs/String` | 目标巡视状态。 |
| `/ugv/goal` | `geometry_msgs/PoseStamped` | UGV 当前导航目标（`odom` 系）。 |
| `/elevation_mapping/elevation_map_postprocessed` | `grid_map_msgs/GridMap` | 融合后的高程与后处理地形图层。 |
| `/uav/pose_cov` | 带协方差的位姿 | 高程建图使用的无人机状态协方差。 |

自定义无人机消息定义在 [`src/uav_truth_tracker/msg`](src/uav_truth_tracker/msg) 下；UGV 交接消息为 [`WorldTarget.msg`](src/mobile_manipulator/msg/WorldTarget.msg)。

可视化：当 `START_RVIZ=true`（默认值）时，会自动打开一个 RViz 窗口，展示高程图、目标巡视标记与地形图层——无需额外步骤查看。

## 十、运行成功的判断标准

按以下顺序确认，即表示运行正常：

1. **检测结果在持续累积**——启动巡视前先确认：
   ```bash
   rostopic hz /mine_camera/rgb/image_raw
   rostopic echo /mine_detection/map
   rostopic echo /detected_targets
   ```
   如果自动侦察被禁用（默认情况），可在 RViz 中用 **2D Nav Goal** 手动引导无人机。
2. **UGV 巡视能启动并完成**——调用
   ```bash
   rosservice call /ugv/start_tour "{}"
   ```
   （或让 `UAV_SURVEY=true` 自动触发）后，`/ugv/tour_status` 会持续推进并最终报告完成；RViz 的 `/target_tour_markers` 会显示 UGV 依次访问各个标记点。
3. **高程图已经建立**——`/terrain_map` 与高程图 RViz 显示项显示出真实地形，而不是空白网格。
4. （可选抓取路径）——MoveIt/GPD 会在每次到达目标后报告一次抓取尝试。按第十一节所述，该阶段仍属实验性质，不作为单独的成功判定条件。

在整个过程中，任何一个 `gnome-terminal` 标签内都不应出现持续性的报错/异常，这是基本预期。

## 十一、常见问题

- **构建时提示找不到 `ego-planner`：** 运行 `bash setup_uav.sh`，或按安装脚本的方式把它克隆到 `src/ego-planner/` 并应用 `patches/` 中的补丁。
- **没有 TensorRT 可执行文件 / CUDA 报错：** 使用 `UAV_DETECT_BACKEND=cpu UAV_DETECT_DEVICE=cpu` 运行；并确认已安装上述 Python 依赖。
- **没有检测结果：** 先检查 RGB、深度、camera_info 话题，再查看 `/mine_camera/diagnostics` 与 `/mine_detection/debug_image`。
- **UGV 不动：** 必须先有目标被确认，才能调用 `/ugv/start_tour`；检查 `/detected_targets`、`/ugv/tour_status`、`/state_estimation` 与 `/terrain_map`。
- **高程图是空的：** 用 `rosrun tf2_ros tf2_echo odom uav0/map_local` 验证锁存的变换，并检查无人机点云话题 `/uav0/mapping/velodyne_points_gated`。
- **抓取阶段失败或表现不稳定：** 属预期情况——设置 `UGV_GRASP=false` 可只跑导航。抓取阶段对最终进近位姿及仿真道具的感知效果较为敏感；请将其视为实验性集成，而非有保证的任务结果（当前状态与已知阻塞点见 [`docs/grasp_PLAN.md`](docs/grasp_PLAN.md)）。

## 致谢

本项目基于 ROS、Gazebo Classic、PX4、MAVROS、EGO-Planner、ANYbotics `elevation_mapping` / ETH `grid_map`、Clearpath Husky、Universal Robots UR5、Robotiq、MoveIt、GPD、DLIO、Ultralytics YOLO，以及项目中包含或引用的 Gazebo 模型集合构建而成。
