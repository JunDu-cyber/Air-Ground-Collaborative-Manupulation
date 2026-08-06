# 户外空地协同连续排雷系统

*[English](README.md)*

一个基于 ROS Noetic / Gazebo Classic 的**无人机（UAV）—无人车（UGV）协同排雷仿真系统**：PX4 无人机携带下视 RGB-D 相机与 YOLO11 分割模型对雷区进行侦察；确认的目标位置被转换到 UGV 的自身坐标系（egocentric frame）中。Husky–UR5 无人车利用激光惯性里程计、地形感知规划器和 MoveIt 流水线执行“抵近—抓取—返程—放置”循环。

> **范围说明。** 本项目是仿真与课程/研究项目，并非真实世界的爆炸物处理（EOD）系统，仅可在配套的 Gazebo/PX4 仿真环境中运行。

本仓库中的提交材料：

- 项目源码和配置：本仓库。
- 汇报文件：[`空地协同连续排雷系统.pptx`](空地协同连续排雷系统.pptx)。
- UAV 建图与地形可视化片段（3 分 11 秒，1920×1080，H.264/MP4）：[`空地协同排雷演示视频.mp4`](空地协同排雷演示视频.mp4)。该视频是辅助材料，不能代替第十节的完整验收检查。

## 一、:mortar_board:项目基本信息

| | |
|---|---|
| 组号 | `3` |
| 作业名称 | `空地协同户外操作系统` |
| 项目名称 | 户外空地协同连续排雷系统 |
| 小组成员 | 武天豪、赵汝堃、杜军、吴淑林 |

## 二、软件包功能说明

**要解决的问题。** 在没有预建共享全局地图的前提下，从空中定位仿真地雷，并让地面机器人依次抵近、抓取、返程和放置每个已确认目标。UGV 的目标交接与地面导航链采用自身坐标系，不依赖 GPS；PX4 SITL 内部仍可能使用仿真 GNSS 传感器。

**主要功能：**
- 无人机下视 RGB-D 相机侦察 + YOLO11s-seg 地雷检测器。
- 多帧空间关联确认，把带噪声的单帧检测整合为稳定的地雷地图。
- 以自身坐标系（egocentric）方式把确认目标交接到 UGV 的本地 `odom` 坐标系（UGV 地面任务不需要共享全局 `map` 坐标系或 GPS）。
- UGV 自主能力：激光惯性里程计、地形感知路径规划、按序访问目标。
- MoveIt Task Constructor 抓取流水线。提交配置使用解析式俯抓候选，并在运输阶段用 Gazebo 固定关节锁住选中的 `landmine_*` 刚体；夹爪闭合用于视觉呈现，不宣称为摩擦夹持。GPD 适配器仅作为非验收研究代码保留。

**方法（简述——算法原理的完整调研见 [`docs/algorithm_research_references.md`](docs/algorithm_research_references.md)，此处不展开）：** UAV 飞行侧使用 PX4 SITL + EGO-Planner；提交的 CPU 检测路径使用 Ultralytics YOLO11s-seg；UGV 里程计使用 DLIO；UGV 导航使用基于 CMU 的局部规划器与 ANYbotics `elevation_mapping` 高程代价；抓取使用 MoveIt Task Constructor 与解析式候选。TensorRT 适配器仍保留，但不属于实测提交配置。

**输入：** 随包提供的 `outdoor_city` Gazebo 世界、由 `src/uav_truth_tracker/launch/spawn_outdoor_mine_field.launch` 生成的五雷场、YOLO11s-seg 权重，以及运行时环境变量（见第八节）。

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
MoveIt 抓取—返程—放置循环
```

空地坐标变换在启动时建立一次：DLIO 拥有 `odom → base_link`；一个锚定关系锁存 `odom → uav0/map_local`。因此 UGV 目标交接与地面任务不需要共享全局 `map` 坐标系或 GPS；这并不表示 PX4 SITL 内部关闭了仿真 GNSS。

## 三、文件目录说明

| 路径 | 内容 | 使用方式 |
|---|---|---|
| `airground_takeoff.sh` | 一体化演示入口和运行默认值。 | 用户执行；通过环境变量调参，不需要改脚本。 |
| `one_key_takeoff.sh` | 为历史调试保留的 UAV-only 飞行/建图启动脚本。 | 不用于完整课程任务；完整流程只运行 `airground_takeoff.sh`。 |
| `record_uav_map.sh` | 为建图/历史调试保留的 UAV-only 点云录制工具。 | 可选调试工具；不会启动完整课程任务。 |
| `stop_airground.sh` | 宽范围关闭当前用户的 ROS/PX4/Gazebo 仿真栈；执行前先阅读第十一节警告。 | 用户执行。 |
| `setup_uav.sh` | 安装外部依赖并编译工作空间。 | 用户执行；可通过 `PX4_DIR` 修改 PX4 位置。 |
| `uav_deps.repos` | 为历史工作流保留的 EGO-Planner 兼容导入清单。 | 仅供兼容；受支持的 `setup_uav.sh` 流程会获取并核对第五节所列的 EGO 准确 commit。 |
| `requirements-runtime.txt` | 已验证 CPU 检测路径的固定 Python 依赖。 | 安装依赖时使用。 |
| `src/uav_truth_tracker/` | UAV 节点、侦察、地雷定位/融合、PX4 启动文件和自定义消息。 | `scripts/` 为源码，`launch/` 与 `config/` 为运行参数。 |
| `src/mobile_manipulator/` | Husky–UR5 仿真、DLIO/规划、高程滤波、目标巡视和桥接节点。 | `scripts/`/`src/` 为源码，`launch/`/`config/` 为运行参数，`rviz/` 为显示预设。 |
| `src/grasp_mtc/` | MoveIt Task Constructor 抓取流水线。 | `scripts/` 为源码，`launch/` 为任务启动参数。 |
| `src/gpd_ros/` | 可选 GPD 消息与检测器封装。 | 没有 `libgpd` 时仍生成 ROS 消息；默认任务不依赖 GPD。 |
| `models/`、`worlds/` | UAV 模型和共享 Gazebo 世界。 | 随包输入资源。 |
| `mine_seg_v2_delivery/` | YOLO 分割权重与模型清单。 | 随包输入模型；默认使用 `weights/best.pt`。 |
| `docs/` | 设计、调研与测试记录。 | 参考资料。 |
| `patches/` | EGO-Planner 项目补丁。 | 由 `setup_uav.sh` 自动应用。 |
| `build/`、`devel/` | Catkin 编译产物。 | 程序生成，可重新编译。 |
| `mine_detection_output/` | 本次运行确认的地雷地图。 | 程序生成，下次运行时覆盖。 |

## 四、运行环境

| 组件 | 已记录环境或可复现目标 |
|---|---|
| 操作系统 | Ubuntu 20.04.6 LTS |
| ROS | ROS Noetic |
| 仿真器 | Gazebo Classic 11.15.1 |
| 最近本机运行使用的飞控栈 | PX4 commit `bda25bfcc1a817f4ba559497c8ad6962f114cfd7`（2026-08-06 检查时为 `v1.17.0-alpha1-1668-gbda25bfcc1-dirty`）+ MAVROS + EGO-Planner commit `bfda51284c8c1b476043255a8145ef925a3778a5` |
| 本仓库安装的干净复现目标 | PX4 标签 `v1.14.3`、commit `1dacb4cdef2d7145754fc788fa8dc482eed74b40`，构建目标为 `px4_sitl_default gazebo-classic` |
| 主要 ROS 软件包 | MAVROS 1.20.1、Navigation 1.17.3、robot_localization 2.7.7、MoveIt 1.1.16、catkin-tools 0.9.4 |
| 构建工具链 | GCC 9.4.0、CMake 3.16.3、C++17、catkin-tools 0.9.4（`catkin build`）、Python 3.8.10 |
| 已验证 CPU 感知环境 | Ultralytics 8.4.60、PyTorch 2.4.1+cpu、torchvision 0.19.1+cpu、OpenCV 4.13.0、NumPy 1.24.4、SciPy 1.10.1、PyYAML 5.3.1 |
| 可选 GPU 加速 | NVIDIA 驱动 + CUDA + TensorRT，以及导出的 ONNX 模型。只有显式选择 `UAV_DETECT_BACKEND=tensorrt` 时才使用；默认 CPU/Ultralytics 路径不需要这些组件。 |
| 桌面会话 | 支持桌面 OpenGL/Gazebo 的图形会话 + `gnome-terminal`（启动脚本会为每个子系统各开一个终端标签，见第七节） |
| 实际测试硬件 | Lenovo ThinkBook 15 G4 IAP，Intel Core i5-1240P（16 逻辑处理器）、16 GB 内存、Intel 集成显卡，无 NVIDIA GPU；不需要实体 UAV/UGV。 |
| 存储要求 | 至少保留 10 GB 可用空间，用于 PX4 源码/编译产物、ROS 编译产物、日志和生成地图。 |

本机 PX4 行记录了开发机上的准确 checkout（包括其本地修改），它本身不可移植。新机器复现时，`setup_uav.sh` 会有意安装并核对干净的 v1.14.3 基线，而不是假装能够复现一个 dirty 工作区。本项目未记录、也未测试过其他操作系统/版本组合；如果你不在 Ubuntu 20.04 + ROS Noetic 上，需要自行适配包名。

## 五、依赖安装方法

**前置条件：** Ubuntu 20.04 已配置 ROS Noetic apt 软件源并安装 `ros-noetic-desktop-full`（本项目实际测试所用课程镜像已提供）。其他全新 Ubuntu 20.04 主机应先安装 ROS Noetic，并确认 `/opt/ros/noetic/setup.bash` 存在；未加载 ROS 环境时，项目安装脚本会主动停止。

将仓库克隆到 catkin 工作区，并从仓库根目录运行安装脚本：

```bash
git clone --recurse-submodules https://github.com/JunDu-cyber/Air-Ground-Collaborative-Manupulation.git learning_ws
cd learning_ws
source /opt/ros/noetic/setup.bash
bash setup_uav.sh
```

`setup_uav.sh` 会初始化并核对固定版本子模块与 ANYbotics 辅助仓库、安装 Python/ROS 依赖、准备自身坐标系导航源码、克隆并修补 EGO-Planner、在 `${PX4_DIR:-$HOME/PX4-Autopilot}` 安装或补全 PX4 SITL，并构建完整工作区。该脚本需要 `sudo` 权限，且耗时可能较长。PX4 版本核对通过后，脚本会把实际目录记录到 `${XDG_CONFIG_HOME:-$HOME/.config}/airground/env.sh`，并向 `~/.bashrc` 追加一个受管理的 Gazebo 环境块；显式导出的 `PX4_DIR` 始终优先。

| 外部资源 | 已验证版本 | 放置位置与处理方式 |
|---|---|---|
| [ZJU FAST-Lab EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner) | `bfda51284c8c1b476043255a8145ef925a3778a5` | `src/ego-planner/`；按提交号获取并自动应用补丁，不需要解压或手动配置环境变量。 |
| [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) | 标签 `v1.14.3`、commit `1dacb4cdef2d7145754fc788fa8dc482eed74b40` | `${PX4_DIR:-$HOME/PX4-Autopilot}`；包含子模块，并编译 `px4_sitl_default gazebo-classic`。若该目录已有其他 PX4 版本，安装脚本不会覆盖，而会停止并提示改用新的 `PX4_DIR`。 |
| `direct_lidar_inertial_odometry` | `fc8d183f18cdcfb9bb4fc754c6d373cedc4cbd04` | Git 子模块，路径为 `src/direct_lidar_inertial_odometry/`。 |
| `autonomous_exploration_development_environment` | `bf0cba71365271ebff09831a05afd78578150300` | Git 子模块，路径为 `src/autonomous_exploration_development_environment/`。 |
| `robot_body_filter` | `b6635e9c40d0524d70e4e0059a5c6c6bb382d6f4` | Git 子模块，路径为 `src/robot_body_filter/`。 |
| `FAST_LIO` / `livox_ros_driver`（可选基线） | `7cc4175de6f8ba2edf34bab02a42195b141027e9` / `3d240d5666129e1a3052e78ee8487a04b08fdda3` | Git 子模块，路径为 `src/FAST_LIO/` 和 `src/livox_ros_driver/`；保留作对比，默认不编译。 |
| ANYbotics `message_logger`、`kindr`、`kindr_ros` | 提交号记录在 `src/elevation_mapping.repos` | 由 `setup_uav.sh` 导入 `src/`。项目修改版 `elevation_mapping` 已直接纳入 `src/elevation_mapping/`，来源见其中的 `UPSTREAM.md`。 |
| YOLO11s-seg 模型 | 清单见 `mine_seg_v2_delivery/DEPLOYMENT.txt` | 已包含在 `mine_seg_v2_delivery/weights/best.pt`，不需要移动或解压。 |

安装脚本会从 `requirements-runtime.txt` 安装固定版本 Python 感知依赖。如以后只需修复该环境，可执行：

```bash
python3 -m pip install --user -r requirements-runtime.txt
```

### 更新或修复已有工作区

拉取新版本后，重新运行受支持的安装入口。它会再次核对源码版本、恢复依赖与补丁、运行 `rosdep`、核对/安装 PX4，并重新编译：

```bash
source /opt/ros/noetic/setup.bash
bash setup_uav.sh
source devel/setup.bash
```

各 `package.xml` 中声明的 ROS 软件包由 `setup_uav.sh` 内的 `rosdep` 处理。GPD 与 TensorRT 仅保留为对比适配器，不属于可复现的提交/验收配置；验收时保持 `GRASP_SOURCE=analytic` 和 `UAV_DETECT_BACKEND=cpu`。

## 六、运行前配置

- **环境变量**（常用任务开关与高级覆盖项见第八节）。已验证默认值为 CPU 检测、人工 UAV 目标、开启 UGV 导航/抓取、高程代价和解析式抓取候选。排查时建议一次只改变一个子系统。
- **模型文件放置。** 提交模型必须位于 `mine_seg_v2_delivery/weights/best.pt`（本仓库已随包提供）。其清单见 [`mine_seg_v2_delivery/DEPLOYMENT.txt`](mine_seg_v2_delivery/DEPLOYMENT.txt)——任务类型、输入尺寸（960）、推荐置信度（清单中为 0.7337337337；启动文件自身默认值为 0.65，见第八节）以及验证指标。请使用 `best.pt`，不要用 `last.pt`。
- **`PX4_DIR`。** 如果 PX4 需要安装在 `~/PX4-Autopilot` 以外的位置，请为 `setup_uav.sh` 导出 `PX4_DIR`。版本核对通过后，安装脚本会把该目录保存到 `${AIRGROUND_ENV_FILE:-${XDG_CONFIG_HOME:-$HOME/.config}/airground/env.sh}`，`airground_takeoff.sh` 会自动读取；显式导出的 `PX4_DIR` 仍可覆盖保存值。
- 默认一体化入口不要求修改个人绝对路径；活动路径从仓库根目录或第八节变量推导。仓库中的部分旧独立 launch 仍保留各自默认值，不属于本流程。

## :rocket:七、完整运行流程

在工作区根目录启动一体化仿真：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash

bash airground_takeoff.sh
```

默认使用已验证的 CPU 检测路径。`UAV_DETECT_BACKEND=tensorrt` 只为另行配置的研究机器保留，未在验收硬件上测试。


**步骤 1。** 在工作区根目录，依次 `source` ROS 环境和工作区 overlay（顺序如上）。
**步骤 2。** 运行 `airground_takeoff.sh`。它是默认演示的唯一入口，不需要第二条 launch 命令。
**步骤 3。** 脚本按固定顺序打开 `gnome-terminal` 标签：`1_Gazebo` → `1b_Mine_Field` → `2_PX4_Spawn_UAV` → `3_UGV` → `3b_UGV_LIO` →（如启用则含 `3c_UGV_NAV`、`7a_MoveGroup`、`7b_Grasp`）→ `4_MAVROS` → `5_AirGround_ROS` → `6_Takeoff` → `8_UAV_Detect`。内置延时只负责按序派发；是否真正就绪要以步骤 4 的检查为准。
**步骤 4。** 等待约 30–60 秒。在新终端中加载两层环境，然后完成下列检查再发送目标（连续运行的 `hz`/`tf2_echo` 命令产生有效数据后，用 `Ctrl-C` 结束）：

```bash
rostopic echo -n1 /mavros/state              # connected: True
rostopic echo -n1 /state_estimation           # 收到一条 UGV 里程计
rosrun tf2_ros tf2_echo odom uav0/map_local   # 变换稳定输出
rostopic hz /terrain_map                      # 频率非零
rostopic hz /mine_camera/rgb/image_raw        # 频率非零
missing=0
for service in /ugv/align_to_mine /grasp/execute /grasp/place /ugv/start_tour; do
  rosservice list | grep -qx "$service" || { echo "missing: $service"; missing=1; }
done
test "$missing" -eq 0
```

`startup sequence done` 只表示启动命令已经发出，不表示所有节点都已就绪。检查通过后再按第十节确认检测并启动巡视。
**步骤 5。** 按第九节（输出）与第十节（成功判定标准）检查结果。

启动脚本会以暂停状态启动 Gazebo、生成五颗地雷的雷场、拉起 PX4、UGV、DLIO、规划模块、空地 TF/高程融合层、MAVROS、无人机起飞桥接节点，以及地雷检测。



## 八、输入说明

提交任务使用的静态输入与人工交互输入：

| 输入 | 格式 / 接口 | 仓库路径或来源 | 默认流程中的作用 |
|---|---|---|---|
| `outdoor_city.world` | SDFormat/XML Gazebo 世界 | `src/mobile_manipulator/worlds/outdoor_city.world` | `airground_takeoff.sh` 选用的默认仿真场景。 |
| `spawn_outdoor_mine_field.launch` | ROS launch/XML | `src/uav_truth_tracker/launch/spawn_outdoor_mine_field.launch` | 在默认世界中生成配套的五雷测试场。 |
| `best.pt` | PyTorch checkpoint | `mine_seg_v2_delivery/weights/best.pt` | 默认 CPU 检测器使用的 YOLO11s-seg 权重。 |
| RViz `2D Nav Goal` | 人工 `geometry_msgs/PoseStamped` 交互 | RViz 工具 → `/move_base_simple/goal` | `UAV_SURVEY=false`（默认）时提供 UAV 巡检航点。 |

任务所需输入均来自仓库资源、仿真传感器数据或 RViz 交互；不需要实体 UAV、UGV、激光雷达、相机、遥控器或其他外部设备。

`airground_takeoff.sh` 接受的常用任务开关：

| 变量 | 默认值 | 作用 |
|---|---:|---|
| `UAV_DETECT` | `true` | 启动无人机地雷检测栈。 |
| `UAV_DETECT_BACKEND` | `cpu` | `cpu` 使用已验证的 Ultralytics/PyTorch 路径；`tensorrt` 是 NVIDIA 非验收配置。 |
| `UAV_DETECT_DEVICE` | `cpu` | 传给检测器的 PyTorch 设备。 |
| `UAV_SURVEY` | `false` | 启用无人机自动侦察航线。 |
| `UGV_NAV` | `true` | 启动 CMU 规划与目标巡视节点。 |
| `UGV_GRASP` | `true` | 启动 MoveIt，并在到达目标时请求抓取。 |
| `GRASP_SOURCE` | `analytic` | 抓取候选源；提交验收只使用 `analytic`。 |
| `GRASP_DETECTOR` | `color` | 抓取流水线使用的腕部相机地雷检测器。 |
| `FLIGHT_H` | `2.0` | 无人机飞行高度（米）。 |
| `LOW_ALTITUDE` | `true` | 使用 EGO 低空飞行/规划配置。 |
| `START_RVIZ` | `true` | 启动高程图 RViz 配置。 |
| `UGV_COST_SOURCE` | `elevation` | UGV 地形代价来源。 |
| `UGV_GLOBAL_PLANNER` | `far` | UGV 全局规划器。 |
| `UGV_UAV_PRIOR` | `true` | 把经过高度门控的 UAV 点云融合到 UGV 中心高程图。 |
| `UGV_ELEVATION_UPDATE` | `false` | 启用后再加入 UGV 激光雷达的实时高程更新。 |
| `UGV_MAP_SIZE` | `120` | 高程图边长（米）。 |
| `UGV_MAP_RES` | `0.35` | 高程图分辨率（米/格）。 |

路径、生成位姿、仿真和启动时序覆盖项：

| 变量 | 默认值 | 作用 |
|---|---:|---|
| `AIRGROUND_ENV_FILE` | `${XDG_CONFIG_HOME:-$HOME/.config}/airground/env.sh` | 保存安装脚本已核验 PX4 目录的受管理文件；只有维护多套安装时才需要覆盖。 |
| `PX4_DIR` | `$HOME/PX4-Autopilot` | PX4 源码/构建目录。 |
| `EGO_WS`、`UGV_WS` | 仓库根目录 | UAV/UGV 启动终端使用的工作区 overlay。 |
| `UGV_WORLD` | `src/mobile_manipulator/worlds/outdoor_city.world` | 默认共享 Gazebo 世界。 |
| `GAZEBO_WORLD` | `UGV_WORLD` 的值 | 传给 `gazebo_ros` 的世界。 |
| `LIDAR_SDF` | `models/iris_depth_camera_lidar_terrain/model.sdf` | PX4 UAV 模型 SDF。 |
| `MAVROS_PX4_LAUNCH` | `/opt/ros/noetic/share/mavros/launch/px4.launch` | MAVROS 启动文件。 |
| `UAV_POINTS_MAP_DIR` | `$HOME/pointcloud_maps` | UAV 点云地图保存目录。 |
| `SPAWN_X`、`SPAWN_Y`、`SPAWN_Z` | `0.0`、`-18.0`、`1.5` | UAV 在 Gazebo 中的生成位置（米）。 |
| `SPAWN_YAW` | `1.5707963` | UAV 生成偏航角（弧度）。 |
| `MAP_LOCAL_Z` | `0.0` | UAV 局部原点在 `odom` 中的竖直偏移，通常保持零。 |
| `PX4_GAZEBO_GUI` | `true` | 显示 Gazebo 客户端。 |
| `PX4_SIM_SPEED_FACTOR` | `1` | PX4 仿真速度倍率。 |
| `PHYSICS_STEP`、`PHYSICS_RATE` | `0.005`、`200.0` | Gazebo 时间步长和最大更新率。 |
| `AG_ENABLE_GATE` | 自动推导 | 高度门覆盖项；通常由高程图所有者和 `UGV_UAV_PRIOR` 自动决定。 |
| `GAZEBO_LOAD_WAIT`、`PX4_WAIT`、`UGV_SPAWN_WAIT` | `10`、`8`、`6` | 向下一终端派发前的启动延时（秒）。 |
| `MAVROS_WAIT`、`ROS_WAIT` | `5`、`20` | MAVROS 与 ROS 层启动延时（秒）。 |

以上是集成脚本读取的全部环境变量；各子系统 launch 还提供直接 `roslaunch` 参数。检测器启动文件自身的 `confidence` 为 **0.65**（`src/uav_truth_tracker/launch/uav_mine_detection.launch`）。只有单独评估检测器时才修改；集成默认使用 0.65。

流水线在运行时消费的话题输入：

| 话题 | 类型 | 含义 |
|---|---|---|
| `/mine_camera/rgb/image_raw` | `sensor_msgs/Image` | UAV 下视 RGB 图像。 |
| `/mine_camera/depth/image_raw` | `sensor_msgs/Image` | 与 RGB 对时的深度图像。 |
| `/mine_camera/rgb/camera_info` | `sensor_msgs/CameraInfo` | 三维投影使用的 RGB 光学标定。 |
| RViz `2D Nav Goal` → `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | `UAV_SURVEY=false`（默认）时的 UAV 人工目标；UGV 不消费该话题。 |
| `/ugv/start_tour` | `std_srvs/Trigger`（服务调用） | 冻结当前已收集目标并启动 UGV 巡视。五雷场必须先等 `confirmed_count: 5`；调用后到来的检测会被有意忽略。 |

默认人工控制 UAV 时，将 RViz 固定坐标系保持为 `odom`，依次点击下表五个雷区附近。目标桥会把点击位置转换到 UAV 局部坐标并自动应用 `FLIGHT_H`；这些坐标只是配套世界中的操作员航点，不会作为算法的检测真值输入。

| 建议点击顺序 | `odom` 平面坐标（m） |
|---:|---:|
| 1 | `(1.4, 0.9)` |
| 2 | `(2.3, -1.1)` |
| 3 | `(5.5, 1.4)` |
| 4 | `(8.5, -1.4)` |
| 5 | `(11.5, 1.3)` |

在每个区域等待标记变为已确认后再继续；YAML 未显示五颗全部确认前，不要调用 `/ugv/start_tour`。

## 九、输出说明

生成文件及覆盖规则：

| 路径 | 行为 |
|---|---|
| `logs/setup_build.log` | 最近一次 `setup_uav.sh` 的 catkin 构建日志；下次安装构建时覆盖，且被 Git 忽略。 |
| `mine_detection_output/mine_map.yaml` | 候选与确认地雷的原子快照；集成检测器启动时重置，地图 revision 改变时覆盖。 |
| `~/pointcloud_maps/uav_points_map_latest.pcd` | `odom` 系中的最新 UAV 累积点云；每 30 秒及正常退出时覆盖。 |
| `~/pointcloud_maps/uav_points_map_YYYYMMDD_HHMMSS.pcd` | 每 30 秒新增的带时间戳快照；需手动清理。 |
| `~/trajectory_logs/flight_trajectory_YYYYMMDD_HHMMSS.csv` | UAV 建图节点每次启动时新建的实际/期望轨迹日志。 |

这些都是未纳入版本控制的生成输出。再次运行或清理前，应把需要的结果复制到其他位置。默认自身坐标系任务把点云记录在 `odom`，因此会禁用旧 GPS/UTM 模式的 `uav_map_origin.yaml`，避免把不匹配的 sidecar 与 PCD 混用。

实时输出以 ROS 话题形式给出，而非文件：

| 接口 | 类型 | 含义 |
|---|---|---|
| `/mine_detection/raw` | `uav_truth_tracker/MineDetectionArray` | 逐帧、经深度定位的检测结果。 |
| `/mine_detection/map` | `uav_truth_tracker/MineMap` | 关联并确认后的地雷假设集合。 |
| `/detected_targets` | `mobile_manipulator/WorldTarget` | 交接给 UGV 的确认地雷位置。 |
| `/ugv/tour_status` | `std_msgs/String` | 目标巡视状态。 |
| `/ugv/goal` | `geometry_msgs/PoseStamped` | UGV 当前导航目标（`odom` 系）。 |
| `/elevation_mapping/elevation_map_postprocessed` | `grid_map_msgs/GridMap` | 融合后的高程与后处理地形图层。 |
| `/terrain_map` | `sensor_msgs/PointCloud2` | UGV 规划器消费的可通行性/代价点云。 |
| `/target_tour_markers` | `visualization_msgs/MarkerArray` | 待处理、当前和已处理巡视目标。 |
| `/uav0/mapping/points_world` | `sensor_msgs/PointCloud2` | 转换到共享 `odom` 系的 UAV 点云。 |
| `/mine_detection/debug_image` | `sensor_msgs/Image` | 带时间戳的检测可视化图像。 |
| `/mine_camera/diagnostics` | `diagnostic_msgs/DiagnosticArray` | RGB-D 频率、帧龄、同步、编码与 TF 健康状态。 |
| `/uav/pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | 高程建图使用的 UAV 状态协方差。 |

自定义无人机消息定义在 [`src/uav_truth_tracker/msg`](src/uav_truth_tracker/msg) 下；UGV 交接消息为 [`WorldTarget.msg`](src/mobile_manipulator/msg/WorldTarget.msg)。

可视化：当 `START_RVIZ=true`（默认值）时，RViz 会预先配置融合/原始高程图、`/terrain_map`、`/target_tour_markers`、UAV 点云、机器人模型和 `/mine_detection/debug_image`。

## 十、运行成功的判断标准

按以下顺序确认，即表示运行正常：

1. **冻结目标集前，五颗地雷都已确认**——先检查：
   ```bash
   rostopic hz /mine_camera/rgb/image_raw
   rostopic echo -n1 /mine_detection/map
   grep '^confirmed_count: 5$' mine_detection_output/mine_map.yaml
   ```
   地图消息应包含五个 `confirmed: true` 条目，YAML 命令应输出 `confirmed_count: 5`。自动侦察默认关闭，可在 RViz 中用 **2D Nav Goal** 手动引导 UAV。不要提前启动巡视：巡视节点会冻结输入集合并忽略后续检测。
2. **UGV 以五个目标启动巡视**——调用
   ```bash
   rosservice call /ugv/start_tour "{}"
   rostopic echo /ugv/tour_status
   ```
   服务返回信息和 `/ugv/tour_status` 必须显示 `total: 5`；RViz 的 `/target_tour_markers` 显示当前与待处理目标。
3. **高程图已经建立**——`/terrain_map` 与高程图 RViz 显示项显示出真实地形，而不是空白网格。
4. **每颗地雷均完成仿真抓取循环**——使用默认 `UGV_GRASP=true` 时，五个目标都必须各自出现一次成功的 `ALIGN → GRASP → NAV_HOME → PLACE` 流程及 `place OK`。GRASP 阶段仍由腕部相机目标驱动接近；夹爪做可视闭合，执行器只在锁定运输时从附近实体中选择对应的 `landmine`/`landmine_*`，建立 Gazebo 固定关节，并保持到 `/grasp/place` 解锁同一实体。只有五次 PLACE 均成功后，最终 `DONE` 才能作为验收结果；单独看到 `DONE` 不够，因为导航、对齐或抓取失败后巡视节点也可能前进。`UGV_GRASP=false` 只用于导航诊断，汇报时必须注明未执行抓取。

整个过程中，任何 `gnome-terminal` 标签内都不应有持续报错/异常。文首视频仅为 UAV 建图/地形可视化片段；完整任务以本节实时检查为准。

## 十一、停止程序

在仓库根目录新开终端并执行：

```bash
bash stop_airground.sh
```

**同一用户下不要并行运行其他 ROS/PX4 会话。** 启动与停止脚本会宽范围清理旧仿真状态：`stop_airground.sh` 会关闭当前 ROS master 上的全部节点，也可能结束本用户的其他 Gazebo、ROS、RViz、PX4 或 MAVROS 进程。请先保存无关工作。

脚本先请求 ROS 退出，再结束匹配的 Gazebo、PX4、MAVROS、规划、检测和抓取进程；不会删除地图、模型、源码或编译产物。Gazebo 窗口关闭，并且下面的命令没有匹配输出，即表示停止完成：

```bash
pgrep -af 'gzserver|gzclient|rosmaster|px4|mavros_node'
```

如果上一次运行异常退出，也应先执行一次 `bash stop_airground.sh`，再重新启动。

## 十二、常见问题

- **构建时提示找不到 `ego-planner`：** 运行 `bash setup_uav.sh`，或按安装脚本的方式把它克隆到 `src/ego-planner/` 并应用 `patches/` 中的补丁。
- **没有 TensorRT 可执行文件 / CUDA 报错：** 使用 `UAV_DETECT_BACKEND=cpu UAV_DETECT_DEVICE=cpu` 运行；并确认已安装上述 Python 依赖。
- **没有检测结果：** 先检查 RGB、深度、camera_info 话题，再查看 `/mine_camera/diagnostics` 与 `/mine_detection/debug_image`。
- **UGV 不动：** 必须先有目标被确认，才能调用 `/ugv/start_tour`；检查 `/detected_targets`、`/ugv/tour_status`、`/state_estimation` 与 `/terrain_map`。
- **高程图是空的：** 用 `rosrun tf2_ros tf2_echo odom uav0/map_local` 验证锁存的变换，并检查无人机点云话题 `/uav0/mapping/velodyne_points_gated`。
- **抓取阶段失败或表现不稳定：** 设置 `UGV_GRASP=false` 可先只运行导航诊断。启用抓取时，重点检查日志中的 `physical mine selected: landmine_*`、`welded ...`，以及 `/gazebo/model_states`、`/grasp/execute`、`/grasp/place`。[`docs/grasp_PLAN.md`](docs/grasp_PLAN.md) 是历史排查记录，不代表当前验收状态。
- **CMake 提示没有 `libgpd`：** 对默认可复现方案而言这是正常警告。保持 `GRASP_SOURCE=analytic`，`gpd_ros` 仍会生成 ROS 消息，工作空间可以继续编译；GPD 只是可选对比后端，不是默认依赖。

## 致谢

本项目基于 ROS、Gazebo Classic、PX4、MAVROS、EGO-Planner、ANYbotics `elevation_mapping` / ETH `grid_map`、Clearpath Husky、Universal Robots UR5、Robotiq、MoveIt、GPD、DLIO、Ultralytics YOLO，以及项目中包含或引用的 Gazebo 模型集合构建而成。
