# Algorithm Research References

> 课程设计参考论文与算法文献 —— 按技术模块分类
> 检索日期：2026-06-11

---

## A. 高程地图构建 — Elevation Map Construction from 3D Point Clouds

### 核心论文

#### A1. Probabilistic Terrain Mapping for Mobile Robots with Uncertain Localization
- **Authors:** Péter Fankhauser, Michael Bloesch, Marco Hutter
- **Venue:** IEEE Robotics and Automation Letters (RA-L), Vol. 3, No. 4, pp. 3019–3026, 2018
- **DOI:** `10.1109/LRA.2018.2849506`
- **贡献:** 提出将 3D LiDAR/点云测量概率性融合为 2.5D 栅格地图的方法，同时考虑定位不确定性。每个栅格存储高程估计和方差，通过 Bayes 更新增量融合新测量。定位协方差驱动方差增长——这是 `elevation_mapping` ROS 包的理论基础。
- **与本项目关系:** 直接对应 UAV 3D 点云 → UGV 高程栅格地图转换步骤。

#### A2. Robot-Centric Elevation Mapping with Uncertainty Estimates
- **Authors:** Péter Fankhauser, Michael Bloesch, Christian Gehring, Marco Hutter, Roland Siegwart
- **Venue:** International Conference on Climbing and Walking Robots (CLAWAR), 2014
- **DOI:** `10.3929/ethz-a-010173654`
- **贡献:** 提出"以机器人为中心"的高程建图概念——地图随机器人移动，仅维护局部区域，避免全局地图的内存和漂移问题。
- **与本项目关系:** UGV 机载 LiDAR 局部高程地图的构建方法参考。

### ROS 软件包

| 包 | 来源 | 功能 |
|----|------|------|
| `elevation_mapping` | `github.com/ANYbotics/elevation_mapping` (BSD-3) | 从 3D 点云生成 2.5D 高程栅格地图（ROS 1 Noetic，源码编译） |
| `grid_map` | `sudo apt install ros-noetic-grid-map` | 通用多层栅格地图库，支持高程/法向量/粗糙度/可通行性等多层叠加 |
| `grid_map_pcl` | 上述 grid_map 的子包 | 将 3D PCL 点云转换为 2.5D 栅格，处理多高程层（植被/树冠过滤——取最低聚类中心为地面高程） |

### 关键算法：Multi-Elevation Handling (`grid_map_pcl`)

当 UAV 俯视采集的点云中包含树冠、建筑物等高空结构时，每个 (x,y) 栅格可能包含多个高程值。`grid_map_pcl` 的处理流程：
1. 点云降采样 + 离群点过滤
2. 对每个 (x,y) 列内的点进行 Z 轴欧几里得聚类
3. **取最低聚类中心作为地面高程**（对森林/杂乱环境鲁棒）

### 高程地图相关其他方法

| 论文 | 年份 | 贡献 |
|------|------|------|
| Bayer & Faigl, "Terrain Modeling in 3D Mapping for Autonomous Navigation in Multi-Layered Environments," Springer, 2025 | 2025 | 将 2.5D 高程地图扩展为支持多层结构（天桥/隧道）的高分辨率 3D 地图 |
| Fu et al., "Traversability Analysis of Quadruped Robot Based on Sparse Point Cloud in Rough Terrain," 2022 | 2022 | 从稀疏点云提取四类地形特征：台阶高度、表面坡度、表面粗糙度、植被密度 |

---

## B. 地形可通行性分析 — Terrain Traversability Estimation

### 核心论文

#### B1. Navigation Planning for Legged Robots in Challenging Terrain
- **Authors:** Martin Wermelinger, Péter Fankhauser, Remo Diethelm, Philipp Krüsi, Roland Siegwart, Marco Hutter
- **Venue:** 2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)
- **DOI:** `10.1109/IROS.2016.7759199`
- **贡献:** 从高程地图计算**坡度（slope）、粗糙度（roughness）、台阶高度（step height）**三类地形特征，构建可通行性地图（traversability map），然后用 RRT* 采样规划器优化路径长度和安全性。在四足机器人 StarlETH 上验证。
- **与本项目关系:** **本项目的核心算法参考**——我们将此方法适配到轮式 Husky UGV（参数化不同，但三种地形特征的计算方法直接复用）。

**三种特征的计算方法：**
1. **坡度（Slope）：** 从表面法向量与垂直轴的夹角计算：`slope = arccos(normal_z)`，设最大可通行坡度阈值（如 25°）
2. **粗糙度（Roughness）：** 邻域内高度的标准差
3. **台阶高度（Step Height）：** 相邻栅格的最大高程差

**可通行性代价公式（通用模式）：**
```
t_ij = α · (1 − slope/s_max) + β · (1 − roughness/r_max)
```
超过台阶高度阈值的区域直接标记为不可通行。

#### B2. A Survey of Traversability Estimation for Mobile Robots
- **Authors:** Christoforos Sevastopoulos, Stasinos Konstantopoulos
- **Venue:** arXiv:2204.10883, 2022
- **贡献:** 全面综述——覆盖基于几何（高程图、坡度、粗糙度、台阶高度）、传统 ML、和深度学习的可通行性估计方法。**最适合作为课程报告中的 survey 引用。**
- **与本项目关系:** 为我们的"基于几何的可通行性分析"方法选择提供文献支撑。

#### B3. Terrain Traversability Analysis Methods for Unmanned Ground Vehicles: A Survey
- **Authors:** P. Papadakis
- **Venue:** Engineering Applications of Artificial Intelligence, Vol. 26, Issue 4, pp. 1373–1385, 2013
- **贡献:** UGV 地形可通行性分析的奠基性综述，覆盖基于几何、视觉和混合方法的可通行性评估。
- **与本项目关系:** 经典引用，论证几何方法的成熟性和可靠性。

### ROS 软件包

| 包 | 来源 | 功能 |
|----|------|------|
| `traversability_estimation` | `github.com/leggedrobotics/traversability_estimation` | Wermelinger 等人 IROS 2016 论文的开源实现，包含 4 种滤波器：表面法向量、坡度、粗糙度、台阶高度 |
| `grid_map_filters` | `ros-noetic-grid-map` 的子包 | YAML 配置的滤波器链：法向量计算 → 孔洞填充 → 平滑 → 数学表达式滤波（边缘检测、粗糙度、可通行性评分）→ 阈值分类 |

### 基于 Husky 的实际验证

#### B4. Local Traversability Assessment in an Unmanned Ground Vehicle: An Analysis of Mobility on the UGV Husky
- **Authors:** Getahun
- **Venue:** KTH Master's Thesis, 2023
- **贡献:** 从 Husky 的几何尺寸推导出**坡度、粗糙度、台阶高度的几何阈值公式**。在真机和仿真上对比验证。**直接适用于本项目的 Husky 平台。**

### 2022 年后的趋势

| 方法 | 代表论文 | 特点 |
|------|---------|------|
| **Hybrid (几何 + 语义)** | Leung et al., "Hybrid Terrain Traversability Analysis in Off-road Environments," 2022 | LiDAR 几何特征 + 相机语义分割（草地/泥地/沥青分类），融合判断可通行性 |
| **Deep Learning / Self-Supervised** | 多个 2023–2025 工作 | 从经验学习可通行性，减少对手动调参的依赖 |
| **Multi-modal Fusion** | "Traversability Analysis for UGVs Based on Multi-modal Information Fusion," IEEE, 2023 | LiDAR + 相机 + IMU 的多模态融合标准 |
| **Benchmark** | Yang et al., "Benchmark and Analysis of Autonomous Robot Path Planning Performance in Rough Terrain," 2025 | 使用 2.5D 高程地图 + slope/roughness 对 DWA、TEB、RPP、MPPI 四种局部规划器在崎岖地形上进行基准测试 |

---

## C. 空-地协同 SLAM / 建图 — Air-Ground Collaborative Mapping

### 核心综述论文

#### C1. Air-Ground Collaborative Robots for Fire and Rescue Missions: Towards Mapping and Navigation Perspective
- **Authors:** Ying Zhang, Haibao Yan, et al.
- **Venue:** arXiv:2412.20699, December 2024 (18 pages, 20 figures)
- **贡献:** 系统综述空-地协作机器人的**建图和导航**方向。提出了 UAV 建图 + UGV 导航的耦合框架。覆盖三个核心技术领域：UAV 建图、UAV/UGV 协同定位、UGV 导航。按 UAV/UGV 数量分类系统架构。
- **与本项目关系:** **直接对标**——本项目的场景（UAV 建图 → UGV 导航）与该综述的框架完全一致，可作为报告中的顶层架构引用。

#### C2. Heterogeneous Agents, Unified Missions: A Survey and Taxonomy on Air–Ground Cooperative Systems
- **Authors:** Yusong Zhou, Jin Zhao, Nianyi Sun, et al.
- **Venue:** Robotics and Autonomous Systems (Elsevier), 2025/2026 (online)
- **覆盖期:** 2015–2025
- **贡献:** 提出三层 AGHS 架构：**决策层 → 实施层 → 应用层**。覆盖技术包括：定位、路径规划、感知、多源融合、任务分配、通信。提出部署分类：L1（集中式）、L2（混合式）、L3（分布式自治）。
- **与本项目关系:** 为异构机器人团队的架构设计提供理论依据。

#### C3. 视觉空地协作SLAM综述 (Visual Air-Ground Cooperative SLAM Review)
- **Authors:** (CETC Wuhu Diamond Aircraft & Shenyang Aerospace University)
- **Venue:** 电光与控制 (Electronics Optics & Control), 2025, Vol. 32, No. 11, p. 71
- **贡献:** 综述视觉 SLAM（ORB-SLAM、VINS 等）在空地异构机器人团队中的应用。分析视觉 SLAM 相对于 LiDAR 在多智能体协作中的优势。
- **与本项目关系:** 如果 UAV 使用视觉建图方案（如视觉里程计 + 点云重建），可引用。

### 协同 SLAM 框架（技术参考）

| 框架 | 特点 | 适用性 |
|------|------|--------|
| **CCM-SLAM** | 集中式协同 SLAM，多 UAV 共享地图 | 中心节点管理全局地图的场景 |
| **COVINS** | 视觉-惯性协同 SLAM，支持多智能体 | UAV + UGV 共享视觉特征 |
| **Swarm-SLAM** | 去中心化，鲁棒性强 | 通信不可靠的室外场景 |
| **ColAG** | Collaborative Air-Ground framework for perception-limited UGVs | 直接对标：UAV 弥补 UGV 感知盲区 |

### 代表性的应用研究

#### C4. A Distributed Multi-Robot Collaborative SLAM Method Based on Air–Ground Cross-Domain Cooperation
- **Venue:** MDPI Drones, 2025, Vol. 9, No. 7, 504
- **贡献:** 提出基于空地跨域协作的分布式多机器人协同 SLAM 方法。

#### C5. Air-Ground Collaboration With SPOMP: Semantic Panoramic Online Mapping and Planning
- **Venue:** IEEE Transactions on Field Robotics, 2024
- **DOI:** `10.1109/...`
- **贡献:** 语义全景在线建图与规划——UAV 提供语义标注的全景地图，UGV 使用该地图进行语义感知导航。

#### C6. Research on Disaster Environment Map Fusion Construction and Reinforcement Learning Navigation Technology Based on Air–Ground Collaborative Multi-Heterogeneous Robot Systems
- **Venue:** MDPI Sensors, 2025, Vol. 25, No. 16, 4988
- **贡献:** 灾后环境空地协同多异构机器人建图融合 + 强化学习导航。

---

## D. GPS-IMU-里程计多源融合定位 — Multi-Sensor Fusion for Outdoor Localization

### 核心方法

#### D1. robot_localization: Dual-EKF with navsat_transform_node

虽然 `robot_localization` 包本身没有对应的单一学术论文，但其融合方法基于经典的 EKF 多传感器融合理论：

- **基础理论:** S. Thrun, W. Burgard, D. Fox, *Probabilistic Robotics*, MIT Press, 2005（Chapter 3: Gaussian Filters, Chapter 7: Mobile Robot Localization）
- **ROS 实现:** T. Moore, D. Stouch, *robot_localization* package, `github.com/cra-ros-pkg/robot_localization`
- **关键方法:** 双 EKF 架构 + navsat_transform_node 将 GPS WGS84 坐标转换为 UTM 局部笛卡尔坐标系

### 双 EKF 架构

```
本地 EKF (ekf_odom)
├── 世界坐标系: odom
├── 输入: 轮式里程计 (velocity) + IMU (orientation, angular velocity, linear acceleration)
├── 输出: odom → base_link (平滑、局部精确)
└── GPS: 不融合（避免 GPS 跳变污染局部位姿）

navsat_transform_node
├── 输入: /gps/fix (NavSatFix) + /imu/data + /odometry/filtered (本地 EKF 输出)
├── 功能: WGS84 (lat/lon/alt) → UTM 笛卡尔坐标
└── 输出: /odometry/gps (nav_msgs/Odometry)

全局 EKF (ekf_map)
├── 世界坐标系: map
├── 输入: 轮式里程计 + IMU + /odometry/gps
├── 输出: map → odom (全局漂移修正)
└── 特点: two_d_mode: false（3D 模式，支持高程估计）
```

### 系统验证与工程参考

#### D2. VAULT: A Mobile Mapping System for ROS 2-based Autonomous Robots
- **Authors:** (multiple)
- **Venue:** arXiv:2506.09583, 2025
- **贡献:** ROS 2 的移动建图系统，融合 GNSS + 视觉-惯性里程计 + IMU 通过 EKF，使用 RTAB-Map 实现室外 SLAM。
- **与本项目关系:** 室外多传感器融合的工程实践参考。

#### D3. CMU MRSD 2026 Localization Subsystem
- **Venue:** CMU MRSD Project, 2026
- **贡献:** 在 Clearpath Warthog 上使用 `navsat_transform_node` + 双 EKF 融合 GPS + 轮式里程计 + IMU，用于农业种植场景。
- **与本项目关系:** 与本项目相同的平台（Clearpath 系列）+ 相同的定位架构。

#### D4. Husarion UGV Localization
- **包:** `husarion_ugv_localization` (ROS index)
- **贡献:** 出厂即提供双 EKF GPS 融合配置——工程化的最佳实践参考。

### 常见问题与对策

| 问题 | 根因 | 对策 |
|------|------|------|
| GPS 报告低协方差但实际偏移数米 | 传感器"欺骗"滤波器 | 膨胀 GPS 测量协方差；拒绝协方差 > 阈值的 fix |
| IMU yaw 与 GPS 推算航向矛盾 | 磁力计未校准 / 磁偏角错误 | 双 EKF 配置中关闭 IMU yaw 融合，改用 GPS 位置差分推算航向 |
| 冷启动时轨迹"荒谬" | UTM 坐标系收敛需时间 | 前几秒仅输出本地 EKF 结果，等待全局 EKF 收敛 |
| REP-103 朝向约定不匹配 | IMU yaw=0 指向北而非东（REP-103 要求） | 配置 `yaw_offset: 1.5707963` |

---

## E. 地形感知导航：代价图层集成 — Terrain-Aware Navigation Costmap Integration

### 核心论文

#### E1. Layered Costmaps for Context-Sensitive Navigation
- **Authors:** David V. Lu, Dave Hershberger, William D. Smart
- **Venue:** 2014 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Chicago
- **DOI:** `10.1109/IROS.2014.6942636`
- **Citations:** ~188
- **贡献:** 提出**分层代价地图（Layered Costmap）**架构——将单一的整体代价图拆分为语义独立的有序图层列表，通过两阶段更新（`updateBounds` + `updateValues`）合成最终代价图。在 PR2 + Gazebo 上验证。
- **与本项目关系:** **本项目架构设计的理论基础**——我们将地形可通行性作为一个独立的 `costmap_2d::Layer` 插件加入层级代价地图，与其他图层（obstacle, static_map, inflation）并行工作。

### 两种技术路线对比

| | **方案 A: grid_map_costmap_2d 桥接** | **方案 B: mesh_navigation** |
|---|---|---|
| **核心思想** | 将 3D 地形可通行性"降维"为 2D 栅格代价图层 | 用 3D 三角形网格代替 2D 栅格，直接在网格面上规划 |
| **代价计算** | 坡度 > 阈值 → 在 2D costmap 中标记为高代价/致命障碍 | 在每个网格顶点直接计算几何属性（坡度、粗糙度、高度差、净空） |
| **规划器** | 标准 move_base（全局 A* + 局部 DWA） | Move Base Flex + CVP/DijkstraMeshPlanner |
| **ROS 兼容性** | ROS 1 Noetic 原生支持 | ROS 2 为主（ROS 1 `noetic` 分支可用但不活跃） |
| **工程复杂度** | 低—中（使用现有包或写一个 Layer 插件） | 高（需要迁移至 MBF 框架） |
| **适用性** | 本项目首选——快速实现，标注 move_base 生态 | 更强大但超过课程设计时间预算 |

### 地形 → 代价转换函数

参考 M4 多模态机器人系统的实际实现：

```
C(T) = 0                                    if T ≥ T_high (0.85)
C(T) = |20 × (T_high − T) / (T_high − T_crit)|   if T_crit ≤ T < T_high
C(T) = |100 − 80 × T / T_crit|                  if T < T_crit (0.60)
```

其中：
- `T_high = 0.85` → 完全可通行（cost = 0）
- `T_crit = 0.60` → 危险地形边界
- `T < 0.60` → cost 范围 20–100（致命障碍）

### 全局规划器的地形代价

A* 边代价（参考 M4 系统的 Energy-Aware A*）：

```
c_ij = d_ij · (1 + w_t · T_ij + C_d)
```

- `d_ij`: 欧几里得距离（直线 1.0，对角线 1.414）
- `T_ij`: costmap 中归一化的可通行性代价
- `w_t = 20`: 地形惩罚权重（可调参数）
- `C_d`: 单位距离能耗

### 可通行性计算流程（grid_map_filters 滤波器链）

```yaml
# 基于 YAML 配置的滤波器链（不写 C++ 代码的方案）
filter_chain:
  - type: SurfaceNormalFilter        # 表面法向量 → 坡度
  - type: InpaintFilter              # 孔洞填充
  - type: MeanFilter                 # 平滑/模糊
  - type: MathExpressionFilter       # 粗糙度 = |高程 - 平滑高程|
  - type: MathExpressionFilter       # 可通行性 = f(slope, roughness)
  - type: ThresholdFilter            # 二值化可通行/不可通行
```

---

## F. 汇总：本项目可引用的论文清单

### 课程报告推荐引用（按重要程度排列）

| # | 论文 | 引用场景 |
|----|------|---------|
| 1 | Fankhauser et al., "Probabilistic Terrain Mapping for Mobile Robots with Uncertain Localization," RA-L 2018 | §3 高程地图构建方法 |
| 2 | Wermelinger et al., "Navigation Planning for Legged Robots in Challenging Terrain," IROS 2016 | §3 地形可通行性分析方法 |
| 3 | Sevastopoulos & Konstantopoulos, "A Survey of Traversability Estimation for Mobile Robots," arXiv 2022 | §2 相关工作综述 |
| 4 | Lu et al., "Layered Costmaps for Context-Sensitive Navigation," IROS 2014 | §3 代价地图分层架构 |
| 5 | Zhang et al., "Air-Ground Collaborative Robots for Fire and Rescue Missions," arXiv 2024 | §1 引言与系统架构 |
| 6 | Zhou et al., "Heterogeneous Agents, Unified Missions: A Survey and Taxonomy on Air–Ground Cooperative Systems," RAS 2025 | §2 相关工作综述 |
| 7 | Thrun, Burgard, Fox, *Probabilistic Robotics*, MIT Press 2005 | §3 多传感器融合理论基础 |
| 8 | Fankhauser et al., "Robot-Centric Elevation Mapping with Uncertainty Estimates," CLAWAR 2014 | §3 高程地图构建方法 |
| 9 | Getahun, "Local Traversability Assessment on the UGV Husky," KTH Thesis 2023 | §4 平台相关参数设定 |
| 10 | Papadakis, "Terrain Traversability Analysis Methods for UGVs: A Survey," EAAI 2013 | §2 经典综述引用 |

### BibTeX 参考

```bibtex
@article{fankhauser2018probabilistic,
  title={Probabilistic Terrain Mapping for Mobile Robots with Uncertain Localization},
  author={Fankhauser, P{\'e}ter and Bloesch, Michael and Hutter, Marco},
  journal={IEEE Robotics and Automation Letters},
  volume={3},
  number={4},
  pages={3019--3026},
  year={2018},
  doi={10.1109/LRA.2018.2849506}
}

@inproceedings{wermelinger2016navigation,
  title={Navigation Planning for Legged Robots in Challenging Terrain},
  author={Wermelinger, Martin and Fankhauser, P{\'e}ter and Diethelm, Remo and Kr{\"u}si, Philipp and Siegwart, Roland and Hutter, Marco},
  booktitle={2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year={2016},
  doi={10.1109/IROS.2016.7759199}
}

@article{sevastopoulos2022survey,
  title={A Survey of Traversability Estimation for Mobile Robots},
  author={Sevastopoulos, Christoforos and Konstantopoulos, Stasinos},
  journal={arXiv preprint arXiv:2204.10883},
  year={2022}
}

@inproceedings{lu2014layered,
  title={Layered Costmaps for Context-Sensitive Navigation},
  author={Lu, David V and Hershberger, Dave and Smart, William D},
  booktitle={2014 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year={2014},
  doi={10.1109/IROS.2014.6942636}
}

@article{zhang2024airground,
  title={Air-Ground Collaborative Robots for Fire and Rescue Missions: Towards Mapping and Navigation Perspective},
  author={Zhang, Ying and Yan, Haibao and others},
  journal={arXiv preprint arXiv:2412.20699},
  year={2024}
}

@article{zhou2025heterogeneous,
  title={Heterogeneous Agents, Unified Missions: A Survey and Taxonomy on Air--Ground Cooperative Systems},
  author={Zhou, Yusong and Zhao, Jin and Sun, Nianyi and others},
  journal={Robotics and Autonomous Systems},
  year={2025},
  publisher={Elsevier}
}

@book{thrun2005probabilistic,
  title={Probabilistic Robotics},
  author={Thrun, Sebastian and Burgard, Wolfram and Fox, Dieter},
  year={2005},
  publisher={MIT Press}
}

@inproceedings{fankhauser2014robotcentric,
  title={Robot-Centric Elevation Mapping with Uncertainty Estimates},
  author={Fankhauser, P{\'e}ter and Bloesch, Michael and Gehring, Christian and Hutter, Marco and Siegwart, Roland},
  booktitle={International Conference on Climbing and Walking Robots (CLAWAR)},
  year={2014},
  doi={10.3929/ethz-a-010173654}
}

@mastersthesis{getahun2023local,
  title={Local Traversability Assessment in an Unmanned Ground Vehicle: An Analysis of Mobility on the UGV Husky},
  author={Getahun},
  school={KTH Royal Institute of Technology},
  year={2023}
}

@article{papadakis2013terrain,
  title={Terrain Traversability Analysis Methods for Unmanned Ground Vehicles: A Survey},
  author={Papadakis, P},
  journal={Engineering Applications of Artificial Intelligence},
  volume={26},
  number={4},
  pages={1373--1385},
  year={2013}
}
```

---

## G. 开源仓库对照表 — Paper ↔ Open-Source Repository Mapping

> **关键结论：本项目的核心技术栈均有成熟的开源仓库支撑。**

### 可直接使用的仓库（ROS Noetic 原生支持，apt 或源码编译）

| # | 论文 | 开源仓库 | ⭐ Stars | 许可证 | ROS Noetic | 维护状态 | 用途 |
|---|------|---------|---------|--------|-----------|---------|------|
| 1 | Fankhauser et al. RA-L 2018 | [`ANYbotics/elevation_mapping`](https://github.com/ANYbotics/elevation_mapping) | ~1.2k | BSD-3 | ✅ 源码编译 | ⚠️ 不再活跃维护 | UAV 点云 → 2.5D 高程栅格 |
| 2 | — (ETH grid_map 框架) | [`ANYbotics/grid_map`](https://github.com/ANYbotics/grid_map) | ~800 | BSD-3 | ✅ apt (`ros-noetic-grid-map`) | ✅ 维护中 | 多层栅格地图（高程/坡度/粗糙度/可通行性） |
| 3 | Wermelinger et al. IROS 2016 | [`leggedrobotics/traversability_estimation`](https://github.com/leggedrobotics/traversability_estimation) | ~200 | BSD-3 | ✅ 源码编译 | ⚠️ 不再活跃维护 | 坡度/粗糙度/台阶高度滤波 → 可通行性地图 |
| 4 | Lu et al. IROS 2014 | ROS `costmap_2d` (内置于 `navigation` 包) | — | BSD | ✅ apt | ✅ 维护中 | 分层代价地图框架（Layer 插件架构） |
| 5 | Thrun et al. 2005 理论基础 | [`cra-ros-pkg/robot_localization`](https://github.com/cra-ros-pkg/robot_localization) | ~1.7k | BSD (ROS1) / Apache-2.0 (ROS2) | ✅ apt | ✅ 活跃维护 | GPS+IMU+Odom 双 EKF 融合，navsat_transform |
| 6 | — (GPS Gazebo 仿真) | [`tu-darmstadt-ros-pkg/hector_gazebo`](https://github.com/tu-darmstadt-ros-pkg/hector_gazebo) | ~200 | BSD | ✅ apt (`ros-noetic-hector-gazebo-plugins`) | ✅ 维护中 | Gazebo GPS 仿真插件 |
| 7 | — (Clearpath 仿真世界) | [`clearpathrobotics/cpr_gazebo`](https://github.com/clearpathrobotics/cpr_gazebo) | ~100 | BSD | ✅ 源码编译 (`noetic-devel`) | ✅ 维护中 | Inspection World 等室外仿真环境 |
| 8 | — (SLAM 工具箱) | [`SteveMacenski/slam_toolbox`](https://github.com/SteveMacenski/slam_toolbox) | ~1.5k | LGPL-2.1 | ✅ apt | ✅ 活跃维护 | 本仓库已在使用——室内外通用 2D SLAM |
| 9 | — (move_base 导航) | ROS `navigation` 元包 | — | BSD | ✅ apt | ✅ 维护中 | 本仓库已在使用——全局/局部规划器 |

### ROS 2 仓库（本项目的 ROS 1 技术栈无法直接使用，但可作为算法参考）

| # | 论文/项目 | 开源仓库 | ⭐ Stars | ROS 版本 | 用途 |
|---|----------|---------|---------|---------|------|
| 10 | Pütz et al. (mesh navigation) | [`naturerobots/mesh_navigation`](https://github.com/naturerobots/mesh_navigation) | ~860 | ROS 2 Humble/Jazzy (Noetic 分支不维护) | 3D 网格面上的坡度/粗糙度感知导航——比 costmap 方案更强大但需 ROS 2 |
| 11 | Li et al. ICRA 2024 | [`SiChiTong/ColAG`](https://github.com/SiChiTong/ColAG) | ~70 | ROS 1 Noetic (Ubuntu 20.04) | 空地协同框架——UAV 为盲 UGV 提供地图，VRPTW 调度 |
| 12 | Zhang et al. arXiv 2024 | [`fast-fire`](https://github.com/fast-fire) 组织下 | — | ROS | 空地协作消防与救援机器人 |

### 仓库质量评估

| 仓库 | 本项目的可用性 | 说明 |
|------|-------------|------|
| **robot_localization** | ✅ **直接可用** | apt 安装，已在 `spawn_robot.launch` 中使用本地 EKF，只需加配 navsat_transform + 全局 EKF |
| **grid_map** | ✅ **直接可用** | apt 安装，含 grid_map_costmap_2d、grid_map_rviz_plugin、grid_map_filters 全部子包 |
| **costmap_2d (layered)** | ✅ **直接可用** | 已在使用中，只需新写一个 terrain layer 插件（继承 `costmap_2d::Layer`） |
| **elevation_mapping** | ⚠️ **需源码编译** | Noetic 有几个已知坑（filter_base.hpp 路径、TBB 链接），但社区文档完善 |
| **traversability_estimation** | ⚠️ **需源码编译** | 依赖 elevation_mapping + grid_map，编译链较长；可简化为 grid_map_filters YAML 方案替代 |
| **cpr_gazebo** | ✅ **直接可用** | git clone + catkin_make，Husky 原生支持 |
| **mesh_navigation** | ❌ **不可直接用** | ROS 2 主力，Noetic 分支停止维护。可引用论文方法（Steepness/Roughness/HeightDiff layer 概念） |
| **ColAG** | ⚠️ **部分可用** | ROS 1 Noetic，但依赖三个独立 workspace 和 EGO-Swarm/MARSIM 无人机仿真栈——集成工作量大 |

### 推荐的本项目技术栈仓库组合

```
本项目的开源仓库依赖：

已有（本仓库）:
├── src/robotiq/          ← Robotiq 夹爪（含 2F-140）
├── src/gazebo-pkgs/      ← Gazebo 抓取插件
├── src/aws-robomaker-small-house-world/  ← 室内仿真世界（将替换）
└── src/mobile_manipulator/  ← MasterControl, YOLO, agent, flask_ui

新增（apt 安装）:
├── ros-noetic-grid-map           ← 高程栅格地图 + costmap 桥接 + 滤波器链
├── ros-noetic-robot-localization ← 双 EKF + navsat_transform（已安装）
├── ros-noetic-hector-gazebo-plugins ← GPS 仿真
├── ros-noetic-gps-umd            ← GPS 消息类型

新增（源码编译）:
├── src/elevation_mapping   ← ANYbotics (3D 点云 → 2.5D 高程)
├── src/kindr               ← elevation_mapping 依赖
├── src/kindr_ros           ← elevation_mapping 依赖
├── src/message_logger      ← elevation_mapping 依赖
└── src/cpr_gazebo          ← Clearpath Inspection World (noetic-devel)
```

---

> 📅 生成日期：2026-06-11
> 🔍 检索工具：WebSearch + WebFetch + 交叉验证
> 📝 文档用途：课程设计报告算法参考与开源仓库索引
