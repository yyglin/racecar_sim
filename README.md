# FSAC LiDAR 地面分割与锥桶检测

本项目面向大学生无人驾驶方程式赛车（FSAC），使用 ROS 2 Jazzy、PCL 和
Gazebo Harmonic 完成 LiDAR 地面分割，并在非地面点云上继续进行锥桶聚类识别。

当前已经完成地面分割、Gazebo 点云桥接、RViz2 可视化和 Ackermann 车辆运动；
锥桶检测目前处于流程设计与实现阶段。

## 1. 系统流程

```text
rosbag 或 Gazebo LiDAR
        ↓
PointCloud2 预处理
        ↓
极坐标分段与局部地面线拟合
        ↓
ground_points + nonground_points
        ↓
自适应聚类与原始点云重建
        ↓
锥桶位置、候选点云与 RViz 标记
```

算法节点只依赖 ROS 2 话题接口。rosbag 与 Gazebo 共用同一套算法，仅通过参数切换输入话题。

## 2. 项目结构

```text
.
├── compose.yaml
├── docker/
│   ├── Dockerfile.algorithm
│   └── Dockerfile.simulation
├── Fast_Segmentation_ws/
│   └── src/fast_ground_segmenter/
│       ├── config/
│       ├── include/fast_ground_segmenter/
│       ├── launch/
│       ├── src/
│       └── test/
├── simulation_ws/
│   └── src/
│       ├── racecar_description/
│       └── racecar_gazebo/
└── data/                         # rosbag 数据，不提交到仓库
```

| 包 | 职责 |
|---|---|
| `fast_ground_segmenter` | 过滤点云、拟合局部地面并输出地面/非地面点 |
| `racecar_description` | 保存赛车、LiDAR 和锥桶模型资源 |
| `racecar_gazebo` | 保存世界、桥接配置、启动文件和键盘遥控节点 |
| `cone_detection` | 规划中的锥桶聚类、重建与筛选包 |

## 3. 快速开始

### 3.1 启动容器

在项目根目录执行：

```bash
docker compose up -d --build algorithm simulation
```

进入算法或仿真容器：

```bash
docker compose exec algorithm bash
docker compose exec simulation bash
```

项目根目录统一挂载到容器内：

```text
. → /Fast_Segmentation_of_3D_Point_Clouds_for_Ground_Vehicles
./data → /data（只读）
```

执行 `docker compose exec algorithm bash` 后会直接进入
`/Fast_Segmentation_of_3D_Point_Clouds_for_Ground_Vehicles/Fast_Segmentation_ws`；
执行 `docker compose exec simulation bash` 后会直接进入
`/Fast_Segmentation_of_3D_Point_Clouds_for_Ground_Vehicles/simulation_ws`。

### 3.2 编译工作区

算法容器：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

仿真容器：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3.3 使用 rosbag 验证地面分割

启动 rosbag 播放和算法节点：

```bash
ros2 launch fast_ground_segmenter bag_ground_segmentation.launch.py \
  bag_path:=/data/rosbag2_2022_04_14-trucks \
  play_rate:=1.0
```

也可以分别播放 rosbag 和运行节点：

```bash
ros2 bag play /data/rosbag2_2022_04_14-trucks --loop

ros2 run fast_ground_segmenter ground_segmenter_node --ros-args \
  -p input_topic:=/sensing/lidar/top/rectified/pointcloud
```

### 3.4 启动 Gazebo 联合仿真

在已加载算法与仿真工作区的终端中执行：

```bash
ros2 launch fast_ground_segmenter gazebo_ground_segmentation.launch.py
```

无桌面环境时：

```bash
ros2 launch fast_ground_segmenter gazebo_ground_segmentation.launch.py \
  gui:=false headless:=true rviz:=false
```

键盘控制节点：

```bash
ros2 run racecar_gazebo keyboard_teleop
```

| 按键 | 功能 |
|---|---|
| `↑` / `↓` | 增加前进速度 / 降速并进入倒车 |
| `←` / `→` | 左转 / 右转 |
| `C` | 转向回中 |
| `Space` 或 `S` | 停车 |
| `Q` 或 `Esc` | 停车并退出 |

## 4. 地面分割

### 4.1 处理流程

```text
PointCloud2
    → 删除 NaN/Inf
    → 距离与高度 ROI 过滤
    → 极坐标分段、分桶
    → 提取每桶最低点
    → 分段拟合局部地面线
    → 逐点分类
```

点的极坐标为：

$$
r=\sqrt{x^2+y^2}, \qquad \theta=\operatorname{atan2}(y,x)
$$

每个角度段内使用多个局部模型拟合地面：

$$
z=ar+b
$$

当点与对应地面线的高度差小于阈值时判定为地面，否则判定为非地面。

### 4.2 模块职责

| 模块 | 职责 |
|---|---|
| `ground_segmenter_node` | 处理 ROS 2 消息、读取参数并组织算法流程 |
| `point_cloud_filter` | 删除无效点并执行 ROI 过滤 |
| `polar_grid` | 将点映射到角度段和距离桶 |
| `ground_line_fitter` | 拟合局部地面线并分类点云 |
| `types.hpp` | 定义桶、地面线和点索引等数据结构 |

核心算法类尽量不依赖 ROS 2，便于单元测试和复用。

### 4.3 ROS 2 接口

节点名称：`ground_segmenter_node`

| 话题 | 用途 |
|---|---|
| 参数 `input_topic` | 原始 LiDAR 点云输入 |
| `/ground_points` | 地面点云 |
| `/nonground_points` | 非地面点云，供锥桶聚类使用 |
| `/debug/filtered_points` | ROI 过滤后的点云 |
| `/debug/bin_min_points` | 各极坐标桶的最低点 |

点云订阅和发布使用 `rclcpp::SensorDataQoS()`，所有输出保留输入消息的时间戳和
`frame_id`。

### 4.4 主要参数

参数文件位于：

```text
Fast_Segmentation_ws/src/fast_ground_segmenter/config/ground_segmenter.yaml
```

```yaml
ground_segmenter_node:
  ros__parameters:
    input_topic: /sensing/lidar/top/rectified/pointcloud

    min_range: 1.0
    max_range: 80.0
    min_z: -3.0
    max_z: 2.0

    num_segments: 360
    num_bins: 120

    max_slope_deg: 10.0
    max_fit_error: 0.15
    max_start_height_error: 0.30
    long_threshold: 2.0
    max_long_height_error: 0.10
    max_point_to_line_distance: 0.20
```

这些值是初始配置，rosbag 和 Gazebo 分别使用独立参数文件进行调试。

### 4.5 验收要点

- 道路主体进入 `/ground_points`；
- 锥桶、车辆和路缘主要进入 `/nonground_points`；
- 地面点数与非地面点数之和等于过滤后点数；
- 连续帧没有明显分类闪烁；
- 算法处理频率能够跟上约 10 Hz 的输入点云。

## 5. 锥桶聚类识别流程

本节独立描述锥桶检测方案。地面分割可能误删锥桶底部点，因此不能直接把非地面聚类作为最终锥桶点云，而应采用“两阶段检测”：

1. 在非地面点云中寻找锥桶候选位置；
2. 回到原始点云恢复候选点，再执行点数和几何过滤。

### 5.1 处理流程

```text
LiDAR PointCloud2
        ↓
Ground Segmenter
        ↓
Non-Ground Cloud
        ↓
Adaptive Clustering
        ↓
Candidate Center + Distance
        ↓
Raw Cloud + KD-Tree
        ↓
Cylinder Reconstruction
        ↓
Point Count Filter
        ↓
Geometry Filter
        ↓
Cone Position + Cone Point Cloud
        ↓
Color Estimation
```

### 5.2 步骤说明

| 步骤 | 目标 | 方法 | 输出 |
|---|---|---|---|
| 接收点云 | 获取 LiDAR 数据并保留重建所需的原始点 | 转换为 PCL 点云，删除 NaN，裁剪检测范围和车身区域 | `raw_cloud` |
| 地面分割 | 降低聚类计算量 | 将点云分为地面点和非地面点 | `non_ground_cloud` |
| 自适应聚类 | 找到疑似锥桶及其他小型目标 | 在非地面点云上建立 KD-Tree，按距离调整欧式聚类阈值 | `candidate_clusters` |
| 中心与距离 | 确定候选目标的大致位置 | 计算聚类质心及其到 LiDAR 的 XY 平面距离 | `center`、`distance` |
| 原始点邻域搜索 | 找回候选附近的原始点 | 在 `raw_cloud` 的 KD-Tree 中进行半径搜索 | `nearby_raw_points` |
| 圆柱重建 | 恢复被误删的锥桶底部点 | 对邻域点执行 XY 半径和高度过滤 | `reconstructed_cluster` |
| 点数过滤 | 检查点数是否符合距离规律 | 将实际点数与距离相关的理论范围比较 | `point_count_valid` |
| 几何过滤 | 排除护栏、车辆和地面残留 | 检查包围盒尺寸、高宽比和点数范围 | `cone_candidates` |
| 结果输出 | 为定位、建图和规划提供观测 | 发布中心、点云和 RViz 标记 | `cone_positions`、`cone_clouds` |
| 颜色估计 | 判断蓝色、黄色或其他类别 | 使用 LiDAR 强度或相机与 LiDAR 融合 | `classified_cones` |

### 5.3 保留和同步原始点云

地面分割后的 `non_ground_cloud` 只用于寻找候选位置，最终锥桶点云从
`raw_cloud` 中重建：

```cpp
raw_cloud = input_cloud;
non_ground_cloud = groundSegmenter(raw_cloud);
```

如果锥桶节点分别订阅原始点云和 `/nonground_points`，必须根据消息时间戳进行同步，
避免使用不同帧的数据重建同一个候选目标。

### 5.4 距离自适应聚类

由于 LiDAR 角分辨率固定，目标越远，点间距越大。聚类阈值可设置为：

$$
r_{cluster}(d)=\min(r_{max},r_0+kd)
$$

- $r_0$：基础聚类距离；
- $k$：距离增益；
- $d$：点到 LiDAR 的距离；
- $r_{max}$：最大阈值，用于防止不同目标粘连。

PCL 的 `EuclideanClusterExtraction` 一次只能使用一个固定阈值，因此第一版可以按
距离分区，再为各区间设置不同的 `cluster_tolerance`。

### 5.5 候选中心与距离

对于包含 $N$ 个点的候选聚类，其质心为：

$$
c_x=\frac{1}{N}\sum x_i,\qquad
c_y=\frac{1}{N}\sum y_i,\qquad
c_z=\frac{1}{N}\sum z_i
$$

候选目标距离使用 XY 平面距离：

$$
d=\sqrt{c_x^2+c_y^2}
$$

### 5.6 Raw Cloud、KD-Tree 与圆柱重建

在 `raw_cloud` 上建立 KD-Tree，并以候选中心进行邻域搜索。KD-Tree 只负责加速
查找附近点，不负责判断目标是否为锥桶。

标准三维 `radiusSearch()` 返回球形邻域，因此还需要执行圆柱过滤。一个点只有同时
满足以下条件才加入重建聚类：

$$
(x-c_x)^2+(y-c_y)^2\leq R^2
$$

$$
z_{min}\leq z\leq z_{max}
$$

其中 $R$ 为圆柱半径。实际系统应优先使用相对于局部地面的高度，以适应赛道坡度。

### 5.7 距离自适应点数过滤

距离 $d$ 处锥桶的理论点数可近似表示为：

$$
E(d)=\frac{1}{2}
\left(\frac{h_c}{2d\tan(r_v/2)}\right)
\left(\frac{w_c}{2d\tan(r_h/2)}\right)
$$

- $h_c,w_c$：锥桶高度和宽度；
- $r_v,r_h$：LiDAR 垂直和水平角分辨率；
- $d$：锥桶距离。

实际点数不要求严格等于理论值，而应满足：

$$
\alpha E(d)\leq N_{actual}\leq\beta E(d)
$$

$\alpha$ 和 $\beta$ 需要使用 rosbag 或 Gazebo 数据调试。

### 5.8 几何过滤

计算重建聚类的轴对齐包围盒：

```text
length = x_max - x_min
width  = y_max - y_min
height = z_max - z_min
```

主要检查：

- 高度、长度和宽度范围；
- 高宽比或长宽比；
- 最大和最小点数；
- 可选的“点云由下向上逐渐变窄”特征。

最终判断为：

$$
is\_cone=point\_count\_valid\land geometry\_valid
$$

### 5.9 输出内容

每个锥桶候选至少保存：

```text
center_x, center_y, center_z
distance
point_count
length, width, height
reconstructed_point_cloud
```

建议发布：

- 锥桶中心或自定义锥桶数组消息；
- 重建后的锥桶点云；
- RViz `MarkerArray`，用于显示中心和包围盒。

### 5.10 推荐代码结构

```text
cone_detection/
├── include/cone_detection/
│   ├── adaptive_clusterer.hpp
│   ├── cone_reconstructor.hpp
│   └── cone_filter.hpp
├── src/
│   ├── adaptive_clusterer.cpp
│   ├── cone_reconstructor.cpp
│   ├── cone_filter.cpp
│   └── cone_detection_node.cpp
├── config/cone_detection.yaml
├── launch/cone_detection.launch.py
├── test/
├── CMakeLists.txt
└── package.xml
```

| 模块 | 职责 |
|---|---|
| `cone_detection_node` | 订阅并同步点云、组织处理流程和发布结果 |
| `adaptive_clusterer` | 在非地面点云上生成候选聚类 |
| `cone_reconstructor` | 从原始点云恢复候选目标点云 |
| `cone_filter` | 执行点数和几何规则判断 |

### 5.11 建议实现顺序

1. 使用固定阈值完成欧式聚类，并在 RViz2 中显示结果；
2. 计算并显示候选中心；
3. 实现原始点云圆柱重建；
4. 增加点数和几何过滤；
5. 将固定阈值升级为距离自适应聚类；
6. 使用 rosbag 和 Gazebo 数据调参；
7. 接入颜色估计、定位和建图模块。

核心原则：

```text
Adaptive Clustering：寻找候选位置
Raw Cloud + KD-Tree：恢复被误删的锥桶点
Point Count Filter：检查点数是否符合距离规律
Geometry Filter：检查形状是否符合锥桶
```

地面分割后的聚类不是最终锥桶点云，最终判断必须基于重建后的候选点云。

## 6. RViz2 与调试

rosbag 数据通常使用点云自身的 `frame_id`（例如 `velodyne_top`）作为 Fixed Frame；
Gazebo 联合仿真使用 `odom`，静态排查时也可以使用 `lidar_link`。

建议显示以下点云：

| 话题 | 建议颜色 |
|---|---|
| 原始点云或 `/lidar/points` | 灰色 |
| `/debug/filtered_points` | 白色 |
| `/debug/bin_min_points` | 蓝色 |
| `/ground_points` | 绿色 |
| `/nonground_points` | 红色 |

RViz2 中的点云 QoS 设置为：

```text
Reliability: Best Effort
Durability: Volatile
```

坐标系遵循 ROS 约定：x 向前、y 向左、z 向上。

```text
odom
└── base_link
    └── lidar_link
```

## 7. 测试与当前状态

推荐验证顺序：

1. 单元测试：检查过滤、索引、拟合和聚类边界条件；
2. 慢速 rosbag：检查连续帧稳定性；
3. 正常频率 rosbag：检查准确性和实时性；
4. Gazebo 静态场景：检查地面、锥桶和坐标关系；
5. Gazebo 动态场景：检查车辆运动时的分类稳定性；
6. 坡道、起伏路面和稀疏锥桶场景：检查泛化能力。

当前状态：

- [x] PointCloud2 输入、PCL 转换和 ROI 过滤；
- [x] 极坐标分段、分桶和桶最低点提取；
- [x] 局部地面线拟合及地面/非地面分类；
- [x] Gazebo 128 线 LiDAR、ROS 2 桥接和 RViz2 联动；
- [x] Ackermann 车辆运动与键盘控制；
- [ ] 将现有地面分割测试源文件接入 CMake/colcon；
- [ ] 完成坡道、起伏路面和动态分割验收；
- [ ] 实现锥桶自适应聚类、原始点云重建和几何过滤；
- [ ] 发布锥桶三维位置、候选点云和可视化标记；
- [ ] 完成实时性、准确率和长时间稳定性测试。
