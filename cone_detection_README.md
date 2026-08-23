# Cone Detection

## 1. 目标

本模块从 LiDAR 点云中检测锥桶，并输出锥桶位置及完整候选点云。

地面分割可能误删锥桶底部点，因此采用两阶段处理：

1. 在非地面点云中寻找锥桶候选位置；
2. 回到原始点云恢复候选点，再进行规则过滤。

## 2. 处理流程

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

## 3. 各步骤的目标与方法

| 步骤 | 目标 | 方法 | 输出 |
|---|---|---|---|
| 1. 接收点云 | 获取 LiDAR 数据并保留重建所需的原始点 | 将 `PointCloud2` 转为 PCL 点云，删除 NaN，并进行检测范围和车身区域裁剪 | `raw_cloud` |
| 2. 地面分割 | 删除大部分地面点，降低聚类计算量 | 使用 Ground Segmenter 将点云分成地面点与非地面点 | `non_ground_cloud` |
| 3. 自适应聚类 | 找到疑似锥桶及其他小型目标 | 在非地面点云上建立 KD-Tree，使用随距离变化的欧式聚类阈值 | `candidate_clusters` |
| 4. 中心与距离 | 确定每个候选目标的大致位置 | 计算聚类质心以及质心到 LiDAR 的 XY 平面距离 | `center`、`distance` |
| 5. 原始点邻域搜索 | 找回候选位置附近的原始点 | 在 `raw_cloud` 上建立 KD-Tree，并以候选中心进行半径搜索 | `nearby_raw_points` |
| 6. 圆柱重建 | 恢复地面分割误删的锥桶底部点 | 对邻域点进行 XY 半径和高度过滤 | `reconstructed_cluster` |
| 7. 点数过滤 | 检查候选点数是否符合该距离处的锥桶特征 | 比较实际点数和距离相关的理论点数范围 | `point_count_valid` |
| 8. 几何过滤 | 排除护栏、车辆、地面残留等错误目标 | 检查包围盒尺寸、高宽比和点数范围 | `cone_candidates` |
| 9. 结果输出 | 为定位、建图、规划和颜色估计提供观测 | 发布锥桶中心、点云和 RViz 标记 | `cone_positions`、`cone_clouds` |
| 10. 颜色估计 | 判断蓝色、黄色或其他锥桶类别 | 使用 LiDAR intensity 或相机与 LiDAR 融合 | `classified_cones` |

## 4. 关键步骤

### 4.1 保留原始点云

地面分割后的 `non_ground_cloud` 只用于寻找候选位置。原始点云必须保留：

```cpp
raw_cloud = input_cloud;
non_ground_cloud = groundSegmenter(raw_cloud);
```

最终锥桶点云应从 `raw_cloud` 重建，而不是直接使用不完整的非地面聚类。

### 4.2 距离自适应聚类

由于 LiDAR 角分辨率固定，目标越远，点之间的空间距离越大。聚类阈值可设置为：

\[
r_{cluster}(d)=\min(r_{max},r_0+kd)
\]

- \(r_0\)：基础聚类距离；
- \(k\)：距离增益；
- \(d\)：点到 LiDAR 的距离；
- \(r_{max}\)：最大阈值，用于防止不同目标粘连。

PCL的 `EuclideanClusterExtraction` 一次只能使用一个固定阈值，因此第一版可按距离分区，再对每个区间设置不同的 `cluster_tolerance`。

### 4.3 候选中心和距离

对一个包含 \(N\) 个点的候选聚类计算质心：

\[
c_x=\frac{1}{N}\sum x_i,\qquad
c_y=\frac{1}{N}\sum y_i,\qquad
c_z=\frac{1}{N}\sum z_i
\]

候选目标距离使用 XY 平面距离：

\[
d=\sqrt{c_x^2+c_y^2}
\]

### 4.4 Raw Cloud + KD-Tree

在 `raw_cloud` 上建立 KD-Tree，并使用候选中心进行邻域搜索。KD-Tree只负责加速查找附近点，不负责判断目标是否为锥桶。

标准三维 `radiusSearch()` 返回球形邻域，因此还需要执行圆柱过滤。

### 4.5 圆柱重建

一个原始点只有同时满足以下条件才加入重建聚类：

\[
(x-c_x)^2+(y-c_y)^2\leq R^2
\]

\[
z_{min}\leq z\leq z_{max}
\]

其中 \(R\) 是圆柱半径。实际系统最好使用相对于局部地面的高度，以适应赛道坡度。

### 4.6 距离自适应点数过滤

距离 \(d\) 处锥桶的理论点数为：

\[
E(d)=\frac{1}{2}
\left(\frac{h_c}{2d\tan(r_v/2)}\right)
\left(\frac{w_c}{2d\tan(r_h/2)}\right)
\]

- \(h_c,w_c\)：锥桶高度和宽度；
- \(r_v,r_h\)：LiDAR垂直和水平角分辨率；
- \(d\)：锥桶距离。

实际点数不要求严格等于理论值，而应满足：

\[
\alpha E(d)\leq N_{actual}\leq\beta E(d)
\]

其中 \(\alpha\) 和 \(\beta\) 需要根据 rosbag 或 Gazebo 数据调试。

### 4.7 几何过滤

计算重建聚类的轴对齐包围盒：

```text
length = x_max - x_min
width  = y_max - y_min
height = z_max - z_min
```

检查项目包括：

- 高度、长度和宽度范围；
- 高宽比或长宽比；
- 最大和最小点数；
- 可选：点云是否从下向上逐渐变窄。

最终判断为：

\[
is\_cone=point\_count\_valid\land geometry\_valid
\]

## 5. 输出内容

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

## 6. 推荐代码结构

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
├── CMakeLists.txt
└── package.xml
```

| 模块 | 职责 |
|---|---|
| `cone_detection_node` | 订阅点云、组织处理流程并发布结果 |
| `adaptive_clusterer` | 在非地面点云上生成候选聚类 |
| `cone_reconstructor` | 从原始点云恢复候选目标点云 |
| `cone_filter` | 执行点数和几何规则判断 |

## 7. 建议实现顺序

1. 使用固定阈值完成欧式聚类，并在 RViz 显示结果；
2. 计算并显示候选中心；
3. 实现原始点云圆柱重建；
4. 增加点数和几何过滤；
5. 将固定阈值升级为距离自适应聚类；
6. 使用 rosbag 和 Gazebo 数据调参；
7. 接入颜色估计、定位和建图模块。

## 8. 核心原则

```text
Adaptive Clustering：寻找候选位置
Raw Cloud + KD-Tree：恢复被误删的锥桶点
Point Count Filter：检查点数是否符合距离规律
Geometry Filter：检查形状是否符合锥桶
```

地面分割后的聚类不是最终锥桶点云；最终判断必须基于重建后的候选点云。
