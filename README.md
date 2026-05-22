# hdl_ws_ros2

ROS 2 Humble 移植版 HDL 定位工作空间：基于 NDT_OMP 的实时 3D 定位，支持全局重定位（`/relocalize`）。由 ROS 1 工程 [hdl_ws](../hdl_ws) 迁移而来。

## 功能概览

| 功能 | 说明 |
|------|------|
| 位姿跟踪 | NDT_OMP 扫描匹配 + UKF（可选 IMU） |
| 全局重定位 | `hdl_global_localization`（BBS / FPFH_RANSAC） |
| 重定位服务 | `/relocalize`（`std_srvs/srv/Empty`） |
| 指标话题 | `/hdl_localization/reloc_status` |
| Livox MID360 | `scripts/convert_pointcloud2.py` 将 `CustomMsg` 转为 `PointCloud2` |

## 目录结构

```text
hdl_ws_ros2/
├── README.md
├── .gitignore              # Git 忽略 build/install/log（上传 GitHub 必需）
├── scripts/
│   └── convert_pointcloud2.py
└── src/
    ├── ndt_omp/
    ├── fast_gicp/
    ├── hdl_global_localization/
    └── hdl_localization/
```

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Livox 驱动（单独工作空间）：[livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)（Humble 分支），发布 `/livox/lidar`（`CustomMsg`）

## 依赖安装

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-tf2-ros \
  ros-humble-tf2-eigen \
  ros-humble-sensor-msgs-py \
  libpcl-dev \
  libeigen3-dev \
  libomp-dev \
  python3-colcon-common-extensions
```

Livox 转换脚本额外需要（随 `livox_ros_driver2` 工作空间编译安装）：

```bash
# 在 livox 工作空间中
cd ~/livox_ws   # 你的 livox_ros_driver2 路径
source /opt/ros/humble/setup.bash
colcon build --packages-select livox_ros_driver2
source install/setup.bash
```

## 编译

```bash
cd ~/hdl_ws_ros2
source /opt/ros/humble/setup.bash

# 必须带上全部依赖包（不要只编 hdl_localization）
colcon build --packages-up-to hdl_localization \
  --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install

source install/setup.bash
```

### 地图文件

将全局地图 PCD 放到：

```text
src/hdl_localization/data/map.pcd
```

编译后安装到 `install/hdl_localization/share/hdl_localization/data/map.pcd`。也可在 launch 时用 `globalmap_pcd:=/path/to/xxx.pcd` 指定。

---

## 使用步骤

以下命令假设工作空间在 `~/hdl_ws_ros2`，按实际路径修改。

### 终端 1：Livox 驱动（MID360）

```bash
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash   # livox_ros_driver2

ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

确认有话题：`ros2 topic echo /livox/lidar --once`

### 终端 2：点云格式转换（CustomMsg → PointCloud2）

```bash
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash

cd ~/hdl_ws_ros2
python3 scripts/convert_pointcloud2.py
```

默认：`/livox/lidar` → `/lidar/pointcloud2`。可选参数：

```bash
python3 scripts/convert_pointcloud2.py --ros-args \
  -p input_topic:=/livox/lidar \
  -p output_topic:=/lidar/pointcloud2 \
  -p output_frame_id:=livox_frame
```

### 终端 3：定位 + 全局定位

```bash
source /opt/ros/humble/setup.bash
source ~/hdl_ws_ros2/install/setup.bash

ros2 launch hdl_localization hdl_localization.launch.py \
  use_sim_time:=false \
  points_topic:=/lidar/pointcloud2 \
  imu_topic:=/livox/imu \
  odom_child_frame_id:=livox_frame
```

实时 Livox 时 `use_sim_time:=false`；播 rosbag 时改为 `true` 且 bag 需 `--clock`。

期望日志示例：

```text
[hdl_global_localization]: hdl_global_localization ready (...)
[hdl_localization]: global localization services are ready
[hdl_localization]: globalmap received!
[hdl_localization]: SetGlobalMap finished
```

### 终端 4：RViz2

```bash
source /opt/ros/humble/setup.bash
source ~/hdl_ws_ros2/install/setup.bash

rviz2 -d $(ros2 pkg prefix hdl_localization)/share/hdl_localization/rviz/hdl_localization_ros2.rviz
```

- Fixed Frame：`map`
- GlobalMap：Durability 选 **Transient Local**

### 终端 5：重定位（可选）

在点云与地图已对齐、且已有扫描数据后：

```bash
source /opt/ros/humble/setup.bash
source ~/hdl_ws_ros2/install/setup.bash

ros2 service call /relocalize std_srvs/srv/Empty "{}"
```

查看结果：

```bash
ros2 topic echo /hdl_localization/reloc_status --once
```

---

## 使用 rosbag 回放（示例）

```bash
# 终端：定位（use_sim_time:=true）
ros2 launch hdl_localization hdl_localization.launch.py \
  use_sim_time:=true \
  points_topic:=/velodyne_points \
  imu_topic:=/imu \
  odom_child_frame_id:=velodyne

# 终端：RViz（同上）

# 终端：播 bag（必须 --clock）
ros2 bag play /path/to/your_bag --clock -r 1.0

# 终端：重定位
ros2 service call /relocalize std_srvs/srv/Empty "{}"
```

工作空间内示例 bag：`src/hdl_localization/sample-bag/subset/`

---

## 初始位姿与重定位测试

| 方式 | 说明 |
|------|------|
| Launch 参数 | `init_pos_x:=25.0 init_pos_y:=15.0` 等；启动日志应打印 `init position (map): ...` |
| RViz | `specify_init_pose:=false`，用 **2D Pose Estimate** 设错初值后再 `/relocalize` |

注意：`/aligned_points` 是 NDT **配准后**的点云，即使初值错误也可能很快看起来贴合地图；请结合 `/odom` 与 `reloc_status` 判断。

---

## 常用检查命令

```bash
ros2 service list | grep -E 'hdl_global|relocalize'
ros2 topic list | grep -E 'odom|globalmap|aligned|pointcloud2|livox'
ros2 param get /hdl_localization init_pos_x
```

---

## 包说明

| 包 | 作用 |
|----|------|
| `ndt_omp` | OpenMP NDT |
| `fast_gicp` | GICP（默认不启用 CUDA） |
| `hdl_global_localization` | 全局定位服务 |
| `hdl_localization` | 定位与重定位主节点 |

## 参考

- [hdl_localization (ROS1)](https://github.com/koide3/hdl_localization)
- [hdl_global_localization](https://github.com/koide3/hdl_global_localization)

## 上传 GitHub

根目录 `.gitignore` 会排除 `build/`、`install/`、`log/`，避免把编译产物推上去。首次提交示例：

```bash
cd ~/hdl_ws_ros2
git init
git add .
git commit -m "ROS2 Humble port of hdl_localization with Livox converter"
git remote add origin https://github.com/<user>/hdl_ws_ros2.git
git branch -M main
git push -u origin main
```

## License

各子包保留原仓库 BSD 等许可证。
