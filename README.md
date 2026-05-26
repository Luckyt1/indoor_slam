# Indoor SLAM

Indoor SLAM 是一套基于 ROS 2 Humble 的室内定位与导航工作区，当前主要面向 Livox MID360 激光雷达。系统组合了 Livox 驱动、Point-LIO 里程计、small_gicp 重定位和 Nav2 导航，用于室内建图、重定位和自主导航实验。

## 项目组成

| 模块 | 路径 | 作用 |
| --- | --- | --- |
| Livox 驱动 | `src/livox_ros_driver2` | 启动 MID360 并发布点云/自定义雷达消息 |
| Point-LIO | `src/Point-LIO` | 激光惯性里程计，输出机器人位姿和注册点云 |
| small_gicp_relocalization | `src/small_gicp_relocalization` | 基于点云地图的重定位 |
| Nav2 导航 | `src/bxi_nav` | 启动地图服务器、Nav2、点云转激光和 RViz |
| 自定义控制器 | `src/pid_path_follower` | Nav2 控制器插件 |
| 地图工具 | `tools` | PCD 到 Nav2 栅格地图转换、地图清理脚本 |

## 环境要求

推荐环境：

```bash
Ubuntu 22.04
ROS 2 Humble
Livox MID360
```

常用依赖：

```bash
sudo apt update
sudo apt install -y \
  tmux \
  python3-numpy \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-tf2-ros
```

如果使用 `rosdep`，可以在工作区根目录执行：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## 编译

在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

只编译导航包：

```bash
colcon build --packages-select nav
source install/setup.bash
```

## 一键启动

`start.sh` 会创建一个 `tmux` 会话，并按 4 个窗格启动完整系统：

| 窗格 | 模块 | 命令 |
| --- | --- | --- |
| 左上 | 雷达驱动 | `ros2 launch livox_ros_driver2 msg_MID360s_launch.py` |
| 左下 | Point-LIO | `ros2 launch point_lio point_lio.launch.py` |
| 右上 | 重定位 | `ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py` |
| 右下 | 导航 | `ros2 launch nav indoor_navigation_launch.py` |

启动：

```bash
./start.sh
```

`start.sh` 默认使用：

```bash
ROS_DOMAIN_ID=37
```

停止：

```bash
./stop.sh
```

注意：当前 `stop.sh` 使用 `tmux kill-server`，会关闭当前用户下所有 tmux 会话。

## 手动启动

如果需要分开排查，可以按顺序手动启动。

终端 1，雷达驱动：

```bash
export ROS_DOMAIN_ID=37
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

终端 2，Point-LIO：

```bash
export ROS_DOMAIN_ID=37
source install/setup.bash
ros2 launch point_lio point_lio.launch.py
```

终端 3，重定位：

```bash
export ROS_DOMAIN_ID=37
source install/setup.bash
ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py
```

终端 4，导航：

```bash
export ROS_DOMAIN_ID=37
source install/setup.bash
ros2 launch nav indoor_navigation_launch.py
```

如需指定地图：

```bash
ros2 launch nav indoor_navigation_launch.py map:=/absolute/path/to/maps.yaml
```

## 导航启动说明

导航入口：

```bash
ros2 launch nav indoor_navigation_launch.py
```

主要功能：

| 节点 | 作用 |
| --- | --- |
| `static_transform_publisher` | 发布 `body_raw -> base_link` 静态变换 |
| `nav2_map_server` | 加载 2D 栅格地图 |
| `nav2_lifecycle_manager` | 自动激活地图服务器 |
| `nav2_bringup/navigation_launch.py` | 启动 Nav2 导航核心节点 |
| `pointcloud_to_laserscan_node` | 将 `/cloud_registered` 转为 `/scan` |
| `rviz2` | 打开 Nav2 默认 RViz 配置 |

常用 launch 参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `map` | `share/nav/maps/maps.yaml` | Nav2 栅格地图 |
| `params_file` | `share/nav/config/nav2_params.yaml` | Nav2 参数文件 |
| `use_sim_time` | `false` | 是否使用仿真时间 |
| `autostart` | `true` | 是否自动激活生命周期节点 |
| `rviz` | `true` | 是否启动 RViz |

## 地图文件

当前导航地图位于：

```bash
src/bxi_nav/maps/maps.yaml
src/bxi_nav/maps/maps.pgm
```

地图参数示例：

```yaml
image: maps.pgm
mode: trinary
resolution: 0.100000
origin: [-22.000000, -14.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

如果默认启动找不到地图，请显式传入地图路径：

```bash
ros2 launch nav indoor_navigation_launch.py map:=$(pwd)/src/bxi_nav/maps/maps.yaml
```

## 从 PCD 生成 Nav2 地图

工具脚本：

```bash
tools/pcd_to_occupancy_grid.py
```

示例：

```bash
python3 tools/pcd_to_occupancy_grid.py \
  --pcd maps/PCD/scans.pcd \
  --output-dir src/bxi_nav/maps \
  --name maps \
  --resolution 0.10 \
  --unknown-as-free
```

输出：

```bash
src/bxi_nav/maps/maps.pgm
src/bxi_nav/maps/maps.yaml
```

可根据点云高度调整：

| 参数 | 说明 |
| --- | --- |
| `--occupied-z-min` / `--occupied-z-max` | 障碍物高度范围 |
| `--free-z-min` / `--free-z-max` | 地面/可通行区域高度范围 |
| `--occupied-dilation` | 障碍物膨胀格数 |
| `--resolution` | 地图分辨率，单位 m/cell |

## 常用检查命令

查看节点：

```bash
ros2 node list
```

查看话题：

```bash
ros2 topic list
```

检查点云：

```bash
ros2 topic hz /cloud_registered
```

检查激光：

```bash
ros2 topic hz /scan
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

查看地图信息：

```bash
ros2 topic echo --once /map --field info
```

## 常见问题

### RViz 没有点云或地图

先确认对应话题是否存在：

```bash
ros2 topic list
```

再确认 RViz 的 Fixed Frame 是否和 TF 树一致，常用 frame 包括 `map`、`odom`、`base_link`。

### Nav2 无法规划

优先检查：

```bash
ros2 lifecycle nodes
ros2 topic echo --once /map --field info
ros2 run tf2_ros tf2_echo map base_link
```

常见原因是地图未加载、TF 不连通、定位还没有收敛，或 `/scan` 没有数据。

### 找不到地图文件

显式传入地图路径：

```bash
ros2 launch nav indoor_navigation_launch.py map:=$(pwd)/src/bxi_nav/maps/maps.yaml
```

也可以重新编译并 source：

```bash
colcon build --packages-select nav
source install/setup.bash
```

### 多机或多终端通信异常

确认所有终端使用同一个 `ROS_DOMAIN_ID`：

```bash
echo $ROS_DOMAIN_ID
```

本项目启动脚本默认使用 `37`。

## 开发备注

修改导航参数：

```bash
src/bxi_nav/config/nav2_params.yaml
```

修改导航 launch：

```bash
src/bxi_nav/launch/indoor_navigation_launch.py
```

修改地图：

```bash
src/bxi_nav/maps/maps.yaml
src/bxi_nav/maps/maps.pgm
```

每次修改 C++ 代码后重新编译：

```bash
colcon build --packages-select nav
source install/setup.bash
```
