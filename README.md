# Indoor SLAM

Indoor SLAM 是一套基于 ROS 2 Humble 的室内定位与导航工作区，当前主要面向 Livox MID360 激光雷达。系统组合了 Livox 驱动、Point-LIO 里程计、small_gicp 重定位和 Nav2 导航，用于室内建图、重定位和自主导航实验。

## 项目组成

| 模块 | 路径 | 作用 |
| --- | --- | --- |
| Livox 驱动 | `src/livox_ros_driver2` | 启动 MID360 并发布点云/自定义雷达消息 |
| Point-LIO | `src/Point-LIO` | 激光惯性里程计，输出机器人位姿和注册点云 |
| small_gicp_relocalization | `src/small_gicp_relocalization` | 基于点云地图的重定位 |
| Nav2 导航 | `src/bxi_nav` | 启动地图服务器、Nav2、点云转激光和 RViz |
| 运行时主管 | `src/bxi_slam_manager` | 静默启动，并按 App 请求切换建图、3D 重定位、导航和续建模式 |
| 导航规划/控制 | NavFn A* + PID Path Follower | 移植自 `indoor_slam`，保留路径库局部避障、地形点云碰撞检测和速度平滑 |
| 地图转换器 | `pcd2pgm_headless` + `scans_nav2_map.cfg` | 保存地图时把 PCD 转成 Nav2 栅格地图 |

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

机器人上的部署入口是仓库根目录的 `install.sh`（需要 root），它会编译整个
工作区并安装到 `/opt/bxi/bxi_rc_slam/install`：

```bash
sudo ./install.sh
```

开发环境可直接在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

只编译导航相关包：

```bash
colcon build --packages-up-to nav
source install/setup.bash
```

## 一键启动

`start.sh` 默认只启动轻量的 `bxi_slam_manager`。系统开机后处于
`idle` 静默模式，不加载历史地图，也不启动 Point-LIO、GICP、Nav2 或 RViz。
Livox 驱动同样按需启动，避免无人使用时持续接收和复制高带宽点云。App 发出
运行模式请求后，主管才启动雷达与对应算法栈：

| 模式 | 行为 |
| --- | --- |
| `new_mapping` | 启动 Point-LIO 和实时 2D 栅格，边扫描边建图 |
| `navigation` | 加载地图 bundle，启动 Point-LIO、3D GICP 和 Nav2；定位成功前禁止运动目标 |
| `extend_mapping` | 在父地图 3D 重定位成功后续建，保存为不可变子版本 |
| `idle` | 停止 Livox 与所有按需算法进程，仅主管保持在线 |

主管每秒检查算法子进程与 3D 定位状态。GICP 状态超过 3 秒未更新或任一算法
子进程异常退出时，系统会立即重新锁住导航并进入 `error`。按需进程组记录在
`BXI_SLAM_PROCESS_REGISTRY` 指定的位置（root 默认
`/run/bxi/slam-processes.json`），主管异常重启时会先清理旧进程组。

正式部署由 `bxi_rc_ros2.service` 启动 SLAM Manager，再由 App 的运行模式接口
让 Manager 拉起导航。所有子进程继承 RC 的 Domain 与 CycloneDDS 配置，不要并行
执行另一套 `ros2 launch`。

仅在离线排障、且正式服务未运行时，可以使用备用 tmux 启动器：

```bash
./start.sh
```

`start.sh` 优先读取 RC 的正式环境文件：

```bash
/opt/bxi/bxi_rc_ros2/env.conf
```

它不会硬编码或创建独立 `ROS_DOMAIN_ID`；环境文件不存在时继承当前 shell 的
ROS/DDS 环境。

如需覆盖雷达配置：

```bash
LIVOX_CONFIG_PATH=/absolute/path/MID360.json ./start.sh
```

停止：

```bash
./stop.sh
```

`stop.sh` 只关闭 `indoor_slam` 会话，不影响当前用户的其它 tmux 任务。

## 手动启动

如果需要分开排查，可以按顺序手动启动。

终端 1，雷达驱动：

```bash
set -a; source /opt/bxi/bxi_rc_ros2/env.conf; set +a
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

终端 2，Point-LIO（含 App 建图控制接口）：

```bash
set -a; source /opt/bxi/bxi_rc_ros2/env.conf; set +a
source install/setup.bash
ros2 launch point_lio point_lio_with_mapping_control.launch.py
```

终端 3，重定位：

```bash
set -a; source /opt/bxi/bxi_rc_ros2/env.conf; set +a
source install/setup.bash
ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py
```

终端 4，导航：

```bash
set -a; source /opt/bxi/bxi_rc_ros2/env.conf; set +a
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
| `nav2_map_server` | 加载 2D 栅格地图 |
| `nav2_lifecycle_manager` | 自动激活地图服务器 |
| `nav2_bringup/navigation_launch.py` | 启动 NavFn A*、PID Path Follower、路径/速度平滑和行为树 |
| `nav_odom` | 把 Point-LIO 里程计转换到规范的 `base_link` 轴并发布 `/nav/odom` |
| `terrain_analysis` | 从注册点云生成 `/terrain_map`，供 PID 局部规划器避障 |
| `collision_monitor` | 对平滑后的 `/cmd_vel` 做最后安全检查并输出 `/cmd_vel_safe` |
| `pointcloud_to_laserscan_node` | 将 `/cloud_registered` 转为 `/scan` |
| `app_nav_gateway` | 保留 `/nav/*` App action/service/topic 接口并转发到 Nav2 |
| `rviz2` | 打开 Nav2 默认 RViz 配置 |

常用 launch 参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `map` | 空（必须显式传入） | Nav2 栅格地图 yaml 的完整路径 |
| `use_sim_time` | `false` | 是否使用仿真时间 |
| `autostart` | `false` | 是否自动激活生命周期节点 |
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
resolution: 0.050000
origin: [-21.000000, -15.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

如果默认启动找不到地图，请显式传入地图路径：

```bash
ros2 launch nav indoor_navigation_launch.py map:=$(pwd)/src/bxi_nav/maps/maps.yaml
```

## 从 PCD 生成 Nav2 地图

PCD 到 Nav2 栅格地图的转换由仓库根目录的 `pcd2pgm_headless` 可执行文件完成，
转换流水线定义在 `scans_nav2_map.cfg`。建图结束保存地图时，
`mapping_control_node`（`src/Point-LIO/scripts`）会自动调用它，把保存的
PCD 转成配套的 `.pgm`/`.yaml` 栅格地图，一般无需手动执行。

手动转换示例（在期望的输出目录下执行）：

```bash
/path/to/pcd2pgm_headless --headless <input.pcd> \
  --config /path/to/scans_nav2_map.cfg \
  --save-point-cloud map.pcd
```

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

正式环境当前使用 `22`，但本项目不再硬编码该值；以
`/opt/bxi/bxi_rc_ros2/env.conf` 为唯一配置源。

## 开发备注

修改导航参数：

```bash
src/bxi_nav/config/nav2_params.yaml
src/bxi_nav/config/collision_monitor_params.yaml
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
colcon build --packages-up-to nav
source install/setup.bash
```
