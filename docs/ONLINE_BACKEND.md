# 在线 Hector 前端 + 工作空间内后端

日期：2026-11-17。此前离线可行性实验现已实现为在线ROS2节点。本版本面向已验证的b0水平MultiEcho激光+IMU链路，Hector继续做局部扫描匹配，Cartographer来源的位姿图/回环算法在工作空间内编译执行。

## 目录与依赖

```text
/home/lc/hector_slam_ros2/
  install.sh                   一键检查依赖并整包编译
  run_online.sh                启动在线前端、后端和RViz
  3rd -> src/hector_slam/3rd    第三方源码/缓存入口
  src/hector_slam/
    3rd/cartographer/          后端/工具完整源码快照、配置、许可证和来源清单
    3rd/cartographer_ros/      原接口和RViz显示代码的来源快照
    3rd/debs/                  安装缺失系统依赖时缓存deb
    hector_backend_core/      编译上述源码，自有静态库及CMake导出
    hector_backend_msgs/      自有子图消息和服务
    hector_backend_rviz/      自有RViz子图插件
    hector_backend/           在线/离线后端、launch、配置
    hector_cloud_frontend/    修复版Hector前端
    hector_mapping/
    hector_slam/
```

`3rd`入口和真实源码都在Hector工作空间内；实际源码位于Git仓库中，克隆时一起获得。无Cartographer二进制、预编译静态库或另一工作空间的源码路径依赖。Cartographer源码保留其namespace及Apache-2.0版权，不把第三方代码冒充原创。

通用系统依赖：ROS2 Jazzy、C++工具链、Ceres、Eigen、Boost、Abseil、Protobuf、glog/gflags、Lua5.3、Cairo、Qt/RViz。脚本通过dpkg检查缺失项，在需要时apt安装并缓存直接下载的deb。系统ROS2 Jazzy应已安装；这不是ROS发行版本身或完全离线的系统安装器。

## 一键安装

```bash
bash /home/lc/hector_slam_ros2/install.sh
```

可选 `--dry-run`、`--skip-system-deps`。仓库内入口为 `scripts/install.sh --workspace /absolute/hector_ws`。没有缺失包时不会调用sudo/apt安装；有缺失时sudo可能要求用户在终端输入系统密码。

构建前显式清除AMENT/CMAKE/COLCON/LD_LIBRARY/PYTHONPATH等外部overlay，只source `/opt/ros/jazzy`。构建目录为`build_online`、`install_online`、`log_online`，避免覆盖以前实验用的install。编译范围覆盖7个在线相关包；Noetic旧辅助包和实验快照不参与构建。第三方源代码由本地hector_backend_core编译，不调用find_package(cartographer)、find_package(cartographer_ros)或find_package(zx_carto)。

## 在线启动与RViz

```bash
bash /home/lc/hector_slam_ros2/run_online.sh \
  output_directory:=/absolute/new_output
```

默认开启RViz，默认独立ROS域228，用仿真时间。另一个终端：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=228
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 bag play /absolute/bag_directory --clock 200 --rate 1
```

本机已建立可用背包输入目录：`/home/lc/hector_slam_ros2/data/b0-2014-07-11-11-00-49_ros2`，其中是实际Downloads MCAP/metadata的链接，不复制或改写原始数据。真实传感器接入时可传`use_sim_time:=false`，需要与本入口输入topic、外参和传感器配置一致。本次验证是bag在线处理，不宣称已完成实车部署。

结束后完成最终优化并保存：

```bash
source /home/lc/hector_slam_ros2/install_online/local_setup.bash
export ROS_DOMAIN_ID=228
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 service call /finish_hector_backend std_srvs/srv/Trigger '{}'
```

之后再Ctrl+C退出。保存优化节点TUM及地图PNG/PGM/YAML。没有指定output_directory时仍可在线显示，但不写结果文件。重新从头回放应重启整套节点。

## 坐标和输入语义

```text
map --后端全局校正--> hector_local_map --Hector原始局部位姿--> base_link
```

- 前端仍用修复版XYZ/重力高度/0.025m体素和严格历史Hector位姿速度去畸变。
- 前端map_frame改名为hector_local_map；去畸变节点增加historical_pose_frame参数，默认仍为map，在线入口显式指定hector_local_map。
- 后端订阅原始 `/slam_out_pose`、`/hector_matching_points`、`/deskew_reference_attitude`，精确按时间戳配对。后端优化位姿不回灌前端，不启动Cartographer局部建图/扫描匹配器。
- 按原生5s/0.2m/1°运动门限选择图节点，90帧半重叠子图，后端权重和回环参数沿用此前离线主实验。
- raw Hector局部地图改为 `/hector_local_map` topic；全局 `/map` 由后端子图合成，避免两个节点争用同一地图话题。
- 后端校正TF在可视化时钟上延展0.6s，以覆盖实时显示查询；不改变原始测量时间戳或局部匹配初值。

## RViz接口

自有插件类：`hector_backend_rviz/SubmapsDisplay`。它使用本工作空间的hector_backend_msgs，原cartographer_rviz插件的消息类型不同，不能混用。

- `/submap_list`：各子图版本、位姿；reliable/transient-local。
- `/submap_query`：查询压缩纹理，返回纹理尺寸、分辨率、切片姿态。
- `/backend_path`：当前优化后图节点轨迹。
- `/map`：合成占据栅格。
- `/backend_status`：接收帧、图节点、子图数和完成状态。

RViz默认显示分块子图与优化轨迹；子图位姿随后端优化更新，历史子图整体平移/旋转。验证中插件成功加载，子图逐步增长且纹理查询/解压通过。地图更新和节点跟踪为实际在线订阅处理，不是展示预先优化好的最终图。

## 验证结果

### 独立整包编译

清空外部overlay，从源码构建7包成功；首次完整构建约4分44秒。核心测试15项通过，0错误/失败/跳过。构建flags/link/CMake缓存未发现外部Cartographer或ZX安装前缀，在线二进制没有libcartographer/zx_carto动态依赖。

### 源码落地不改变后端结果

用之前相同的18932帧冻结Hector输入，对照外部Cartographer库和本工作空间源码编译库：6138个节点、69个子图、1743条跨子图约束一致，优化后的轨迹数组逐项完全相同，最大XY差0。该结论包含完整回环优化，不只测试前30s。

### 新鲜前端、真实1倍速在线整包

- Hector输出18934帧；在线后端接收全部18934帧。
- 6134图节点、69子图、1688条跨子图约束；子图数实际从1增长到69。
- 纹理查询成功，首张子图413×1016，解压839216字节，等于宽×高×2。
- 全部完成最终优化和轨迹/PNG保存。后续增加的PGM/YAML保存路径另用30s在线测试验证通过。
- 观察器收到18933条匹配点云，但后端收到18934条；这是观察器的best-effort采样少1条，不是后端丢输入。
- 同一轮在线轨迹1m局部段的相对平移变化：中位数0.79mm、P95为4.04mm；相对转角变化P95约0.317°。这些是结构变化量，不是对独立真值的精度。

在线重新运行的Hector轨迹与此前冻结记录有差异：历史位姿参与前处理的回调到达时序可改变可用历史速度，不能把新鲜实时重跑声称为逐帧确定性复现。严格等价结论只用于相同冻结输入下的前后端库对照。未用更接近Cartographer轨迹作为版本选优标准；局部结构、回访一致性和全局漂移收敛是主要观察项。

验证脚本：`tools/validation/check_online_backpack.py`。本机结果在`online_validation/full`、`online_validation/offline_equivalence_full`、`online_validation/save_map30`。本阶段尚未把在线图做成pbstream持久化/恢复功能，也没有新增实车定位控制接口。
