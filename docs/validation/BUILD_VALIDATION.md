# Jazzy 构建与验证

2026-11-16，Ubuntu24.04 / ROS2 Jazzy / Release。

- 专用工作空间：`/home/lc/hector_slam_ros2`。
- 构建包：hector_mapping、hector_slam、hector_cloud_frontend，全部成功。
- 核心测试：15 tests，0 errors，0 failures，0 skipped（包含gtest套件汇总）。
- 安装后的两个frontend launch均通过参数解析与Python/XML检查。
- 安装后的EKF/pre/desk/time通用入口曾完成真实1倍速前10s检查：98个原始位姿与匹配点云，96个业务位姿，先验地图单一哈希，全部匹配接受。
- 迁移前后冻结输入验证：772帧位置、四元数、时间戳逐项完全一致，详见migration_output_equivalence.json。
- 最终完整业务bag实验为772原始/769业务位姿，原始传感器计数完整，XYZ全束重建最大差约1.01e-6m，地图不变。

构建命令：
```bash
source /opt/ros/jazzy/setup.bash
source /path/to/cartographer/install/setup.bash
colcon build --packages-select hector_mapping hector_slam hector_cloud_frontend --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --packages-select hector_mapping
colcon test-result --verbose
```

原Noetic辅助包未完成移植，通过COLCON_IGNORE排除；实验快照不参与构建。
