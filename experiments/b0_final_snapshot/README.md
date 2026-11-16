# b0 Hector：共享 IMU 参考姿态与三维匹配输入

> 后续原版行为对照已完成：原版默认算法参数（0.025m、0.4m、0.9rad）在前30秒没有米级漂移；原版行为沿用我们之前的0.05m、0.2m、0.08rad组合时，即使不做desk/内部IMU处理也会漂移。参数组合是关键因素，不能将旧现象单独归因于输入链路，更不能称为上游已确认历史bug。本目录的输入改进是在旧参数组合下改善表现。完整对照见 `/home/lc/slam_11_13/results/b0_upstream_behavior_20261116/README.md`。

修正 b0 背包包前 6–8 秒沿 X 多走约 1.4 米、地图墙面重影的问题。仅用水平 MultiEcho 雷达第一回波和 IMU；平移去畸变速度来自严格早于扫描第一束的 Hector 历史位姿。无轮速、EKF、Cartographer 匹配、后端优化或参考轨迹回灌。

## 修正

旧链路：三维逐点去畸变 -> 0.5°二维 LaserScan 重采样并丢 Z -> 独立 0.5 秒姿态滤波 -> 重力投影 -> 默认高度 [-1,1]m -> 未做空间点密度过滤 -> Hector。

新链路：保留逐点去畸变 XYZ -> 和去畸变使用完全同一时刻、同一套 ImuTracker 的参考四元数 -> 一次重力投影 -> 基于重力方向高度 [-0.8,2.0]m 过滤 -> 0.025m 三维体素过滤 -> 原 Hector 多分辨率 GN 匹配与栅格更新。体素过滤复用 Cartographer 的 sensor::VoxelFilter，地图和位姿仍完全由 Hector 产生。Cartographer 原生 2D 配置同样使用这个高度范围及 0.025m 体素过滤。

关键对照：仅统一姿态并保留 XYZ，前30秒 RMSE 仍为 0.82m；加入体素过滤后约0.037m。保留原0.5秒重力滤波器但使用同样高度/体素过滤，约0.046m。保留旧[-1,1]高度即使加体素仍约1.15m。因此不支持“单纯换 IMU 滤波器即可”的结论；倾斜时的高度筛选与密集扫描点对栅格匹配的权重必须同时处理。0.025/0.05/0.1m 体素对照都在约0.04m量级，未依靠截断位置输出或针对6–8秒的特殊规则。

此目录内 include/hector_slam_lib 与诊断开始时原核心逐字相同；没有采用试验中的运动软先验、梯度修正或步长回溯。旧生产/业务定位入口保持原样。

## 运行

```bash
bash /home/lc/slam_11_13/scripts/run_b0_hector_gravity_cloud.sh
```

默认前30秒、1倍速、先启动 RViz 再开始回放，完成后保留地图和窗口。Ctrl+C 关闭。本入口用独立 ROS domain 221，检测已有发布者后才允许启动。

指定验证窗口（例如前90秒）：

```bash
bash /home/lc/slam_11_13/scripts/run_b0_hector_gravity_cloud.sh --duration 90
```

完整 bag 可传 `--duration 0`，会使用8192格地图以容纳该包全程范围；整包仍待验证。无窗口测试：末尾加 `--no-rviz --no-hold`。自定义输出：`--output /绝对路径/新目录`。结果包括 MCAP、PGM/YAML、TUM轨迹、点云/姿态配对状态、历史预测状态、日志、源码快照及复用原绘图脚本产生的三模块图。脚本只接受1倍速；不提供倍速参数。

首次需要构建时会自动调用 build.sh；也可手动运行该脚本。

## 输入与显示

- `/points_carto`：保留 XYZ 的去畸变点云，base_link，扫描最后一束的时刻。
- `/deskew_reference_attitude`：同一时刻、同一 ImuTracker 的参考姿态。是合成的姿态载体，不是原始 IMU 测量；gyro/accel 标记为不可用。
- `hector_cloud_node` 严格按纳秒时间戳配对上述消息，不插值另一个姿态滤波器。
- `/hector_matching_points`：实际投影到 XY、经过高度和体素过滤的匹配点，重新表达在 base_link，使完整姿态 TF 下显示于地图平面。RViz 默认显示它。
- `/scan_carto`：为了复用旧诊断保留的中间二维扫描，Hector 新入口不订阅它，RViz 不显示它。
- `/cloud_input_status`：原始、通过高度过滤、体素过滤后的点数及精确配对时间。
- `/deskew_status`：历史平移速度的有效性与源时刻。

## 验证边界

30秒和90秒均真实1倍速闭环回放。前30秒使用另一次1倍速、无回环优化的 Cartographer 在线轨迹作对照，不是独立真值。量化不做空间对齐或尺度拟合。整包513秒尚未验证；不据此前30秒数据宣称整包精度。

结果见 `/home/lc/slam_11_13/results/b0_gravity_cloud_20261116/`，含 evo 原始输出及完整对照 CSV/JSON。overview 下有对应图片。
