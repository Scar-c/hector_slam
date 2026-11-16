# 最终修复版 Hector：原 24 组 bag 的同语义固定地图定位

2026-11-16，仅新增运行用户指定的一种语义配置：**EKF + pre + time + desk 全开**，即原矩阵 `hector_ekf_desk_pre_time`。业务 FFT 缓存初始化，真实完整 **1 倍速**回放。对照采用原 24 组中已经验收的同配置结果，没有重跑整个矩阵。

**本轮未观察到整体 ATE 提升。** 业务连续定位 RMSE 从 3.77 cm 变为 3.95 cm；Hector 原始输出最大误差由 10.24 cm 降到 9.43 cm，但原始 RMSE 从 3.43 cm 变为 3.62 cm。不能据原始单项最大值下降就称为整体改进；两条轨迹视觉上接近。这里是每版本一次回放，毫米级 RMSE 差异不作统计显著性结论。

## 同语义与最终修复版的适配

输入保持 `/home/lc/slam_11_13` bag，地图保持 `results/business_single/lo/map.yaml + map.pgm`。已对 bag、地图、EKF 配置逐项校验 SHA256 与原 24 组一致。FFT 初值保持 x=−0.4331519525 m、y=−0.0522358196 m、yaw=−0.0261799388 rad；初始化结果 SHA256 同为 `3b3dd844c9d1e9cbc101fb9f3b0917965499581a67ed801bb547bd9d12050c35`。

| 功能 | 本次处理 |
|---|---|
| EKF | 相同的 wheel + IMU、50 Hz、two_d_mode 配置 |
| pre | 相同的 EKF 相对运动预测作为 Hector 匹配初值 |
| desk | 相同 EKF TF，按 `header.stamp + i*time_increment` 逐束补偿 |
| time | 相同 EKF TF，将前后雷达统一到合并参考时刻 |
| 初始化 | 相同缓存业务 FFT，只在开头执行 |
| 连续定位 | Hector 固定 PGM 多分辨率匹配，map→odom 与 EKF odom→base 直接组合 |
| 地图 | 所有先验层禁止写入；不建图 |
| 额外低通 | 没有加入地图校正量低通 |

直接复用 `b0_hector_gravity_cloud/hector_cloud_node.cpp` 最终版及其 Hector 核心副本，适配为固定地图 + EKF 模式，而非把背包的 IMU/Hector 历史速度模型替换进来。改动与来源见 `source_provenance.json`、`upstream_sources/` 和独立源码。

最终版的输入优化已应用：

1. 原扫描合并器保留原 `/scan_amcl` 和几何验证输出，同时新增 `/points_aligned_xyz`。新通道直接保存经过同一 EKF 逐束/时间补偿的原始 XYZ，不再通过二维 LaserScan 角度重分桶后还原。
2. `reference_attitude.cpp` 复用最终版的 Cartographer `ImuTracker`、10 s 重力时间常数与观测更新顺序，在每帧点云的精确时间戳输出四元数。该姿态仅供一次重力投影；运动去畸变和匹配预测仍来自原 EKF。
3. 匹配输入使用最终版的重力投影、重力高度区间 **[−0.8,2.0] m**、Cartographer **0.025 m 三维体素过滤**，然后传入原 Hector GN 匹配。每帧点云和姿态严格按纳秒时间戳配对。
4. 保留原矩阵的地图分辨率/三层先验金字塔、0.1–30 m 匹配范围、合并帧率10 Hz、接受阈值、EKF TF 容差0.3 s及坐标原点。合并点云在 base_link 中，匹配射线原点为 base_link 的零点，不套用背包雷达的外参常量。建图更新距离/角度阈值在固定地图模式下不参与地图更新。

由于原 EKF 为二维模式，逐束运动补偿使用其平面运动；没有为了照搬背包实验而改成独立 IMU 三维旋转 deskew。这是本次“同语义”约束下的最终修复版适配。

## 验收与结果

- 前雷达1174、后雷达1157、IMU15471、wheel2577，全部完整接收。
- XYZ 点云、参考姿态、Hector 过滤状态、匹配状态、位姿均为 **772 帧**，精确时间戳集合完全一致；772次全部接受，0拒绝。
- 业务轨迹769帧，EKF轨迹3867帧；Hector覆盖77.100 s，最大相邻间隔0.100125 s。
- PGM/YAML 发布一致；全程只有一个地图哈希，各层先验不更新。
- 原扫描 EKF 逐束几何重建：772帧，所有波束100%一致。
- 新 XYZ 通道：重建每帧所有原始束的外参及 EKF 源时刻→参考时刻变换，最大三维差约 **1.01×10⁻⁶ m**；过滤点数及平面匹配点检查通过。见运行目录 `xyz_input_validation.json`。
- 每帧原始点数平均1964.7，体素后平均921.5。此 bag 中范围/高度筛选没有排除点，因此背包场景中高度裁剪的收益没有在此数据体现；这只能解释该项作用有限，不能据此断定全部精度差异的原因。
- 匹配分数均值从旧版0.6625变为0.6958；输入采样改变后，分数上升不代表定位误差降低。

主参考与原24组一致：**建图在线原始轨迹** `business_single/lo/trajectory_online.tum`，不是独立真值。evo 1.38.1，XY 平移 APE，最近时间关联≤40 ms，不做空间对齐、尺度拟合或时间偏移搜索。

| 输出 | 指标 | 原24组同配置 | 最终修复适配版 |
|---|---|---:|---:|
| 业务 `/base_pose` | RMSE | 3.7709 cm | 3.9539 cm |
| 业务 `/base_pose` | P95 | 7.5667 cm | 8.0360 cm |
| 业务 `/base_pose` | 最大 | 11.1687 cm | 11.9414 cm |
| Hector 原始 `/slam_out_pose` | RMSE | 3.4302 cm | 3.6193 cm |
| Hector 原始 `/slam_out_pose` | P95 | 6.7768 cm | 7.1240 cm |
| Hector 原始 `/slam_out_pose` | 最大 | 10.2405 cm | 9.4293 cm |

业务两版本都关联769帧，且对应同一组参考节点；只比较公共参考节点时结论一致。原始输出各关联772帧，另取公共764个参考节点，RMSE仍为旧3.4045 cm、新3.6200 cm。附表使用 `trajectory_at_save.tum`，原始 RMSE旧3.5044 cm、新3.6902 cm，业务旧3.8722 cm、新4.0971 cm；同样没有整体提升。原生 evo ZIP、逐点误差、关联时刻、CSV与JSON均在 `comparison/`。

三模块结果图直接复用 `/home/lc/slam_11_13/scripts/plot_localization.py`，图例标注 `hector_ekf_desk_pre_time__fixed_final` 和所有功能开关；另保存新旧对比图。

## 复现与一键 RViz

```bash
bash /home/lc/slam_11_13/scripts/run_hector_ekf_desk_pre_time_fixed_localization.sh
```

默认同 bag/地图/FFT，先打开 RViz，等待5秒后从头1倍速回放，结束保留窗口。RViz 绿色点云使用实际传入最终版 Hector 的 `/hector_matching_points`。专用 ROS 域229；冲突时拒绝启动。可传新输出目录、`--no-rviz --no-hold`。

源码项目：`/home/lc/ZHUXING_V2WS-master/test/hector_fixed_prior`。构建：`bash build.sh`。本次正式原始结果：`runs/fixed_final_1x_complete`；`runs/fixed_final_1x` 是首次受限 DDS 初始化失败后的准备目录，无传感器回放，不作为测试结果。

离线复算：

```bash
MPLCONFIGDIR=/tmp/hector_carto_mpl /home/lc/ZHUXING_V2WS-master/test/hector_prior_ws/dependencies/evo_env/bin/python \
  /home/lc/ZHUXING_V2WS-master/test/hector_fixed_prior/evaluate_comparison.py
```

本次结果说明：最终输入优化成功应用并通过几何验证，但对这份原本已在厘米量级的固定地图/EKF定位数据，尚未得到整体精度收益。需要继续研究具体机制时，应针对误差集中的约15–50 s匹配段进行独立对照，而不是直接宣称优化对所有场景有效。
