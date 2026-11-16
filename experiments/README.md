# 本机实验源码快照

这些文件记录Jazzy开发期间的原始实验与分析实现，不参与colcon构建。**历史脚本仍引用原业务工作区路径和外部ROS包；不能直接当成通用安装入口。** 可安装运行的代码已整合到根目录的`hector_mapping`与`hector_cloud_frontend`。

| 目录 | 内容与要求 |
|---|---|
| `local_business_matrix` | 12配置×2共享初始化、evo与CPU/PSS工具；依赖原业务地图/bag、zx_msgs、zx_localization、URDF及缓存FFT计算器 |
| `fixed_prior_snapshot` | 最终输入链在原EKF/pre/desk/time语义上的适配和验收；包含精确XYZ重建与新旧evo对照 |
| `b0_final_snapshot` | 最终背包XYZ/共享姿态/体素链，依赖原本机Cartographer兼容库 |
| `b0_ablation_snapshot` | 前30s输入开关消融、官方行为只加体素的对照源码 |

原业务地图/bag根目录为`/home/lc/slam_11_13`，开发工作区为`/home/lc/ZHUXING_V2WS-master`，公开背包转换包在`/home/lc/Downloads/b0-2014-07-11-11-00-49_ros2`。这些原始数据未提交。复制目录中的旧README是带日期的历史记录（部分完整跑包前的“待验证”表述保留用于追溯），**最终结论以`docs/JAZZY_CHANGES.md`为准**。

本机可继续用已验收的一键入口：

```bash
bash /home/lc/slam_11_13/scripts/run_hector_ekf_desk_pre_time_fixed_localization.sh
```

该入口默认打开RViz、等待5秒、完整1倍速回放，结束保留窗口；新增结果在`results`，三模块图和对比图在`overview`。可移植入口及依赖见根README。
