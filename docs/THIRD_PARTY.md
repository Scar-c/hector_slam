# 来源与许可证

- Hector SLAM 核心来自 TU Darmstadt `hector_slam` 的 `noetic-devel` 提交 `a5e77fd3297055f5f0776ed56fafa60790494e98`。保留原 BSD-3-Clause 版权与许可证，具体差异见 `hector_mapping/UPSTREAM.md` 和 `JAZZY_CHANGES.md`。
- ROS 2接口、先验定位、EKF及输入适配沿用BSD-3-Clause，业务来源注释保留。
- `hector_cloud_frontend` 链接 Cartographer 的 `ImuTracker` 与 `sensor::VoxelFilter` C++工具。Cartographer 为Apache-2.0；库由外部依赖提供，安装包遗漏的体素过滤API声明头以原样保留在`hector_cloud_frontend/third_party/cartographer`，附原许可证及SHA256。本仓库未冒充这些工具是Hector原生算法，也未复制/运行其建图后端。业务实验原来链接兼容的`zx_carto`。
- `experiments/local_business_matrix/initializers` 调用业务工作区已有的 `AdaptiveFftMatcher`，需要其源目录、配置及FFTW/OpenCV等依赖。未将外部业务FFT核心复制进本仓库。
- 离线指标使用evo1.38.1；其环境及二进制不打包进仓库。图像为本地实验输出，原始传感器bag不提交。
