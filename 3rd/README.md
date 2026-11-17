# Workspace-local third-party dependencies

`cartographer/`: exact source snapshot of the core/tool implementations used by the offline experiment, Apache-2.0; compiled and exported as hector_backend_core.

`cartographer_ros/`: original interface and RViz-plugin sources, Apache-2.0, retained for provenance; the renamed/ported Hector adapters are built from hector_backend_msgs and hector_backend_rviz.

`debs/`: one-click installer cache for missing general system development dependencies (Ceres, Eigen, Protobuf, etc). Binary package cache is not committed. System ROS2 Jazzy and its regular dependencies remain required; there is no requirement for an installed Cartographer or cartographer_ros workspace.

The workspace root `3rd` entry resolves to this directory, so cloning the repository preserves all third-party source dependencies.
