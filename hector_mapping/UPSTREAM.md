# Provenance

Core source: https://github.com/tu-darmstadt-ros-pkg/hector_slam
Branch: noetic-devel
Commit: a5e77fd3297055f5f0776ed56fafa60790494e98
Retrieved: 2026-11-12
Original location: hector_mapping/include/hector_slam_lib

The BSD-3-Clause notices are preserved in each vendored file and LICENSE.
The ROS 2 adapter and build/launch files are new and use the same license.
This is an independent local port, not an upstream ROS 2 release.

Core changes relative to that commit:
- Remove ROS 1 tf/geometry_msgs helper from UtilFunctions.h; include Eigen directly.
- Refresh coarse-level scans during known-pose mapping, where matching is bypassed.
- Add iostream include and initialize the stored Hessian on reset.
- Use checked FullPivLU instead of an unchecked Hessian inverse. Singular/nonfinite
  steps keep the current pose; the Gauss-Newton scan-matching objective is unchanged.
- Reject nonfinite coordinates before map indexing and reserve the last map
  row/column for bilinear interpolation's neighbour access.

New adapter uses a serialized executor instead of the original map-publisher thread.
Laser projection applies the full laser-to-base TF to endpoints and ray origin.
Metadata checks and finite range filtering replace the ROS 1 laser_geometry adapter.
The initial adapter optionally integrates gyro and uses gravity complementary attitude
correction for scan yaw prediction and roll/pitch projection. The initial adapter
did not add deskew; later optional frontends are described below. The map origin is the corner half a cell below
and left of core grid coordinate (0,0), rather than the ROS 1 /1000 offset.
The upstream Hessian accessor is converted to a local pose covariance approximation.

Fixed-prior localization extension (2026-11-13):
- Import PGM/YAML trinary rasters into all pyramid levels; coarse occupied cells
  win, and unknown children remain unknown unless an occupied child is present.
- Core world coordinates are local metres from the raster corner; the adapter
  transforms hints, published poses and covariance using the YAML origin/yaw.
- Add core localizationOnly guard to block updateByScan, including known-pose
  calls; reset clears tracking state while retaining the fixed prior.
- Add stateless Hector matching and accepted-pose commit with Hessian evaluation.
- A fixed endpoint-distance field selects initial seeds (bounded or global) and
  gates bad matches; continuous optimization remains Hector Gauss-Newton.
- Runtime fingerprint checks all pyramid occupancy values on every map publish.

Optional original business EKF integration:
- Use the EKF odom->base transform at the scan timestamp to propagate the last
  accepted raw map correction. No duplicate gyro yaw prediction is added.
- Directly compute map->odom = raw Hector map->base * inverse(odom->base).
  No low-pass filter or correction blending is present.
- Expose the original AMCL TF transform_tolerance (0.3 seconds for the replay).
- Reuse the business base_pose_from_tf logic in an isolated executable; remove
  its AMCL lifecycle gate because the Hector pipeline contains no AMCL node.

Jazzy integration and final input frontends (2026-11-16):
- Add independent EKF per-beam deskew and common-reference frame alignment.
- Preserve original compensated XYZ alongside the legacy rebinned LaserScan.
- Add hector_cloud_frontend mapping and fixed-prior adapters using exact-time
  gravity reference pairing, gravity-height filtering and 0.025 m 3D voxel input.
- Reuse Cartographer ImuTracker and VoxelFilter as separately licensed utility
  dependencies, never its scan matching or pose graph backend.
- Keep the original EKF/pre/desk/time semantics in fixed-prior comparisons;
  historical Hector velocity deskew is a separate backpack mapping mode.
- Preserve Noetic history and mark unported ROS 1 auxiliary packages ignored.
- Detailed source inventory and measured positive/negative results are recorded
  in docs/JAZZY_CHANGES.md at the repository root.
