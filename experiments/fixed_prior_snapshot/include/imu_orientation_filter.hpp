// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>
#include <Eigen/Geometry>

namespace hector_mapping_ros2 {
// Gravity-referenced attitude filter. Gyro supplies short-term rotation; gravity
// (or a valid sensor orientation) corrects roll/pitch without imposing absolute yaw.
class ImuOrientationFilter {
public:
  struct State { double time; Eigen::Quaterniond orientation; Eigen::Vector3d gyro; };
  static double yaw(const Eigen::Quaterniond &q) {
    return std::atan2(2*(q.w()*q.z()+q.x()*q.y()), 1-2*(q.y()*q.y()+q.z()*q.z()));
  }
  static Eigen::Quaterniond heading(double angle) {
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
  }
  static Eigen::Quaterniond delta(const Eigen::Vector3d &gyro, double dt) {
    const double magnitude=gyro.norm();
    return magnitude < 1e-12 ? Eigen::Quaterniond::Identity() :
      Eigen::Quaterniond(Eigen::AngleAxisd(magnitude*dt, gyro/magnitude));
  }
  static Eigen::Quaterniond gravityTilt(const Eigen::Vector3d &acceleration) {
    const double roll=std::atan2(acceleration.y(), acceleration.z());
    const double pitch=std::atan2(-acceleration.x(), std::hypot(acceleration.y(), acceleration.z()));
    return Eigen::Quaterniond(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
  }
  void reset() { states_.clear(); }
  std::optional<double> latestTime() const { return states_.empty() ? std::nullopt : std::optional<double>(states_.back().time); }
  bool add(double time, const Eigen::Vector3d &gyro, const Eigen::Vector3d &acceleration,
           double gravity, double time_constant,
           const std::optional<Eigen::Quaterniond> &measured_orientation=std::nullopt)
  {
    if (!std::isfinite(time) || !gyro.allFinite() ||
        (!states_.empty() && time <= states_.back().time)) return false;
    std::optional<Eigen::Quaterniond> tilt;
    if (measured_orientation && measured_orientation->coeffs().allFinite() && measured_orientation->norm()>1e-6) {
      const Eigen::Quaterniond q=measured_orientation->normalized();
      tilt=heading(-yaw(q))*q;
    } else if (acceleration.allFinite() && std::abs(acceleration.norm()-gravity)<0.2*gravity) {
      tilt=gravityTilt(acceleration);
    }
    Eigen::Quaterniond q;
    if (states_.empty()) {
      if (!tilt) return false;
      q=*tilt;
    } else {
      const auto &previous=states_.back();
      const double dt=time-previous.time;
      // A gap must not be integrated as one huge turn.
      q=dt<=0.1 ? (previous.orientation*delta(0.5*(previous.gyro+gyro), dt)).normalized() : previous.orientation;
      if (tilt) {
        const Eigen::Quaterniond target=heading(yaw(q))*(*tilt);
        q=q.slerp(1-std::exp(-dt/time_constant), target).normalized();
      }
    }
    states_.push_back({time,q,gyro});
    while (states_.size()>2 && time-states_.front().time>5.0) states_.pop_front();
    return true;
  }
  std::optional<Eigen::Quaterniond> at(double time, double tolerance, bool allow_extrapolation=true) const {
    if (states_.empty() || time<states_.front().time) return std::nullopt;
    auto next=std::lower_bound(states_.begin(), states_.end(), time,
      [](const State &state, double t){ return state.time<t; });
    if (next==states_.end()) {
      const auto &last=states_.back();
      if (!allow_extrapolation || time-last.time>tolerance) return std::nullopt;
      return (last.orientation*delta(last.gyro,time-last.time)).normalized();
    }
    if (next==states_.begin() || next->time==time) return next->orientation;
    const auto previous=std::prev(next);
    if (next->time-previous->time>tolerance) return std::nullopt;
    return previous->orientation.slerp((time-previous->time)/(next->time-previous->time),next->orientation).normalized();
  }
private:
  std::deque<State> states_;
};
}  // namespace hector_mapping_ros2
