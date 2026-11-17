#pragma once
#include <geometry_msgs/msg/pose.hpp>
#include "cartographer/transform/rigid_transform.h"
namespace hector_backend_rviz {
inline cartographer::transform::Rigid3d ToRigid3d(const geometry_msgs::msg::Pose&p){return {{p.position.x,p.position.y,p.position.z},{p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z}};}
}
