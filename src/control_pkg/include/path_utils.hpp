#pragma once
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace ctrl_utils {
inline double normalize_angle(double a){ while(a>M_PI)a-=2*M_PI; while(a<-M_PI)a+=2*M_PI; return a; }
inline double yaw_from_quat(const geometry_msgs::msg::Quaternion &q){
  double siny_cosp = 2.0*(q.w*q.z + q.x*q.y);
  double cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}
inline int nearest_index(const nav_msgs::msg::Path &path,double x,double y){
  int best=-1; double best_d2=std::numeric_limits<double>::infinity();
  for(size_t i=0;i<path.poses.size();++i){
    const auto &p=path.poses[i].pose.position;
    double dx=p.x-x, dy=p.y-y; double d2=dx*dx+dy*dy;
    if(d2<best_d2){ best_d2=d2; best=(int)i; }
  }
  return best;
}
struct RateLimiter {
  double max_rate; double prev; bool has_prev;
  RateLimiter(double r,double init=0.0):max_rate(r),prev(init),has_prev(false){}
  double step(double target,double dt){
    if(!has_prev){ prev=target; has_prev=true; return target; }
    double maxd=max_rate*dt;
    double delta=std::clamp(target-prev,-maxd,maxd);
    prev+=delta; return prev;
  }
};
inline double clamp(double v,double lo,double hi){ return std::max(lo,std::min(hi,v)); }
}
