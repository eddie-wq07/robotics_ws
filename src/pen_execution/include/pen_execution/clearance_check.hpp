#pragma once

#include "pen_execution/job.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pen_execution {
// A one-job permit for commissioning a raised, extrusion-OFF path. It does not
// qualify the surface for printing and never accepts an arbitrary drawing.
inline bool clearance_check_job(const Job& job) {
  return job.source["purpose"] && job.source["purpose"].as<std::string>() == "air_clearance_check";
}
inline void validate_clearance_check(const Job& job, const std::string& permit,
                                     const std::string& serial) {
  auto require=[](bool ok, const char* why) { if(!ok)throw std::runtime_error(why); };
  require(permit.size()==64 && job.digest==permit, "Air clearance check lacks an exact-job permit");
  require(job.source["robot_sn"].as<std::string>()==serial, "Clearance check robot identity mismatch");
  require(job.source["air_run"] && job.source["air_run"].as<bool>(), "Clearance check must be an air run");
  require(job.samples.size()>=2 && job.samples.back().time_s<=180., "Clearance check exceeds 180 seconds");
  require(std::none_of(job.samples.begin(),job.samples.end(),[](const Sample& s){return s.extrude;}),
          "Clearance check forbids all extrusion");
  const auto profile=job.source["profile"];
  require(profile["surface"]["type"].as<std::string>()=="plane", "Clearance check requires a plane");
  const auto approved=job.source["clearance_check"];
  require(approved && approved["operator_route_confirmed"] && approved["operator_route_confirmed"].as<bool>(),
          "Clearance check requires operator route confirmation");
  auto vec=[](const YAML::Node& node) {
    auto v=node.as<std::vector<double>>();
    if(v.size()!=3 || !std::all_of(v.begin(),v.end(),[](double x){return std::isfinite(x);}))
      throw std::runtime_error("Invalid clearance geometry vector");
    return std::array<double,3>{v[0],v[1],v[2]};
  };
  auto rotate=[](const std::array<double,4>& q, const std::array<double,3>& v) {
    const std::array<double,3> t{2*(q[1]*v[2]-q[2]*v[1]),2*(q[2]*v[0]-q[0]*v[2]),2*(q[0]*v[1]-q[1]*v[0])};
    return std::array<double,3>{v[0]+q[3]*t[0]+q[1]*t[2]-q[2]*t[1],
      v[1]+q[3]*t[1]+q[2]*t[0]-q[0]*t[2],v[2]+q[3]*t[2]+q[0]*t[1]-q[1]*t[0]};
  };
  auto quat=[](const YAML::Node& node) {
    auto q=node.as<std::vector<double>>();double n=0;
    for(double x:q)n+=x*x;
    if(q.size()!=4 || !std::isfinite(n) || std::abs(n-1)>1e-6)throw std::runtime_error("Invalid clearance geometry quaternion");
    return std::array<double,4>{q[0],q[1],q[2],q[3]};
  };
  const auto origin=vec(profile["T_world_surface"]["xyz_m"]);
  const auto normal=rotate(quat(profile["T_world_surface"]["quaternion_xyzw"]),{0,0,1});
  const auto tcp=vec(profile["T_flange_rdk_tcp"]["xyz_m"]);
  const auto tip=vec(profile["T_flange_pen_tip"]["xyz_m"]);
  const auto tcp_q=quat(profile["T_flange_rdk_tcp"]["quaternion_xyzw"]);
  const auto tip_q=quat(profile["T_flange_pen_tip"]["quaternion_xyzw"]);
  require(std::abs(tcp_q[0])+std::abs(tcp_q[1])+std::abs(tcp_q[2])+std::abs(tip_q[0])+std::abs(tip_q[1])+std::abs(tip_q[2])<1e-9,
          "Clearance check requires flange-aligned tip and TCP axes");
  const double radius=profile["tool_envelope"]["radius_m"].as<double>();
  const double length=profile["tool_envelope"]["length_m"].as<double>();
  require(std::isfinite(radius) && std::isfinite(length) && radius>0 && length>=2*radius,
          "Invalid clearance tool envelope");
  require(distance(job.samples.front().pose,job.samples.back().pose)<.0005 &&
          angular_distance(job.samples.front().pose,job.samples.back().pose)<.005,
          "Clearance check must return to its exact starting pose");
  std::array<double,3> previous_velocity{}; double previous_dt=0;
  for(size_t i=0;i<job.samples.size();++i) {
    const auto& s=job.samples[i];const auto& p=s.pose;
    const std::array<double,4> q{p[3],p[4],p[5],p[6]};
    const auto offset=rotate(q,{tip[0]-tcp[0],tip[1]-tcp[1],tip[2]-tcp[2]});
    const auto axis=rotate(q,{0,0,-1});
    double tip_height=0,axis_dot=0;
    for(size_t k=0;k<3;++k){tip_height+=(p[k]+offset[k]-origin[k])*normal[k];axis_dot+=axis[k]*normal[k];}
    const double gap=std::min(tip_height+radius*axis_dot,tip_height+(length-radius)*axis_dot)-radius;
    require(gap>=.019, "Clearance check tool envelope is less than 19 mm above the plane");
    require(distance(p,job.samples.front().pose)<=.25, "Clearance check exceeds its 250 mm local workspace");
    if(i) {
      const auto& old=job.samples[i-1];const double dt=s.time_s-old.time_s;
      require(distance(p,old.pose)/dt<=.006, "Clearance check exceeds 6 mm/s");
      require(angular_distance(p,old.pose)/dt<=.1, "Clearance check exceeds angular speed bound");
      std::array<double,3> velocity{};double a2=0;
      for(size_t k=0;k<3;++k){velocity[k]=(p[k]-old.pose[k])/dt;
        if(i>1){const double a=(velocity[k]-previous_velocity[k])/((dt+previous_dt)*.5);a2+=a*a;}}
      require(i<2 || std::sqrt(a2)<=.15, "Clearance check exceeds acceleration bound");
      previous_velocity=velocity;previous_dt=dt;
    }
  }
}
}  // namespace pen_execution
