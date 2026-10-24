#pragma once

#include <array>
#include <filesystem>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace pen_execution {
using Pose = std::array<double, 7>;  // x,y,z,qx,qy,qz,qw
struct Sample {
  double time_s;
  Pose pose;
  std::string phase;
  bool extrude;
};
struct Job {
  std::filesystem::path path;
  std::string digest;
  bool fake_only;
  bool calibrated;
  std::string profile_sha256;
  YAML::Node source;
  std::vector<Sample> samples;
};

std::string sha256(const std::string& bytes);
double distance(const Pose& a, const Pose& b);
double angular_distance(const Pose& a, const Pose& b);
bool valid_pose(const Pose& pose);
Pose blend_pose(const Pose& a, const Pose& b, double fraction);
Job parse_job(const std::string& bytes);
Job load_job(const std::filesystem::path& path, const std::filesystem::path& root,
             const std::string& expected_sha256);
}  // namespace pen_execution
