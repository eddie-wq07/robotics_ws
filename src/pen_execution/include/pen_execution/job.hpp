#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace pen_execution {
// Match the planner's 500k-sample cap, including dense 1 kHz filled drawings.
inline constexpr std::size_t kMaxJobSamples = 500000;
inline constexpr std::size_t kMaxJobBytes = 128 * 1024 * 1024;
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
YAML::Node home_connector_for_start(const YAML::Node& routes, const std::vector<double>& current);
bool at_joint_route_end(const YAML::Node& route, const std::vector<double>& current);
Pose blend_pose(const Pose& a, const Pose& b, double fraction);
// Worker-thread loading progress. A negative fraction means an unmeasured stage.
using JobProgress = std::function<void(const std::string&, double)>;
Job parse_job(const std::string& bytes, const JobProgress& progress = {});
Job load_job(const std::filesystem::path& path, const std::filesystem::path& root,
             const std::string& expected_sha256, const JobProgress& progress = {});
}  // namespace pen_execution
