#include "pen_execution/job.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <openssl/sha.h>
#include <set>
#include <sstream>
#include <stdexcept>

namespace pen_execution {
namespace {
void require(bool test, const char* message) {
  if (!test) throw std::runtime_error(message);
}
}

YAML::Node home_connector_for_start(const YAML::Node& routes, const std::vector<double>& current) {
  const auto approach=routes["approach"]["start_positions"].as<std::vector<double>>();
  auto matches=[](const std::vector<double>& a,const std::vector<double>& b,double tolerance) {
    if(a.size()!=7 || b.size()!=7)return false;
    for(size_t i=0;i<7;++i)
      if(!std::isfinite(a[i]) || !std::isfinite(b[i]) || std::abs(a[i]-b[i])>tolerance)return false;
    return true;
  };
  if(matches(current,approach,0.03))return YAML::Node(YAML::NodeType::Undefined);
  const auto connector=routes["home_connector"];
  require(connector && connector["checked_joint_home"] && connector["checked_joint_home"].as<bool>(),
          "Robot is outside the approach start and no checked Home connector is configured");
  require(matches(current,connector["start_positions"].as<std::vector<double>>(),0.03),
          "Robot is outside both the recorded Home and approach start");
  const auto points=connector["points"];
  require(points.IsSequence() && points.size()>0,"Home connector has no points");
  require(matches(points[points.size()-1]["positions"].as<std::vector<double>>(),approach,1e-6),
          "Home connector does not end at the saved approach start");
  return connector;
}

std::string sha256(const std::string& bytes) {
  unsigned char hash[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(bytes.data()), bytes.size(), hash);
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (const auto byte : hash) out << std::setw(2) << static_cast<unsigned>(byte);
  return out.str();
}

bool at_joint_route_end(const YAML::Node& route, const std::vector<double>& current) {
  const auto points=route["points"];
  if(!points.IsSequence() || points.size()==0 || current.size()!=7)return false;
  const auto end=points[points.size()-1]["positions"].as<std::vector<double>>();
  if(end.size()!=7)return false;
  for(size_t i=0;i<7;++i)
    if(!std::isfinite(end[i]) || !std::isfinite(current[i]) || std::abs(end[i]-current[i])>0.03)return false;
  return true;
}

double distance(const Pose& a, const Pose& b) {
  double sum = 0;
  for (size_t i = 0; i < 3; ++i) sum += (a[i] - b[i]) * (a[i] - b[i]);
  return std::sqrt(sum);
}
double angular_distance(const Pose& a, const Pose& b) {
  double dot = 0;
  for (size_t i = 3; i < 7; ++i) dot += a[i] * b[i];
  return 2 * std::acos(std::clamp(std::abs(dot), 0.0, 1.0));
}
bool valid_pose(const Pose& pose) {
  double norm = 0;
  for (size_t i = 0; i < 7; ++i) {
    if (!std::isfinite(pose[i])) return false;
    if (i >= 3) norm += pose[i] * pose[i];
  }
  return std::abs(norm - 1.0) < 0.002;
}
Pose blend_pose(const Pose& a, const Pose& b, double fraction) {
  const double u = std::clamp(fraction, 0.0, 1.0);
  Pose result{};
  for (size_t i = 0; i < 3; ++i) result[i] = a[i] + u * (b[i] - a[i]);
  double dot = 0;
  for (size_t i = 3; i < 7; ++i) dot += a[i] * b[i];
  const double sign = dot < 0 ? -1.0 : 1.0;
  const double angle = std::acos(std::clamp(std::abs(dot), 0.0, 1.0));
  const double left = angle < 1e-6 ? 1-u : std::sin((1-u)*angle)/std::sin(angle);
  const double right = angle < 1e-6 ? u : std::sin(u*angle)/std::sin(angle);
  double norm = 0;
  for (size_t i = 3; i < 7; ++i) {
    result[i] = left * a[i] + right * sign * b[i];
    norm += result[i] * result[i];
  }
  for (size_t i = 3; i < 7; ++i) result[i] /= std::sqrt(norm);
  return result;
}

Job parse_job(const std::string& bytes, const JobProgress& progress) {
  require(bytes.size() <= kMaxJobBytes, "Job exceeds 128 MiB");
  Job job;
  if (progress) progress("PARSING", -1);
  job.source = YAML::Load(bytes);
  const auto& doc = job.source;
  require(doc.IsMap(), "Job must be an object");
  require(doc["kind"].as<std::string>() == "pen_job", "Expected kind=pen_job");
  const auto version = doc["schema_version"] ? doc["schema_version"] : doc["version"];
  require(version.as<int>() == 1, "Unsupported job schema version");
  require(doc["frame"].as<std::string>() == "world", "Job requires world frame");
  require(doc["units"].as<std::string>() == "m", "Job requires metre units");
  require(doc["pose_format"].as<std::string>() == "x,y,z,qx,qy,qz,qw", "Unexpected pose format");
  job.fake_only = doc["fake_only"].as<bool>();
  job.calibrated = doc["calibrated"].as<bool>();
  job.profile_sha256 = doc["profile_sha256"].as<std::string>();
  require(job.profile_sha256.size() == 64, "Missing profile digest");
  const auto samples = doc["samples"];
  require(samples.IsSequence() && samples.size() >= 2 && samples.size() <= kMaxJobSamples,
          "Job requires 2..500000 samples");
  const std::set<std::string> phases = {"HOLD", "LOWER", "DRAW", "RELEASE", "LIFT", "TRAVEL"};
  if (progress) progress("SAMPLES", 0);
  for (const auto& row : samples) {
    Sample sample{};
    sample.time_s = row["time_s"].as<double>();
    sample.phase = row["phase"].as<std::string>();
    sample.extrude = row["extrude"].as<bool>();
    require(row["pose"].IsSequence() && row["pose"].size() == 7, "Pose needs seven values");
    for (size_t i = 0; i < 7; ++i) sample.pose[i] = row["pose"][i].as<double>();
    require(valid_pose(sample.pose), "Nonfinite or nonunit pose");
    require(std::isfinite(sample.time_s) && sample.time_s >= 0 && sample.time_s <= 3600,
            "Invalid sample time");
    require(phases.count(sample.phase), "Unknown sample phase");
    require(!sample.extrude || sample.phase == "DRAW", "Extrusion is only valid during DRAW");
    if (job.samples.empty()) {
      require(sample.time_s == 0 && !sample.extrude, "Job must start at time zero with pen off");
    } else {
      const auto& previous = job.samples.back();
      const double dt = sample.time_s - previous.time_s;
      require(dt > 0 && dt <= 0.0251, "Samples must increase and be spaced at most 25 ms apart");
      require(distance(previous.pose, sample.pose) <= 0.005, "Cartesian step exceeds 5 mm");
      require(angular_distance(previous.pose, sample.pose) <= 0.10, "Orientation step exceeds 0.10 rad");
      if (sample.phase == "LIFT" && previous.phase != "LIFT")
        require(!previous.extrude, "LIFT must be preceded by a pen-off sample");
    }
    job.samples.push_back(sample);
    if (progress && (job.samples.size() % 1024 == 0 || job.samples.size() == samples.size()))
      progress("SAMPLES", static_cast<double>(job.samples.size()) / samples.size());
  }
  require(!job.samples.back().extrude, "Job must finish with pen off");
  if (doc["purpose"])
    require(doc["purpose"].as<std::string>()!="return_home" && doc["purpose"].as<std::string>()!="flexiv_home",
            "Obsolete Home job; prepare the recorded joint-controller Home instead");
  if (doc["purpose"] && (doc["purpose"].as<std::string>()=="joint_home" ||
                         doc["purpose"].as<std::string>()=="joint_retreat"))
    require(std::none_of(job.samples.begin(),job.samples.end(),[](const auto& sample){return sample.extrude;}),
            "Homing must keep extrusion off");
  const auto pen = doc["pen"];
  require(pen.IsMap(), "Missing pen timing settings");
  const double release_delay=pen["release_delay_s"].as<double>();
  const double press_delay=pen["press_delay_s"].as<double>();
  const double recycle_press_delay=pen["recycle_press_delay_s"] ? pen["recycle_press_delay_s"].as<double>() : press_delay;
  const double press_cap=pen["max_press_s"].as<double>();
  const double recycle_cap=pen["recycle_interval_s"] ? pen["recycle_interval_s"].as<double>() : press_cap;
  require(std::isfinite(release_delay) && release_delay>=0 && release_delay<=10 &&
          std::isfinite(press_delay) && press_delay>=0 && press_delay<=10 &&
          std::isfinite(recycle_press_delay) && recycle_press_delay>=0 && recycle_press_delay<=10 &&
          std::isfinite(press_cap) && press_cap>0 && press_cap<=3600 &&
          std::isfinite(recycle_cap) && recycle_cap>0 && recycle_cap<=3600,
          "Invalid pen timing settings");
  const auto events = doc["pen_events"];
  if (progress) progress("TIMING", 0);
  require(events.IsSequence(), "Missing pen event schedule");
  std::vector<bool> event_at_sample(job.samples.size(),false);
  double last_event_time=-1;
  for (const auto& event : events) {
    const double time = event["time_s"].as<double>();
    const bool extrude = event["extrude"].as<bool>();
    require(std::isfinite(time) && time >= 0 && time <= job.samples.back().time_s &&
            time>last_event_time,
            "Invalid pen event time");
    last_event_time=time;
    auto sample = std::lower_bound(job.samples.begin(), job.samples.end(), time,
      [](const Sample& value, double t) { return value.time_s < t - 1e-8; });
    require(sample != job.samples.end() && std::abs(sample->time_s-time) < 1e-7 &&
            sample->extrude == extrude, "Pen events must agree with sampled schedule");
    event_at_sample[static_cast<size_t>(sample-job.samples.begin())]=true;
  }
  require(event_at_sample.front(), "Missing initial pen-off event");
  double pressed_at=-1, released_at=-1;
  double active_press_delay=press_delay;
  bool new_stroke=true;
  Pose pressed_pose{},released_pose{};
  for (size_t i=0;i<job.samples.size();++i) {
    const auto& sample=job.samples[i];
    if (!sample.extrude && (sample.phase=="LOWER" || sample.phase=="LIFT" || sample.phase=="TRAVEL"))
      new_stroke=true;
    if (i && sample.extrude!=job.samples[i-1].extrude) {
      require(event_at_sample[i], "Every extrusion transition needs an explicit event");
      if (sample.extrude) {
        active_press_delay=new_stroke ? press_delay : recycle_press_delay;
        new_stroke=false;
        pressed_at=sample.time_s; pressed_pose=sample.pose;
      } else {
        require(pressed_at>=0 && sample.time_s-pressed_at<=std::min(press_cap,recycle_cap)+1e-8,
                "Continuous extrusion exceeds press or recycle interval");
        released_at=sample.time_s; released_pose=sample.pose; pressed_at=-1;
      }
    }
    if (sample.extrude && pressed_at>=0) {
      require(sample.time_s-pressed_at<=std::min(press_cap,recycle_cap)+1e-8,
              "Continuous extrusion exceeds press or recycle interval");
      if (sample.time_s-pressed_at<active_press_delay-1e-8)
        require(distance(sample.pose,pressed_pose)<1e-5 && angular_distance(sample.pose,pressed_pose)<0.001,
                "Motion begins before pen press delay completes");
    }
    if (!sample.extrude && released_at>=0 && sample.time_s-released_at<release_delay-1e-8) {
      require(sample.phase!="LIFT" && sample.phase!="TRAVEL",
              "Lift or travel begins before pen release delay completes");
      require(distance(sample.pose,released_pose)<1e-5 && angular_distance(sample.pose,released_pose)<0.001,
              "Motion begins before pen release delay completes");
    }
    if (progress && ((i+1) % 1024 == 0 || i+1 == job.samples.size()))
      progress("TIMING", static_cast<double>(i+1) / job.samples.size());
  }
  job.digest = sha256(bytes);
  return job;
}

Job load_job(const std::filesystem::path& path, const std::filesystem::path& root,
             const std::string& expected_sha256, const JobProgress& progress) {
  if (progress) progress("READING", -1);
  const auto canonical = std::filesystem::canonical(path);
  const auto root_path = std::filesystem::canonical(root);
  const auto relative = canonical.lexically_relative(root_path);
  require(!relative.empty() && !relative.is_absolute(), "Job is outside jobs_root");
  for (const auto& part : relative) require(part != "..", "Job is outside jobs_root");
  require(std::filesystem::is_regular_file(canonical), "Job is not a regular file");
  require(std::filesystem::file_size(canonical) <= kMaxJobBytes, "Job exceeds 128 MiB");
  std::ifstream stream(canonical, std::ios::binary);
  require(stream.good(), "Unable to read job");
  std::string bytes((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
  if (progress) progress("HASHING", -1);
  require(expected_sha256.size() == 64 && sha256(bytes) == expected_sha256,
          "Job changed since preview or digest is invalid");
  auto job = parse_job(bytes, progress);
  job.path = canonical;
  return job;
}
}  // namespace pen_execution
