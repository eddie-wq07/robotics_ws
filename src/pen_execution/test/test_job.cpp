#include "pen_execution/job.hpp"
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <unistd.h>

namespace {
YAML::Node valid_job() {
  return YAML::Load(R"(
schema_version: 1
kind: pen_job
frame: world
units: m
pose_format: x,y,z,qx,qy,qz,qw
fake_only: true
calibrated: false
profile_sha256: aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
pen: {release_delay_s: 0.01, press_delay_s: 0, max_press_s: 30}
samples:
  - {time_s: 0.0, pose: [0,0,0,0,0,0,1], phase: HOLD, extrude: false}
  - {time_s: 0.01, pose: [0.001,0,0,0,0,0,1], phase: DRAW, extrude: true}
  - {time_s: 0.02, pose: [0.001,0,0,0,0,0,1], phase: RELEASE, extrude: false}
  - {time_s: 0.03, pose: [0.001,0,0.001,0,0,0,1], phase: LIFT, extrude: false}
pen_events:
  - {time_s: 0.0, extrude: false}
  - {time_s: 0.01, extrude: true}
  - {time_s: 0.02, extrude: false}
)");
}
}

TEST(Job, ValidTimelineHasExplicitReleaseBeforeLift) {
  const auto job=pen_execution::parse_job(YAML::Dump(valid_job()));
  ASSERT_EQ(job.samples.size(),4u);
  EXPECT_TRUE(job.samples[1].extrude);
  EXPECT_FALSE(job.samples[2].extrude);
  EXPECT_EQ(job.samples[3].phase,"LIFT");
}
TEST(Job, RejectsExtrusionDuringLift) {
  auto job=valid_job(); job["samples"][3]["extrude"]=true;
  EXPECT_THROW(pen_execution::parse_job(YAML::Dump(job)),std::runtime_error);
}
TEST(Job, RejectsLiftWithoutPrecedingRelease) {
  auto job=valid_job(); job["samples"][2]["phase"]="DRAW"; job["samples"][2]["extrude"]=true;
  EXPECT_THROW(pen_execution::parse_job(YAML::Dump(job)),std::runtime_error);
}
TEST(Job, RejectsStaleIntervalsAndInvalidQuaternions) {
  auto job=valid_job(); job["samples"][1]["time_s"]=0.2;
  EXPECT_THROW(pen_execution::parse_job(YAML::Dump(job)),std::runtime_error);
  job=valid_job(); job["samples"][1]["pose"][6]=0;
  EXPECT_THROW(pen_execution::parse_job(YAML::Dump(job)),std::runtime_error);
}
TEST(Job, RejectsEventDisagreementAndWrongFrame) {
  auto job=valid_job(); job["pen_events"][2]["extrude"]=true;
  EXPECT_THROW(pen_execution::parse_job(YAML::Dump(job)),std::runtime_error);
  job=valid_job(); job["frame"]="unregistered_surface";
  EXPECT_THROW(pen_execution::parse_job(YAML::Dump(job)),std::runtime_error);
}
TEST(Job, RequiresAllExtrusionTransitionsInEventSchedule) {
  auto job=valid_job();
  job["pen_events"].remove(1);
  EXPECT_THROW(pen_execution::parse_job(YAML::Dump(job)),std::runtime_error);
}
TEST(Job, EnforcesReleaseDwellBeforeLiftAndContinuousPressCap) {
  auto job=valid_job(); job["pen"]["release_delay_s"]=0.1;
  EXPECT_THROW(pen_execution::parse_job(YAML::Dump(job)),std::runtime_error);
  job=valid_job(); job["pen"]["max_press_s"]=0.005;
  EXPECT_THROW(pen_execution::parse_job(YAML::Dump(job)),std::runtime_error);
}
TEST(Job, HashAndRootProtectReviewedArtifact) {
  const auto base=std::filesystem::temp_directory_path()/ ("pen-execution-test-"+std::to_string(getpid()));
  std::filesystem::create_directories(base/"allowed");
  const auto file=base/"allowed"/"job.json";
  const auto bytes=YAML::Dump(valid_job());
  { std::ofstream output(file); output << bytes; }
  EXPECT_NO_THROW(pen_execution::load_job(file,base/"allowed",pen_execution::sha256(bytes)));
  EXPECT_THROW(pen_execution::load_job(file,base/"allowed",std::string(64,'0')),std::runtime_error);
  std::filesystem::copy_file(file,base/"outside.json");
  EXPECT_THROW(pen_execution::load_job(base/"outside.json",base/"allowed",pen_execution::sha256(bytes)),std::runtime_error);
  std::filesystem::remove_all(base);
}
TEST(Job, QuaternionInterpolationTakesShortArcAndStaysUnit) {
  pen_execution::Pose a={0,0,0,0,0,0,1};
  pen_execution::Pose b={1,2,3,0,0,0,-1};
  const auto mid=pen_execution::blend_pose(a,b,0.5);
  EXPECT_TRUE(pen_execution::valid_pose(mid));
  EXPECT_NEAR(mid[0],0.5,1e-9);
  EXPECT_NEAR(pen_execution::angular_distance(a,mid),0,1e-9);
}
