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
TEST(Job, HomeJobsMustKeepPenOff) {
  auto job=valid_job();job["purpose"]="joint_home";
  EXPECT_THROW(pen_execution::parse_job(YAML::Dump(job)),std::runtime_error);
  job["purpose"]="joint_retreat";
  EXPECT_THROW(pen_execution::parse_job(YAML::Dump(job)),std::runtime_error);
  for (auto row:job["samples"]) row["extrude"]=false;
  YAML::Node events(YAML::NodeType::Sequence);
  YAML::Node off;off["time_s"]=0.;off["extrude"]=false;events.push_back(off);job["pen_events"]=events;
  EXPECT_NO_THROW(pen_execution::parse_job(YAML::Dump(job)));
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
  job=valid_job(); job["pen"]["recycle_interval_s"]=0.005;
  EXPECT_THROW(pen_execution::parse_job(YAML::Dump(job)),std::runtime_error);
  job["pen"]["recycle_interval_s"]=0.02;
  EXPECT_NO_THROW(pen_execution::parse_job(YAML::Dump(job)));
}
TEST(Job, UsesSeparateStrokeAndRecyclePressDelays) {
  auto job=valid_job();
  job["pen"]["press_delay_s"]=.04;
  job["pen"]["recycle_press_delay_s"]=.02;
  job["pen"]["release_delay_s"]=0.;
  job["samples"]=YAML::Node(YAML::NodeType::Sequence);
  for (int i=0;i<=12;++i) {
    YAML::Node sample;
    sample["time_s"]=i*.01;
    sample["extrude"]=(i>=1 && i<=5) || (i>=8 && i<=10);
    sample["phase"]=i==0 ? "HOLD" : i==12 ? "LIFT" :
      sample["extrude"].as<bool>() ? "DRAW" : "RELEASE";
    sample["pose"]=std::vector<double>{i>=10 ? .002 : i>=5 ? .001 : 0.,0.,i==12 ? .001 : 0.,0.,0.,0.,1.};
    job["samples"].push_back(sample);
  }
  job["pen_events"]=YAML::Load("[{time_s: 0, extrude: false}, {time_s: 0.01, extrude: true}, {time_s: 0.06, extrude: false}, {time_s: 0.08, extrude: true}, {time_s: 0.11, extrude: false}]");
  EXPECT_NO_THROW(pen_execution::parse_job(YAML::Dump(job)));
  job["pen"].remove("recycle_press_delay_s");
  EXPECT_THROW(pen_execution::parse_job(YAML::Dump(job)),std::runtime_error);
  job["pen"]["recycle_press_delay_s"]=.02;
  job["samples"][2]["pose"][0]=.001;
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

TEST(Job, HomeConnectorIsOnlyUsedFromItsCheckedStart) {
  YAML::Node routes;
  const std::vector<double> home(7,0.2), park(7,0.0);
  routes["approach"]["start_positions"]=park;
  routes["home_connector"]["checked_joint_home"]=true;
  routes["home_connector"]["start_positions"]=home;
  routes["home_connector"]["points"][0]["positions"]=park;
  EXPECT_FALSE(pen_execution::home_connector_for_start(routes,park));
  EXPECT_TRUE(pen_execution::home_connector_for_start(routes,home));
  EXPECT_THROW(pen_execution::home_connector_for_start(routes,std::vector<double>(7,0.5)),std::runtime_error);
  routes["home_connector"]["points"][0]["positions"]=home;
  EXPECT_THROW(pen_execution::home_connector_for_start(routes,home),std::runtime_error);
  routes["home_connector"]["points"][0]["positions"]=park;
  routes["home_connector"]["checked_joint_home"]=false;
  EXPECT_THROW(pen_execution::home_connector_for_start(routes,home),std::runtime_error);
}

TEST(Job, ApproachEndpointCanBeRecognizedWithoutRelaxingJointTolerance) {
  YAML::Node route;
  route["points"][0]["positions"]=std::vector<double>(7,0.2);
  EXPECT_TRUE(pen_execution::at_joint_route_end(route,std::vector<double>(7,0.2)));
  EXPECT_FALSE(pen_execution::at_joint_route_end(route,std::vector<double>(7,0.24)));
  EXPECT_FALSE(pen_execution::at_joint_route_end(route,std::vector<double>(6,0.2)));
}
