// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// Tests of the Qt-free core: parsing, classification, detection, the plan, the emitters and the
// logic of the verification. No test framework; a failed CHECK is counted and printed.
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <set>

#include "gz_bridge_helper/project.hpp"
#include "gz_bridge_helper/types.hpp"
#include "gz_bridge_helper/verify.hpp"

#ifndef GZBH_TEST_DATA
#error "GZBH_TEST_DATA must point at test/data"
#endif

using namespace gzb;

namespace {

int g_checks = 0, g_failed = 0;

void report(bool ok, const char* expr, const char* file, int line, const std::string& extra = "") {
  ++g_checks;
  if (!ok) {
    ++g_failed;
    std::cerr << file << ":" << line << ": CHECK failed: " << expr << (extra.empty() ? "" : "  (" + extra + ")") << "\n";
  }
}

#define CHECK(cond) report((cond), #cond, __FILE__, __LINE__)
#define CHECK_EQ(a, b) report((a) == (b), #a " == " #b, __FILE__, __LINE__, "got '" + toText(a) + "' and '" + toText(b) + "'")
#define CHECK_NEAR(a, b, eps) report(std::fabs((a) - (b)) <= (eps), #a " ~ " #b, __FILE__, __LINE__, "got " + std::to_string(a) + " and " + std::to_string(b))

std::string toText(const std::string& s) { return s; }
std::string toText(const char* s) { return s; }
std::string toText(double v) { return std::to_string(v); }
std::string toText(int v) { return std::to_string(v); }
std::string toText(size_t v) { return std::to_string(v); }
std::string toText(bool v) { return v ? "true" : "false"; }

std::string data(const std::string& name) { return std::string(GZBH_TEST_DATA) + "/" + name; }

std::string slurp(const std::string& path) {
  std::string text, err;
  if (!readTextFile(path, text, err)) {
    std::cerr << "cannot read fixture " << path << "\n";
    std::exit(2);
  }
  return text;
}

bool hasIssue(const Issues& issues, Severity s, const std::string& needle) {
  for (const Issue& i : issues) {
    if (i.severity == s && i.message.find(needle) != std::string::npos) return true;
  }
  return false;
}

std::string issuesText(const Issues& issues) {
  std::string out;
  for (const Issue& i : issues) out += std::string(severityName(i.severity)) + ": " + i.message + "\n";
  return out;
}

const BridgeEntry* findEntry(const Plan& p, const std::string& rosTopic) {
  for (const BridgeEntry& e : p.bridge) {
    if (e.rosTopic == rosTopic && e.enabled) return &e;
  }
  return nullptr;
}

std::string tempDir(const std::string& name) {
  namespace fs = std::filesystem;
  const fs::path p = fs::temp_directory_path() / ("gzbh_test_" + name);
  std::error_code ec;
  fs::remove_all(p, ec);
  fs::create_directories(p);
  return p.string();
}

Project load(const std::string& urdf, const std::vector<std::string>& yamls = {}) {
  Project p;
  p.loadUrdfFile(data(urdf));
  for (const std::string& y : yamls) p.addYamlFile(data(y));
  return p;
}

// ---------------------------------------------------------------------------------- common

void testCommon() {
  CHECK_EQ(num(0.1), "0.1");
  CHECK_EQ(num(-2.5), "-2.5");
  CHECK_EQ(num(1e-15), "0");
  CHECK_EQ(num(0.2400000001), "0.24");
  CHECK_EQ(num(100.0), "100");

  const Transform rz = Transform::fromXyzRpy({1, 2, 3}, {0, 0, kPi / 2});
  const Vec3 p = rz.apply({1, 0, 0});
  CHECK_NEAR(p.x, 1.0, 1e-12);
  CHECK_NEAR(p.y, 3.0, 1e-12);
  CHECK_NEAR(p.z, 3.0, 1e-12);
  // Roll, pitch, yaw order: Rz * Ry * Rx.
  const Transform t = Transform::fromXyzRpy({0, 0, 0}, {kPi / 2, kPi / 2, 0});
  const Vec3 q = t.rotate({0, 1, 0});  // Rx turns y into z, Ry turns z into x
  CHECK_NEAR(q.x, 1.0, 1e-12);
  CHECK_NEAR(q.y, 0.0, 1e-12);
  CHECK_NEAR(q.z, 0.0, 1e-12);
  const Transform ab = Transform::fromXyzRpy({1, 0, 0}, {0, 0, kPi / 2}) * Transform::fromXyzRpy({1, 0, 0}, {0, 0, 0});
  CHECK_NEAR(ab.t.x, 1.0, 1e-12);
  CHECK_NEAR(ab.t.y, 1.0, 1e-12);

  CHECK_EQ(yamlScalar("/clock"), "/clock");
  CHECK_EQ(yamlScalar("gz.msgs.Clock"), "gz.msgs.Clock");
  CHECK_EQ(yamlScalar("true"), "\"true\"");
  CHECK_EQ(yamlScalar("12"), "\"12\"");
  CHECK_EQ(yamlScalar("a b"), "\"a b\"");
  CHECK_EQ(yamlScalar(""), "\"\"");
  CHECK_EQ(yamlScalar("say \"x\""), "\"say \\\"x\\\"\"");
  CHECK_EQ(pyString("it's"), "'it\\'s'");
  CHECK_EQ(closestMatch("lazzy", {"lazy", "qos_profile"}), "lazy");
  CHECK_EQ(closestMatch("zzzzzzzz", {"lazy", "qos_profile"}), "");
  CHECK_EQ(editDistance("kitten", "sitting"), 3);
  CHECK(validIdentifier("a_1"));
  CHECK(!validIdentifier("1a"));
  CHECK(!validIdentifier("a-b"));
  CHECK_EQ(baseName("/x/y/z.urdf"), "z.urdf");
  CHECK_EQ(stem("/x/y/z.urdf"), "z");
  CHECK_EQ(xmlEscape("a<b&\"c\""), "a&lt;b&amp;&quot;c&quot;");

  const std::string dir = tempDir("io");
  std::string err;
  CHECK(writeTextFile(dir + "/a/b/c.txt", "hello", err));
  std::string back;
  CHECK(readTextFile(dir + "/a/b/c.txt", back, err));
  CHECK_EQ(back, "hello");
  CHECK(!readTextFile(dir + "/missing.txt", back, err));
  CHECK(!err.empty());
}

// ----------------------------------------------------------------------------------- types

void testTypes() {
  CHECK_EQ(messageTypePairs().size(), static_cast<size_t>(71));
  CHECK_EQ(serviceTypePairs().size(), static_cast<size_t>(4));
  CHECK(isKnownPair("sensor_msgs/msg/Imu", "gz.msgs.IMU"));
  CHECK(isKnownPair("sensor_msgs/msg/JointState", "gz.msgs.Model"));
  CHECK(isKnownPair("tf2_msgs/msg/TFMessage", "gz.msgs.Pose_V"));
  CHECK(isKnownPair("rosgraph_msgs/msg/Clock", "gz.msgs.Clock"));
  CHECK(isKnownPair("std_msgs/msg/Float64", "gz.msgs.Double"));
  CHECK(!isKnownPair("sensor_msgs/msg/Imu", "gz.msgs.Twist"));
  CHECK(knownRosType("nav_msgs/msg/Odometry"));
  CHECK(!knownRosType("nav_msgs/msg/Odometri"));
  CHECK(knownGzType("gz.msgs.PointCloudPacked"));
  CHECK_EQ(gzTypesFor("nav_msgs/msg/Odometry").size(), static_cast<size_t>(2));
  CHECK_EQ(gzTypesFor("sensor_msgs/msg/JointState").front(), "gz.msgs.Model");
  const auto rosForPose = rosTypesFor("gz.msgs.Pose");
  CHECK(std::find(rosForPose.begin(), rosForPose.end(), "geometry_msgs/msg/PoseStamped") != rosForPose.end());
  CHECK(gzTypesFor("not/a/Type").empty());
}

// ------------------------------------------------------------------------------------ URDF

UrdfModel parse(const std::string& xml, Issues& issues) { return parseUrdf(xml, issues); }

void testUrdf() {
  {
    Issues is;
    const UrdfModel m = parse(slurp(data("diffbot_plain.urdf")), is);
    CHECK(m.valid);
    CHECK(!hasErrors(is));
    CHECK_EQ(m.name, "diffbot");
    CHECK_EQ(m.root, "base_link");
    CHECK_EQ(m.links.size(), static_cast<size_t>(4));
    CHECK_EQ(m.joints.size(), static_cast<size_t>(3));
    CHECK_EQ(m.movableJoints().size(), static_cast<size_t>(2));
    const Joint* lw = m.joint("left_wheel_joint");
    CHECK(lw != nullptr);
    if (lw) {
      CHECK(lw->type == JointType::Continuous);
      CHECK_EQ(lw->parent, "base_link");
      CHECK_NEAR(lw->xyz.y, 0.12, 1e-12);
      CHECK_NEAR(m.jointAxisInRoot(*lw).y, 1.0, 1e-12);
      CHECK_NEAR(m.jointPosition(*lw).y, 0.12, 1e-12);
    }
    CHECK_NEAR(m.poseOf("caster_link").t.z, -0.025, 1e-12);
    CHECK_NEAR(m.poseOf("caster_link").t.x, -0.15, 1e-12);
    const Link* wl = m.link("left_wheel_link");
    CHECK(wl && wl->collision.kind == Geometry::Kind::Cylinder);
    if (wl) {
      CHECK_NEAR(wl->shape().a, 0.05, 1e-12);
      CHECK_NEAR(wl->mass, 0.4, 1e-12);
    }
    CHECK(m.parentJoint("base_link") == nullptr);
    CHECK_EQ(m.childJoints("base_link").size(), static_cast<size_t>(3));
  }
  {
    Issues is;
    const UrdfModel m = parse(slurp(data("diffbot_gz.urdf")), is);
    CHECK(m.valid);
    CHECK_EQ(m.sensors.size(), static_cast<size_t>(3));
    CHECK_EQ(m.sensors[0].name, "front_camera");
    CHECK_EQ(m.sensors[0].type, "camera");
    CHECK_EQ(m.sensors[0].reference, "camera_link");
    CHECK_EQ(m.sensors[0].topic, "/front_cam/image");
    CHECK_NEAR(m.sensors[0].updateRate, 15.0, 1e-12);
    CHECK_EQ(m.sensors[1].topic, "");
    CHECK_NEAR(m.sensors[1].updateRate, 200.0, 1e-12);
    CHECK_EQ(m.sensors[2].type, "depth_camera");
  }
  {
    Issues is;
    const UrdfModel m = parse(slurp(data("diffbot_ros2control.urdf")), is);
    CHECK(m.valid);
    CHECK_EQ(m.ros2control.size(), static_cast<size_t>(1));
    CHECK_EQ(m.ros2control[0].hardwarePlugin, "mock_components/GenericSystem");
    CHECK_EQ(m.ros2control[0].joints.size(), static_cast<size_t>(2));
    CHECK_EQ(m.ros2control[0].joints[0].commandInterfaces.size(), static_cast<size_t>(1));
    CHECK_EQ(m.ros2control[0].joints[0].stateInterfaces.size(), static_cast<size_t>(2));
  }
  {
    Issues is;
    parse("this is <not xml", is);
    CHECK(hasIssue(is, Severity::Error, "not well-formed XML"));
  }
  {
    Issues is;
    parse("<sdf version=\"1.9\"/>", is);
    CHECK(hasIssue(is, Severity::Error, "no <robot> element"));
  }
  {
    Issues is;
    const UrdfModel m = parse("<robot name='x' xmlns:xacro='http://www.ros.org/wiki/xacro'><xacro:property name='a' value='1'/><link name='b'/></robot>", is);
    CHECK(m.xacro);
    CHECK(!m.valid);
    CHECK(hasIssue(is, Severity::Error, "still contains xacro"));
    Issues is2;
    const UrdfModel m2 = parse("<robot name='x'><link name='a'><visual><geometry><box size='${w} 1 1'/></geometry></visual></link></robot>", is2);
    CHECK(m2.xacro);
  }
  {
    Issues is;
    parse("<robot name='x'><link name='a'/><link name='a'/></robot>", is);
    CHECK(hasIssue(is, Severity::Error, "more than one link named 'a'"));
  }
  {
    Issues is;
    const UrdfModel m = parse("<robot name='x'><link name='a'/><joint name='j' type='fixed'><parent link='a'/><child link='zz'/></joint></robot>", is);
    CHECK(hasIssue(is, Severity::Error, "not a link of this robot"));
    CHECK(m.valid);  // the bad joint is dropped, the rest is usable
    CHECK(m.joints.empty());
  }
  {
    Issues is;
    const UrdfModel m = parse("<robot name='x'><link name='a'/><link name='b'/></robot>", is);
    CHECK(!m.valid);
    CHECK(hasIssue(is, Severity::Error, "2 root links (a, b)"));
  }
  {
    Issues is;
    const UrdfModel m = parse(
        "<robot name='x'><link name='a'/><link name='b'/>"
        "<joint name='j1' type='fixed'><parent link='a'/><child link='b'/></joint>"
        "<joint name='j2' type='fixed'><parent link='b'/><child link='a'/></joint></robot>",
        is);
    CHECK(!m.valid);
    CHECK(hasIssue(is, Severity::Error, "loop"));
  }
  {
    Issues is;
    parse("<robot name='x'><link name='a'/><link name='b'/><joint name='j' type='revolute'><parent link='a'/><child link='b'/></joint></robot>", is);
    CHECK(hasIssue(is, Severity::Error, "needs a <limit>"));
  }
  {
    Issues is;
    parse("<robot name='x'><link name='a'/><link name='b'/><joint name='j' type='wobbly'><parent link='a'/><child link='b'/></joint></robot>", is);
    CHECK(hasIssue(is, Severity::Error, "unknown type 'wobbly'"));
  }
  {
    Issues is;
    const UrdfModel m = parse(
        "<robot name='x'><link name='a'><visual><geometry><mesh filename='package://my_meshes/a.dae'/></geometry></visual>"
        "<inertial><mass value='1'/></inertial></link>"
        "<link name='b'><visual><geometry><mesh filename='/home/me/b.stl'/></geometry></visual><inertial><mass value='1'/></inertial></link>"
        "<link name='c'><visual><geometry><mesh filename='c.stl'/></geometry></visual><inertial><mass value='1'/></inertial></link>"
        "<joint name='j' type='fixed'><parent link='a'/><child link='b'/></joint>"
        "<joint name='k' type='fixed'><parent link='a'/><child link='c'/></joint></robot>",
        is);
    CHECK(m.valid);
    CHECK_EQ(m.meshPackages.size(), static_cast<size_t>(1));
    CHECK(m.meshPackages.count("my_meshes") == 1);
    CHECK(hasIssue(is, Severity::Warning, "fixed path"));
    CHECK(hasIssue(is, Severity::Warning, "without a package:// or model:// prefix"));
  }
  {
    Issues is;
    parse("<robot name='x'><link name='a'><visual><geometry><box size='1 1 1'/></geometry></visual></link></robot>", is);
    CHECK(hasIssue(is, Severity::Warning, "no <inertial>"));
  }
  {
    // Extra Gazebo plugins are read.
    Issues is;
    const UrdfModel m = parse(
        "<robot name='x'><link name='a'/><gazebo><plugin filename='gz-sim-diff-drive-system' name='gz::sim::systems::DiffDrive'/></gazebo></robot>", is);
    CHECK(m.hasGazeboPlugin("diff-drive"));
    CHECK(!m.hasGazeboPlugin("ackermann"));
    CHECK_EQ(m.plugins.size(), static_cast<size_t>(1));
  }
}

// ------------------------------------------------------------------------------------ YAML

void testYaml() {
  {
    Issues is;
    const ParsedYaml y = parseYaml("controllers.yaml", slurp(data("controllers.yaml")), is);
    CHECK(y.ok);
    CHECK(y.is(YamlKind::Controllers));
    CHECK(!y.is(YamlKind::Overrides));
    CHECK(!y.is(YamlKind::BridgeConfig));
    CHECK_EQ(y.kindText(), "ros2_control controllers");
    CHECK_NEAR(y.controllers.updateRate, 100.0, 1e-12);
    CHECK_EQ(y.controllers.controllers.size(), static_cast<size_t>(2));
    CHECK(y.controllers.isBroadcaster(y.controllers.controllers[0]));
    CHECK(!y.controllers.isBroadcaster(y.controllers.controllers[1]));
    const ControllerSpec& dd = y.controllers.controllers[1];
    CHECK_EQ(dd.name, "diff_drive_controller");
    CHECK_EQ(dd.type, "diff_drive_controller/DiffDriveController");
    CHECK(dd.hasParameters);
    CHECK_EQ(dd.jointRefs.size(), static_cast<size_t>(2));
    CHECK_EQ(dd.jointRefs[0].first, "left_wheel_names");
    CHECK_EQ(dd.jointRefs[0].second, "left_wheel_joint");
    CHECK_EQ(dd.commandInterfaces.size(), static_cast<size_t>(1));
    CHECK_EQ(dd.stateInterfaces.size(), static_cast<size_t>(2));
    CHECK(is.empty());
  }
  {
    Issues is;
    const ParsedYaml y = parseYaml("bridge_existing.yaml", slurp(data("bridge_existing.yaml")), is);
    CHECK(y.is(YamlKind::BridgeConfig));
    CHECK_EQ(y.kindText(), "ros_gz_bridge config");
    // Two entries are usable; the four broken ones are the ones ros_gz_bridge would skip.
    CHECK_EQ(y.bridge.size(), static_cast<size_t>(2));
    CHECK_EQ(y.bridge[0].rosTopic, "/cmd_vel");
    CHECK_EQ(y.bridge[0].gzTopic, "/my_own/cmd_vel");
    CHECK(y.bridge[0].direction == Direction::RosToGz);
    CHECK_EQ(y.bridge[1].rosTopic, "/battery");
    CHECK_EQ(y.bridge[1].gzTopic, "/battery");
    CHECK_EQ(countIssues(is, Severity::Error), 4);
    CHECK(hasIssue(is, Severity::Warning, "Unknown key 'lazzy' in bridge entry '/battery' is ignored; did you mean 'lazy'?"));
    CHECK(hasIssue(is, Severity::Error, "'/bad_direction': direction 'SIDEWAYS'"));
    CHECK(hasIssue(is, Severity::Error, "'/no_gz_type': ros_type_name and gz_type_name must both be set"));
    CHECK(hasIssue(is, Severity::Error, "'/mismatch': sensor_msgs/msg/Imu cannot be converted to gz.msgs.Twist; the bridge pairs it with gz.msgs.IMU"));
    CHECK(hasIssue(is, Severity::Error, "'/both': topic_name and ros_topic_name are mutually exclusive"));
  }
  {
    Issues is;
    const ParsedYaml y = parseYaml("overrides.yaml", slurp(data("overrides.yaml")), is);
    CHECK(y.is(YamlKind::Overrides));
    CHECK_EQ(y.kindText(), "overrides");
    CHECK(is.empty());
    const Overrides& o = y.overrides;
    CHECK(o.packageName && *o.packageName == "tiny_sim");
    CHECK(o.modelName && *o.modelName == "tiny");
    CHECK(o.worldName && *o.worldName == "lab");
    CHECK(o.stepSize && std::fabs(*o.stepSize - 0.002) < 1e-12);
    CHECK(o.gravity && std::fabs(o.gravity->z + 9.81) < 1e-12);
    CHECK(o.spherical && std::fabs(o.spherical->latitude - 48.8566) < 1e-12 && std::fabs(o.spherical->heading - 90) < 1e-12);
    CHECK(o.spawnX && o.spawnY && o.spawnZ && std::fabs(*o.spawnX - 1) < 1e-12 && std::fabs(*o.spawnY + 2) < 1e-12 && std::fabs(*o.spawnZ - 0.3) < 1e-12);
    CHECK(o.spawnYaw && std::fabs(*o.spawnYaw - 1.57) < 1e-12);
    CHECK_EQ(o.sensors.size(), static_cast<size_t>(3));
    CHECK(o.sensors.at("laser").samples && *o.sensors.at("laser").samples == 720);
    CHECK(o.sensors.at("laser").rangeMax && std::fabs(*o.sensors.at("laser").rangeMax - 20) < 1e-12);
    CHECK(o.sensors.at("body_imu").enabled && !*o.sensors.at("body_imu").enabled);
    CHECK(o.drive.rosCmdVelTopic && *o.drive.rosCmdVelTopic == "/robot/cmd_vel");
    CHECK(o.drive.maxAngularVelocity && std::fabs(*o.drive.maxAngularVelocity - 2.5) < 1e-12);
    CHECK(o.jointStates && *o.jointStates);
    CHECK_EQ(o.exclude.size(), static_cast<size_t>(1));
  }
  {
    // The template shown to users is itself valid and says nothing wrong.
    Issues is;
    const ParsedYaml y = parseYaml("template.yaml", overridesTemplate(), is);
    CHECK(y.is(YamlKind::Overrides));
    CHECK(is.empty());
    if (!is.empty()) std::cerr << issuesText(is);
  }
  {
    // One file can be several kinds.
    Issues is;
    const ParsedYaml y = parseYaml("both.yaml", slurp(data("controllers.yaml")) + "package_name: both_sim\n", is);
    CHECK(y.is(YamlKind::Controllers));
    CHECK(y.is(YamlKind::Overrides));
    CHECK_EQ(y.kindText(), "ros2_control controllers + overrides");
    CHECK(y.overrides.packageName && *y.overrides.packageName == "both_sim");
  }
  {
    Issues is;
    const ParsedYaml y = parseYaml("bad.yaml", "a: [b\nc: d\n", is);
    CHECK(!y.ok);
    CHECK(hasIssue(is, Severity::Error, "not valid YAML"));
    CHECK(hasIssue(is, Severity::Error, "line"));
  }
  {
    Issues is;
    const ParsedYaml y = parseYaml("empty.yaml", "# nothing\n", is);
    CHECK(y.ok);
    CHECK(y.kinds.empty());
    CHECK(hasIssue(is, Severity::Warning, "empty"));
  }
  {
    Issues is;
    parseYaml("scalar.yaml", "just text\n", is);
    CHECK(hasIssue(is, Severity::Error, "must be a map"));
  }
  {
    Issues is;
    parseYaml("list.yaml", "- 1\n- 2\n", is);
    CHECK(hasIssue(is, Severity::Error, "none of its entries has bridge keys"));
  }
  {
    Issues is;
    const ParsedYaml y = parseYaml("typo.yaml", "packge_name: x\nmodel_name: ok\n", is);
    CHECK(y.is(YamlKind::Overrides));
    CHECK(hasIssue(is, Severity::Warning, "Unknown key 'packge_name' is ignored; did you mean 'package_name'?"));
  }
  {
    Issues is;
    const ParsedYaml y = parseYaml("what.yaml", "foo: 1\nbar: 2\n", is);
    CHECK(y.kinds.empty());
    CHECK(hasIssue(is, Severity::Warning, "Could not tell what this file is"));
  }
  {
    Issues is;
    parseYaml("types.yaml",
              "spawn: {x: abc}\nworld: {gravity: [1, 2], step_size: -1}\nsensors: {a: {width: 0}, b: {range_min: 5, range_max: 1}}\n"
              "drive: {wheel_radius: -1}\njoints: {position_control: 5}\n",
              is);
    CHECK(hasIssue(is, Severity::Error, "'spawn.x' must be a number"));
    CHECK(hasIssue(is, Severity::Error, "world.gravity"));
    CHECK(hasIssue(is, Severity::Error, "world.step_size"));
    CHECK(hasIssue(is, Severity::Error, "width, height and samples must be at least 1"));
    CHECK(hasIssue(is, Severity::Error, "range_min' must be smaller than range_max"));
    CHECK(hasIssue(is, Severity::Error, "greater than 0"));
  }
  {
    Issues is;
    parseYaml("sc.yaml", "world:\n  spherical_coordinates: {latitude: 95, longitude: 0}\n", is);
    CHECK(hasIssue(is, Severity::Error, "latitude must be within -90..90"));
    Issues is2;
    parseYaml("sc2.yaml", "world:\n  spherical_coordinates: {latitude: 5}\n", is2);
    CHECK(hasIssue(is2, Severity::Error, "needs at least latitude and longitude"));
  }
  {
    Issues is;
    parseYaml("cm.yaml", "controller_manager:\n  ros__parameters:\n    update_rate: fast\n    a:\n      type: x/Y\n", is);
    CHECK(hasIssue(is, Severity::Error, "update_rate must be a number"));
    Issues is2;
    parseYaml("cm2.yaml", "controller_manager:\n  other: 1\n", is2);
    CHECK(hasIssue(is2, Severity::Error, "no ros__parameters"));
    Issues is3;
    const ParsedYaml y3 = parseYaml("cm3.yaml", "/**:\n  x: 1\n/ns/controller_manager:\n  ros__parameters:\n    c1:\n      type: a/B\n", is3);
    CHECK(y3.is(YamlKind::Controllers));
    CHECK_EQ(y3.controllers.controllers.size(), static_cast<size_t>(1));
  }
  {
    // A service entry and a qos profile are valid; a wrong qos profile is not.
    Issues is;
    const ParsedYaml y = parseYaml(
        "svc.yaml",
        "- service_name: /world/w/create\n  ros_type_name: ros_gz_interfaces/srv/SpawnEntity\n  gz_req_type_name: gz.msgs.EntityFactory\n  gz_rep_type_name: gz.msgs.Boolean\n"
        "- topic_name: /clock\n  ros_type_name: rosgraph_msgs/msg/Clock\n  gz_type_name: gz.msgs.Clock\n  qos_profile: CLOCK\n  lazy: true\n  subscriber_queue: 5\n"
        "- topic_name: /x\n  ros_type_name: std_msgs/msg/String\n  gz_type_name: gz.msgs.StringMsg\n  qos_profile: FAST\n",
        is);
    CHECK_EQ(y.bridge.size(), static_cast<size_t>(2));
    CHECK(y.bridge[0].isService());
    CHECK_EQ(y.bridge[1].qos, "CLOCK");
    CHECK(y.bridge[1].lazy);
    CHECK_EQ(y.bridge[1].subscriberQueue, 5);
    CHECK(y.bridge[1].direction == Direction::Bidirectional);  // the bridge's own default
    CHECK(hasIssue(is, Severity::Error, "qos_profile 'FAST'"));
  }
}

}  // namespace

void testDetectAndPlan();
void testEmit();
void testProject();
void testVerify();

int main() {
  testCommon();
  testTypes();
  testUrdf();
  testYaml();
  testDetectAndPlan();
  testEmit();
  testProject();
  testVerify();
  std::cout << g_checks << " checks, " << g_failed << " failed\n";
  return g_failed == 0 ? 0 : 1;
}

// The remaining tests are in test_core_flow.inc to keep this file readable.
#include "test_core_flow.inc"
