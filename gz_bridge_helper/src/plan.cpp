// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include "gz_bridge_helper/plan.hpp"

#include <algorithm>
#include <cmath>
#include <map>

#include "gz_bridge_helper/types.hpp"

namespace gzb {

namespace {

const char* kWhere = "Plan";

bool validModelName(const std::string& s) {
  if (s.empty()) return false;
  for (char c : s) {
    if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-')) return false;
  }
  return true;
}

/// "/a" + "b" -> "/a/b"; "/" + "b" -> "/b".
std::string joinTopic(const std::string& parent, const std::string& leaf) { return (parent == "/" ? "" : parent) + "/" + leaf; }

std::string f3(double v) {
  char buf[32];
  std::snprintf(buf, sizeof buf, "%.3f", v);
  return buf;
}

template <class T>
void takeIf(std::optional<T>& dst, const std::optional<T>& src) {
  if (src) dst = src;
}

/// Later files win, key by key.
void mergeOverrides(Overrides& a, const Overrides& b) {
  takeIf(a.packageName, b.packageName);
  takeIf(a.modelName, b.modelName);
  takeIf(a.worldName, b.worldName);
  takeIf(a.useSimTime, b.useSimTime);
  takeIf(a.stepSize, b.stepSize);
  takeIf(a.gravity, b.gravity);
  takeIf(a.spherical, b.spherical);
  takeIf(a.spawnX, b.spawnX);
  takeIf(a.spawnY, b.spawnY);
  takeIf(a.spawnZ, b.spawnZ);
  takeIf(a.spawnRoll, b.spawnRoll);
  takeIf(a.spawnPitch, b.spawnPitch);
  takeIf(a.spawnYaw, b.spawnYaw);
  for (const auto& kv : b.sensors) {
    SensorOverride& s = a.sensors[kv.first];
    takeIf(s.enabled, kv.second.enabled);
    takeIf(s.rosTopic, kv.second.rosTopic);
    takeIf(s.frameId, kv.second.frameId);
    takeIf(s.updateRate, kv.second.updateRate);
    takeIf(s.width, kv.second.width);
    takeIf(s.height, kv.second.height);
    takeIf(s.horizontalFov, kv.second.horizontalFov);
    takeIf(s.samples, kv.second.samples);
    takeIf(s.rangeMin, kv.second.rangeMin);
    takeIf(s.rangeMax, kv.second.rangeMax);
  }
  takeIf(a.drive.enabled, b.drive.enabled);
  takeIf(a.drive.wheelSeparation, b.drive.wheelSeparation);
  takeIf(a.drive.wheelRadius, b.drive.wheelRadius);
  takeIf(a.drive.rosCmdVelTopic, b.drive.rosCmdVelTopic);
  takeIf(a.drive.rosOdomTopic, b.drive.rosOdomTopic);
  takeIf(a.drive.maxLinearVelocity, b.drive.maxLinearVelocity);
  takeIf(a.drive.maxAngularVelocity, b.drive.maxAngularVelocity);
  takeIf(a.jointStates, b.jointStates);
  takeIf(a.positionControl, b.positionControl);
  takeIf(a.pGain, b.pGain);
  takeIf(a.iGain, b.iGain);
  takeIf(a.dGain, b.dGain);
  a.bridge.insert(a.bridge.end(), b.bridge.begin(), b.bridge.end());
  a.exclude.insert(a.exclude.end(), b.exclude.begin(), b.exclude.end());
}

/// Lowest point of the model's shapes below the root frame, from the exact extent of each box,
/// cylinder and sphere in its place (a mesh has no known extent and is skipped).
double lowestZ(const UrdfModel& m) {
  double low = 0;
  for (const Link& l : m.links) {
    const Geometry& g = l.shape();
    const Transform T = m.poseOf(l.name) * Transform::fromXyzRpy(l.shapeXyz, l.shapeRpy);
    double below = 0;  // how far the shape reaches below its own centre
    switch (g.kind) {
      case Geometry::Kind::Sphere: below = g.a; break;
      case Geometry::Kind::Cylinder: {
        const double az = std::fabs(T.r[2][2]);  // z of the cylinder axis
        below = az * g.b / 2 + g.a * std::sqrt(std::max(0.0, 1 - az * az));
        break;
      }
      case Geometry::Kind::Box:
        below = (std::fabs(T.r[2][0]) * g.a + std::fabs(T.r[2][1]) * g.b + std::fabs(T.r[2][2]) * g.c) / 2;
        break;
      default: continue;
    }
    low = std::min(low, T.t.z - below);
  }
  return low;
}

struct Builder {
  Plan& p;
  Issues& issues;
  std::string group;  // set before a run of add() calls

  void add(const std::string& ros, const std::string& gz, const std::string& rosType, const std::string& gzType, Direction d,
           const std::string& purpose, const std::string& qos = "", const std::string& frame = "") {
    BridgeEntry e;
    e.rosTopic = ros;
    e.gzTopic = gz;
    e.rosType = rosType;
    e.gzType = gzType;
    e.direction = d;
    e.qos = qos;
    e.frameId = frame;
    e.origin = "generated";
    e.purpose = purpose;
    e.group = group;
    validateEntry(e, kWhere, issues);
    p.bridge.push_back(e);
  }
};

}  // namespace

bool validPackageName(const std::string& s) {
  if (s.size() < 2 || !std::islower(static_cast<unsigned char>(s[0]))) return false;
  for (char c : s) {
    if (!(std::islower(static_cast<unsigned char>(c)) || std::isdigit(static_cast<unsigned char>(c)) || c == '_')) return false;
  }
  return s.find("__") == std::string::npos && s.back() != '_';
}

std::string packageNameFor(const std::string& robotName) {
  std::string out;
  for (char c : robotName) {
    if (std::isalnum(static_cast<unsigned char>(c))) out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    else if (!out.empty() && out.back() != '_') out.push_back('_');
  }
  while (!out.empty() && out.back() == '_') out.pop_back();
  if (out.empty() || !std::isalpha(static_cast<unsigned char>(out[0]))) out = "robot_" + out;
  while (!out.empty() && out.back() == '_') out.pop_back();
  return out;
}

bool Plan::hasKind(SensorKind k) const {
  return std::any_of(sensors.begin(), sensors.end(), [k](const PlanSensor& s) { return s.kind == k; });
}

bool Plan::wantsSensorsSystem() const { return hasKind(SensorKind::Camera) || hasKind(SensorKind::Lidar); }

const PlanSensor* Plan::sensor(const std::string& name) const {
  for (const PlanSensor& s : sensors) {
    if (s.name == name) return &s;
  }
  return nullptr;
}

Plan buildPlan(const UrdfModel& m, const Detection& det, const std::set<std::string>& selected,
               const std::vector<ParsedYaml>& yamls, Issues& issues) {
  Plan p;
  p.mode = det.mode;
  if (!m.valid) return p;

  Overrides ov;
  for (const ParsedYaml& y : yamls) {
    if (y.is(YamlKind::Overrides)) mergeOverrides(ov, y.overrides);
  }

  // ----------------------------------------------------------------------------- names
  p.modelName = m.name;
  if (ov.modelName) {
    if (validModelName(*ov.modelName)) p.modelName = *ov.modelName;
    else addIssue(issues, Severity::Error, "YAML overrides", "model_name '" + *ov.modelName + "' may only use letters, digits, '_' and '-'; '" + m.name + "' is used.");
  }
  if (!validModelName(p.modelName)) {
    addIssue(issues, Severity::Warning, kWhere,
             "The robot name '" + p.modelName + "' has characters that are not allowed in Gazebo topic names; it is shortened to letters, digits, '_' and '-'.");
    std::string clean;
    for (char c : p.modelName) clean.push_back(std::isalnum(static_cast<unsigned char>(c)) || c == '-' ? c : '_');
    p.modelName = clean.empty() ? "robot" : clean;
  }
  p.packageName = packageNameFor(m.name) + "_sim";
  if (!validPackageName(p.packageName)) p.packageName = "robot_sim";
  if (ov.packageName) {
    if (validPackageName(*ov.packageName)) p.packageName = *ov.packageName;
    else addIssue(issues, Severity::Error, "YAML overrides",
                  "package_name '" + *ov.packageName + "' is not a valid ROS 2 package name (lowercase letters, digits and '_', starting with a letter); '" + p.packageName + "' is used.");
  }
  p.worldName = "sim_world";
  if (ov.worldName) {
    if (validModelName(*ov.worldName)) p.worldName = *ov.worldName;
    else addIssue(issues, Severity::Error, "YAML overrides", "world.name '" + *ov.worldName + "' may only use letters, digits, '_' and '-'; '" + p.worldName + "' is used.");
  }
  p.useSimTime = ov.useSimTime.value_or(true);
  if (ov.stepSize) p.stepSize = *ov.stepSize;
  if (ov.gravity) p.gravity = *ov.gravity;
  p.spherical = ov.spherical;
  p.baseFrame = m.root;
  p.spawnXyz.x = ov.spawnX.value_or(0);
  p.spawnXyz.y = ov.spawnY.value_or(0);
  if (ov.spawnZ) {
    p.spawnXyz.z = *ov.spawnZ;
  } else {
    const double z = -lowestZ(m) + 0.05;
    p.spawnXyz.z = std::round(z * 1000) / 1000;
    addIssue(issues, Severity::Info, kWhere, "Spawn height " + f3(p.spawnXyz.z) + " m was computed from the lowest shape of the URDF (set spawn.z to change it).");
  }
  p.spawnRoll = ov.spawnRoll.value_or(0);
  p.spawnPitch = ov.spawnPitch.value_or(0);
  p.spawnYaw = ov.spawnYaw.value_or(0);
  p.meshPackages.assign(m.meshPackages.begin(), m.meshPackages.end());

  const std::string mp = "/model/" + p.modelName;
  Builder b{p, issues, ""};

  // /clock is always bridged: every node that uses simulated time needs it.
  b.group = "clock";
  b.add("/clock", "/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock", Direction::GzToRos, "Simulation time, so that use_sim_time works", "CLOCK");

  // ---------------------------------------------------------------------------- sensors
  for (const auto& kv : ov.sensors) {
    if (!det.sensor(kv.first)) {
      std::vector<std::string> names;
      for (const SensorCandidate& s : det.sensors) names.push_back(s.name);
      const std::string hint = closestMatch(kv.first, names);
      addIssue(issues, Severity::Warning, "YAML overrides",
               "sensors." + kv.first + " does not match a sensor found in the URDF" + (hint.empty() ? "." : "; did you mean '" + hint + "'?"));
    }
  }
  for (const SensorCandidate& c : det.sensors) {
    if (!selected.count(c.id)) continue;
    SensorOverride so;
    auto it = ov.sensors.find(c.name);
    if (it != ov.sensors.end()) so = it->second;
    if (so.enabled && !*so.enabled) continue;

    PlanSensor s;
    s.name = c.name;
    s.kind = c.kind;
    s.link = c.link;
    s.frameLink = so.frameId.value_or(c.frameLink);
    s.existing = c.existing;
    s.addToUrdf = !c.existing;
    const std::string base = mp + "/sensor/" + c.name;
    std::string suffix;
    switch (c.kind) {
      case SensorKind::Camera: suffix = "image"; s.updateRate = 30; break;
      case SensorKind::Imu: suffix = "imu"; s.updateRate = 100; break;
      case SensorKind::Lidar: suffix = "scan"; s.updateRate = 10; break;
      case SensorKind::Gps: suffix = "navsat"; s.updateRate = 1; break;
      case SensorKind::Magnetometer: suffix = "magnetometer"; s.updateRate = 20; break;
      case SensorKind::AirPressure: suffix = "air_pressure"; s.updateRate = 20; break;
      case SensorKind::Altimeter: suffix = "altimeter"; s.updateRate = 20; break;
      default: continue;
    }
    if (c.existing) {
      if (c.needsTopic) {
        if (selected.count("sensortopic:" + c.name)) {
          s.addTopicToUrdf = true;
          s.gzTopic = base + "/" + suffix;
        } else {
          addIssue(issues, Severity::Warning, kWhere,
                   "The sensor '" + c.name + "' has no <topic> and adding one was not ticked, so it is not bridged (its Gazebo topic cannot be known).");
          continue;
        }
      } else {
        s.gzTopic = c.gzTopic;
      }
      if (so.updateRate || so.width || so.height || so.horizontalFov || so.samples || so.rangeMin || so.rangeMax) {
        addIssue(issues, Severity::Warning, "YAML overrides",
                 "The sensor '" + c.name + "' is already written in the URDF, so its update_rate/width/height/horizontal_fov/samples/range settings are not applied; only ros_topic and frame_id are.");
      }
    } else {
      s.gzTopic = base + "/" + suffix;
      if (so.updateRate) s.updateRate = *so.updateRate;
      if (so.width) s.width = *so.width;
      if (so.height) s.height = *so.height;
      if (so.horizontalFov) s.horizontalFov = *so.horizontalFov;
      if (so.samples) s.samples = *so.samples;
      if (so.rangeMin) s.rangeMin = *so.rangeMin;
      if (so.rangeMax) s.rangeMax = *so.rangeMax;
    }
    if (c.kind == SensorKind::Camera) s.gzSecondTopic = joinTopic(topicParent(s.gzTopic), "camera_info");
    if (c.kind == SensorKind::Lidar) s.gzSecondTopic = joinTopic(s.gzTopic, "points");

    const std::string n = "/" + c.name;
    switch (c.kind) {
      case SensorKind::Camera: s.rosTopic = n + "/image_raw"; break;
      case SensorKind::Imu: s.rosTopic = n + "/data"; break;
      case SensorKind::Lidar: s.rosTopic = n + "/scan"; break;
      case SensorKind::Gps: s.rosTopic = n + "/fix"; break;
      case SensorKind::Magnetometer: s.rosTopic = n + "/magnetic_field"; break;
      case SensorKind::AirPressure: s.rosTopic = n + "/air_pressure"; break;
      case SensorKind::Altimeter: s.rosTopic = n + "/altimeter"; break;
      default: break;
    }
    if (so.rosTopic) s.rosTopic = absoluteTopic(*so.rosTopic);
    if (c.kind == SensorKind::Camera) s.rosSecondTopic = joinTopic(topicParent(s.rosTopic), "camera_info");
    if (c.kind == SensorKind::Lidar) s.rosSecondTopic = joinTopic(topicParent(s.rosTopic), "points");
    p.sensors.push_back(s);
  }

  b.group = "sensor";
  for (const PlanSensor& s : p.sensors) {
    const std::string who = "'" + s.name + "'";
    switch (s.kind) {
      case SensorKind::Camera:
        b.add(s.rosTopic, s.gzTopic, "sensor_msgs/msg/Image", "gz.msgs.Image", Direction::GzToRos, "Camera " + who + ": image", "", s.frameLink);
        b.add(s.rosSecondTopic, s.gzSecondTopic, "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", Direction::GzToRos, "Camera " + who + ": intrinsics", "", s.frameLink);
        break;
      case SensorKind::Imu:
        b.add(s.rosTopic, s.gzTopic, "sensor_msgs/msg/Imu", "gz.msgs.IMU", Direction::GzToRos, "IMU " + who, "", s.frameLink);
        break;
      case SensorKind::Lidar:
        b.add(s.rosTopic, s.gzTopic, "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan", Direction::GzToRos, "Lidar " + who + ": scan", "", s.frameLink);
        b.add(s.rosSecondTopic, s.gzSecondTopic, "sensor_msgs/msg/PointCloud2", "gz.msgs.PointCloudPacked", Direction::GzToRos, "Lidar " + who + ": point cloud", "", s.frameLink);
        break;
      case SensorKind::Gps:
        b.add(s.rosTopic, s.gzTopic, "sensor_msgs/msg/NavSatFix", "gz.msgs.NavSat", Direction::GzToRos, "GPS " + who, "", s.frameLink);
        break;
      case SensorKind::Magnetometer:
        b.add(s.rosTopic, s.gzTopic, "sensor_msgs/msg/MagneticField", "gz.msgs.Magnetometer", Direction::GzToRos, "Magnetometer " + who, "", s.frameLink);
        break;
      case SensorKind::AirPressure:
        b.add(s.rosTopic, s.gzTopic, "sensor_msgs/msg/FluidPressure", "gz.msgs.FluidPressure", Direction::GzToRos, "Air pressure " + who, "", s.frameLink);
        break;
      case SensorKind::Altimeter:
        b.add(s.rosTopic, s.gzTopic, "ros_gz_interfaces/msg/Altimeter", "gz.msgs.Altimeter", Direction::GzToRos, "Altimeter " + who, "", s.frameLink);
        break;
      default: break;
    }
  }
  if (p.hasKind(SensorKind::Gps) && !p.spherical) {
    p.spherical = Overrides::Spherical{};
    addIssue(issues, Severity::Info, kWhere,
             "A GPS needs to know where the world is on Earth. The world is placed at latitude 0, longitude 0; set world.spherical_coordinates in an overrides file to change it.");
  }

  // ------------------------------------------------------------ Gazebo systems (not ros2_control)
  if (p.mode == ControlMode::GzSystems) {
    b.group = "drive";
    for (const DriveCandidate& dc : det.drives) {
      if (!selected.count(dc.id)) continue;
      if (ov.drive.enabled && !*ov.drive.enabled) continue;
      PlanDrive pd;
      pd.d = dc;
      if (ov.drive.wheelSeparation) pd.d.wheelSeparation = *ov.drive.wheelSeparation;
      if (ov.drive.wheelRadius) pd.d.wheelRadius = *ov.drive.wheelRadius;
      pd.gzCmdVel = mp + "/cmd_vel";
      pd.gzOdom = mp + "/odometry";
      pd.gzTf = mp + "/tf";
      if (ov.drive.rosCmdVelTopic) pd.rosCmdVel = absoluteTopic(*ov.drive.rosCmdVelTopic);
      if (ov.drive.rosOdomTopic) pd.rosOdom = absoluteTopic(*ov.drive.rosOdomTopic);
      pd.baseFrame = m.root;
      pd.maxLinear = ov.drive.maxLinearVelocity.value_or(0);
      pd.maxAngular = ov.drive.maxAngularVelocity.value_or(0);
      if (pd.d.wheelRadius <= 0) {
        addIssue(issues, Severity::Error, kWhere,
                 "The wheel radius could not be measured (the wheels have no cylinder or sphere shape). Give drive.wheel_radius in an overrides file; the drive is not added.");
        continue;
      }
      if (pd.d.wheelSeparation <= 0) {
        addIssue(issues, Severity::Error, kWhere, "The wheel separation is 0; give drive.wheel_separation in an overrides file. The drive is not added.");
        continue;
      }
      p.drives.push_back(pd);
      const PlanDrive& d = p.drives.back();
      b.add(d.rosCmdVel, d.gzCmdVel, "geometry_msgs/msg/Twist", "gz.msgs.Twist", Direction::RosToGz, "Velocity command for the " + std::string(driveKindLabel(dc.kind)));
      b.add(d.rosOdom, d.gzOdom, "nav_msgs/msg/Odometry", "gz.msgs.Odometry", Direction::GzToRos, "Odometry of the " + std::string(driveKindLabel(dc.kind)), "", "");
      b.add("/tf", d.gzTf, "tf2_msgs/msg/TFMessage", "gz.msgs.Pose_V", Direction::GzToRos, "Transform " + d.odomFrame + " -> " + d.baseFrame + " from the drive");
    }
    if (selected.count("odometry")) {
      b.group = "odometry";
      p.odometry = true;
      p.gzOdomTopic = mp + "/odometry";
      p.gzOdomTfTopic = mp + "/tf";
      b.add(p.rosOdomTopic, p.gzOdomTopic, "nav_msgs/msg/Odometry", "gz.msgs.Odometry", Direction::GzToRos, "Odometry from the model's pose");
      b.add("/tf", p.gzOdomTfTopic, "tf2_msgs/msg/TFMessage", "gz.msgs.Pose_V", Direction::GzToRos, "Transform odom -> " + m.root + " from the model's pose");
    }
    if (selected.count("joint_states")) {
      b.group = "joint";
      p.jointStates = true;
      p.gzJointStateTopic = mp + "/joint_state";
      b.add(p.rosJointStateTopic, p.gzJointStateTopic, "sensor_msgs/msg/JointState", "gz.msgs.Model", Direction::GzToRos, "Position of every movable joint");
    }
    p.pGain = ov.pGain.value_or(10);
    p.iGain = ov.iGain.value_or(0);
    p.dGain = ov.dGain.value_or(0.5);
    for (const JointControlCandidate& jc : det.jointControls) {
      if (!selected.count(jc.id)) continue;
      PlanJointControl c;
      c.joint = jc.joint;
      c.lower = jc.lower;
      c.upper = jc.upper;
      c.gzTopic = mp + "/joint/" + jc.joint + "/cmd_pos";
      c.rosTopic = "/joint/" + jc.joint + "/cmd_pos";
      p.jointControls.push_back(c);
      b.add(c.rosTopic, c.gzTopic, "std_msgs/msg/Float64", "gz.msgs.Double", Direction::RosToGz, "Target angle of " + jc.joint);
    }
  }

  // ------------------------------------------------------------------------ ros2_control
  if (p.mode == ControlMode::Ros2Control) {
    p.addControlPlugin = selected.count("ros2control:plugin") > 0;
    p.patchHardware = selected.count("ros2control:hardware") > 0;
    std::vector<ControllerSpec> ordered;
    for (const ControllerSpec& c : det.ros2.controllers) {
      if (c.type.find("Broadcaster") != std::string::npos) ordered.push_back(c);
    }
    for (const ControllerSpec& c : det.ros2.controllers) {
      if (c.type.find("Broadcaster") == std::string::npos) ordered.push_back(c);
    }
    p.controllers = ordered;
    std::set<std::string> taken;
    for (const ParsedYaml& y : yamls) {
      if (!y.is(YamlKind::Controllers)) continue;
      Plan::ConfigFile f;
      f.sourcePath = y.path;
      f.text = y.text;
      f.name = baseName(y.path);
      for (int n = 2; taken.count(f.name); ++n) f.name = stem(y.path) + "_" + std::to_string(n) + ".yaml";
      taken.insert(f.name);
      p.controllerFiles.push_back(f);
      if (y.kinds.size() > 1) {
        addIssue(issues, Severity::Warning, "YAML " + baseName(y.path),
                 "This file holds controllers and also other settings (" + y.kindText() +
                     "). The whole file is passed to ros2_control as a parameter file, which expects only node sections; keep overrides in a separate file.");
      }
    }
    if (p.controllerFiles.empty() && !p.controllers.empty()) {
      addIssue(issues, Severity::Error, kWhere, "Controllers are listed but there is no controllers file to load them from.");
    }
    // The controllers against the URDF.
    for (const ControllerSpec& c : p.controllers) {
      for (const auto& ref : c.jointRefs) {
        if (!m.joint(ref.second)) {
          const std::string hint = [&] {
            std::vector<std::string> names;
            for (const Joint& j : m.joints) names.push_back(j.name);
            return closestMatch(ref.second, names);
          }();
          addIssue(issues, Severity::Error, "Controllers",
                   "Controller '" + c.name + "': " + ref.first + " names the joint '" + ref.second + "', which is not in the URDF" +
                       (hint.empty() ? "." : "; did you mean '" + hint + "'?"));
          continue;
        }
        const Joint* j = m.joint(ref.second);
        if (!j->movable()) {
          addIssue(issues, Severity::Error, "Controllers", "Controller '" + c.name + "': '" + ref.second + "' is a fixed joint and cannot be controlled.");
          continue;
        }
        const Ros2ControlJoint* decl = nullptr;
        for (const Ros2Control& rc : m.ros2control) {
          for (const Ros2ControlJoint& rj : rc.joints) {
            if (rj.name == ref.second) decl = &rj;
          }
        }
        if (!decl) {
          if (!m.ros2control.empty()) {
            addIssue(issues, Severity::Error, "Controllers",
                     "Controller '" + c.name + "' uses the joint '" + ref.second + "', but the URDF's <ros2_control> does not declare it, so the hardware offers no interface for it.");
          }
          continue;
        }
        for (const std::string& ci : c.commandInterfaces) {
          if (std::find(decl->commandInterfaces.begin(), decl->commandInterfaces.end(), ci) == decl->commandInterfaces.end()) {
            addIssue(issues, Severity::Error, "Controllers",
                     "Controller '" + c.name + "' wants the command interface '" + ci + "' on '" + ref.second + "', but <ros2_control> declares only: " +
                         [&] {
                           std::string s;
                           for (const std::string& d : decl->commandInterfaces) s += (s.empty() ? "" : ", ") + d;
                           return s.empty() ? std::string("(none)") : s;
                         }() + ".");
          }
        }
        for (const std::string& si : c.stateInterfaces) {
          if (std::find(decl->stateInterfaces.begin(), decl->stateInterfaces.end(), si) == decl->stateInterfaces.end()) {
            addIssue(issues, Severity::Warning, "Controllers",
                     "Controller '" + c.name + "' reads the state interface '" + si + "' of '" + ref.second + "', which <ros2_control> does not declare.");
          }
        }
      }
    }
    for (const Ros2Control& rc : m.ros2control) {
      for (const Ros2ControlJoint& rj : rc.joints) {
        if (!m.joint(rj.name)) addIssue(issues, Severity::Error, "URDF", "<ros2_control> declares the joint '" + rj.name + "', which is not a joint of the URDF.");
      }
    }
    for (const ParsedYaml& y : yamls) {
      if (y.is(YamlKind::Controllers) && y.controllers.updateRate > 0 && y.controllers.updateRate > 1.0 / p.stepSize + 1e-9) {
        addIssue(issues, Severity::Warning, "YAML " + baseName(y.path),
                 "The controller manager update_rate (" + f3(y.controllers.updateRate) + " Hz) is faster than the physics step (" + f3(1.0 / p.stepSize) +
                     " Hz): controllers would update more often than the world changes. Lower it or set world.step_size smaller.");
      }
    }
  }

  // ------------------------------------------------------------------ user bridge entries
  std::vector<BridgeEntry> user;
  for (const ParsedYaml& y : yamls) {
    if (y.is(YamlKind::BridgeConfig)) user.insert(user.end(), y.bridge.begin(), y.bridge.end());
    if (y.is(YamlKind::Overrides)) user.insert(user.end(), y.overrides.bridge.begin(), y.overrides.bridge.end());
  }
  // exclude: switches off generated entries by their ROS topic
  for (const std::string& ex : ov.exclude) {
    bool hit = false;
    for (BridgeEntry& e : p.bridge) {
      if (e.rosTopic == absoluteTopic(ex)) {
        e.enabled = false;
        e.purpose += " (excluded by your overrides)";
        hit = true;
      }
    }
    if (!hit) addIssue(issues, Severity::Warning, "YAML overrides", "exclude lists '" + ex + "', but no generated bridge entry has that ROS topic.");
  }
  for (const BridgeEntry& u : user) {
    if (!u.isService()) {
      for (BridgeEntry& g : p.bridge) {
        if (g.enabled && !g.isService() && g.rosTopic == u.rosTopic && g.origin == "generated") {
          g.enabled = false;
          g.purpose += " (replaced by " + u.origin + ")";
          addIssue(issues, Severity::Info, kWhere, "Your bridge entry for " + u.rosTopic + " (" + u.origin + ") replaces the generated one.");
          if (g.gzTopic != u.gzTopic) {
            addIssue(issues, Severity::Warning, kWhere,
                     "Your entry for " + u.rosTopic + " uses the Gazebo topic " + u.gzTopic + ", but the plugin or sensor that this helper adds to the URDF uses " + g.gzTopic +
                         ". They only meet if you change that topic in the URDF (or untick the item that adds it).");
          }
        }
      }
    }
    BridgeEntry e = u;
    if (e.purpose.empty()) e.purpose = "From " + u.origin;
    e.group = "user";
    p.bridge.push_back(e);
  }

  // Two bridges that publish the same ROS topic from different Gazebo topics are a mistake.
  for (size_t i = 0; i < p.bridge.size(); ++i) {
    const BridgeEntry& a = p.bridge[i];
    if (!a.enabled || a.isService() || a.direction == Direction::RosToGz) continue;
    for (size_t j = i + 1; j < p.bridge.size(); ++j) {
      const BridgeEntry& c = p.bridge[j];
      if (!c.enabled || c.isService() || c.direction == Direction::RosToGz || c.rosTopic != a.rosTopic) continue;
      if (a.rosTopic == "/tf" && a.rosType == c.rosType) continue;  // many publishers on /tf are normal
      if (a.gzTopic == c.gzTopic && a.rosType == c.rosType && a.gzType == c.gzType) {
        addIssue(issues, Severity::Warning, kWhere, "The bridge entry for " + a.rosTopic + " appears twice (" + a.origin + " and " + c.origin + ").");
      } else {
        addIssue(issues, Severity::Error, kWhere,
                 "Two bridges publish " + a.rosTopic + " from different sources (" + a.gzTopic + " and " + c.gzTopic + "); the messages would be mixed. Rename one with sensors.<name>.ros_topic or exclude it.");
      }
    }
  }
  return p;
}

}  // namespace gzb
