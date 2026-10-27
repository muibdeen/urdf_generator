// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include "urdf_generator/sensors.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <set>

namespace urdfgen {
namespace {

double toRad(double deg) { return deg * kPi / 180.0; }

double clean(double v) { return std::fabs(v) < 1e-12 ? 0.0 : v; }

struct Marker {
  Geometry geometry;
  double mass;
};

Marker markerOf(SensorType t) {
  switch (t) {
    case SensorType::Camera: return {Geometry::box(0.03, 0.06, 0.03), 0.05};
    case SensorType::Imu: return {Geometry::box(0.03, 0.03, 0.01), 0.02};
    case SensorType::Lidar: return {Geometry::cylinder(0.035, 0.045), 0.2};
    case SensorType::Gps: return {Geometry::cylinder(0.03, 0.015), 0.05};
    case SensorType::Odometry: return {Geometry::cylinder(0.02, 0.005), 0.01};
  }
  return {Geometry::box(0.02, 0.02, 0.02), 0.01};
}

bool validSensorName(const std::string& n) {
  if (n.empty()) return false;
  for (char c : n) {
    if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_')) return false;
  }
  return true;
}

}  // namespace

std::vector<SensorType> allSensorTypes() {
  return {SensorType::Camera, SensorType::Imu, SensorType::Lidar, SensorType::Gps, SensorType::Odometry};
}

const char* sensorId(SensorType t) {
  switch (t) {
    case SensorType::Camera: return "camera";
    case SensorType::Imu: return "imu";
    case SensorType::Lidar: return "lidar";
    case SensorType::Gps: return "gps";
    case SensorType::Odometry: return "odometry";
  }
  return "sensor";
}

const char* sensorLabel(SensorType t) {
  switch (t) {
    case SensorType::Camera: return "Camera";
    case SensorType::Imu: return "IMU";
    case SensorType::Lidar: return "LiDAR";
    case SensorType::Gps: return "GPS";
    case SensorType::Odometry: return "Odometry";
  }
  return "Sensor";
}

std::array<float, 3> sensorColor(SensorType t) {
  switch (t) {
    case SensorType::Camera: return {{0.15f, 0.40f, 0.85f}};
    case SensorType::Imu: return {{0.20f, 0.65f, 0.30f}};
    case SensorType::Lidar: return {{0.85f, 0.22f, 0.22f}};
    case SensorType::Gps: return {{0.95f, 0.78f, 0.15f}};
    case SensorType::Odometry: return {{0.62f, 0.35f, 0.82f}};
  }
  return {{0.5f, 0.5f, 0.5f}};
}

double sensorHeight(SensorType t) {
  const Geometry g = markerOf(t).geometry;
  return g.kind == Geometry::Kind::Box ? g.c.value() : g.b.value();
}

// -------------------------------------------------------------------- link axis

LinkAxis linkAxis(const Link& link) {
  LinkAxis out;
  if (!link.hasBody) return out;

  const Transform T = Transform::fromOrigin(link.origin);
  Vec3 local{0, 0, 1};  // the body's own axis
  double half = 0;
  const Geometry& g = link.geometry;
  switch (g.kind) {
    case Geometry::Kind::Cylinder:
      local = {0, 0, 1};
      half = g.b.value() / 2;
      break;
    case Geometry::Kind::Sphere:
      local = {0, 0, 1};
      half = g.a.value();
      break;
    case Geometry::Kind::Box: {
      const double d[3] = {g.a.value(), g.b.value(), g.c.value()};
      int axis = 0;
      if (d[1] > d[axis] + 1e-12) axis = 1;
      if (d[2] > d[axis] + 1e-12) axis = 2;
      local = {axis == 0 ? 1.0 : 0.0, axis == 1 ? 1.0 : 0.0, axis == 2 ? 1.0 : 0.0};
      half = d[axis] / 2;
      break;
    }
  }

  Vec3 a = T.rotate(local);
  double centre = dot(T.t, a);  // where the middle of the body is, along the axis
  if (centre < -1e-9) {
    a = a * -1.0;
    centre = -centre;
  } else if (centre < 1e-9) {
    // A body centred on the root: pick the sign so that the axis has a positive main component.
    const double ax = std::fabs(a.x), ay = std::fabs(a.y), az = std::fabs(a.z);
    const double lead = (ax >= ay && ax >= az) ? a.x : (ay >= az ? a.y : a.z);
    if (lead < 0) a = a * -1.0;
    centre = 0;
  }
  out.direction = {clean(a.x), clean(a.y), clean(a.z)};
  out.lower = std::min(0.0, clean(centre - half));
  out.upper = std::max(0.0, clean(centre + half));
  return out;
}

double linkTop(const Link& link) {
  if (!link.hasBody) return 0.0;
  const Mesh m = tessellate(link.geometry);
  const Transform T = Transform::fromOrigin(link.origin);
  double top = -1e300;
  for (const Vec3& v : m.vertices) top = std::max(top, T.apply(v).z);
  return clean(top);
}

std::vector<std::string> attachableLinks(const Robot& robot) {
  std::vector<std::string> out;
  for (const Link& l : robot.flatten().links) {
    if (l.attachable) out.push_back(l.name);
  }
  return out;
}

SensorConfig defaultSensor(const Robot& robot, SensorType type, const std::vector<SensorConfig>& existing) {
  SensorConfig s;
  s.type = type;

  std::set<std::string> taken;
  for (const SensorConfig& e : existing) taken.insert(e.name);
  const std::string base = sensorId(type);
  s.name = base;
  for (int n = 2; taken.count(s.name); ++n) s.name = base + "_" + std::to_string(n);

  const Part part = robot.flatten();
  const Link* host = nullptr;
  for (const Link& l : part.links) {
    if (l.name == "base_link") host = &l;
  }
  if (!host) {
    for (const Link& l : part.links) {
      if (l.hasBody && l.attachable) {
        host = &l;
        break;
      }
    }
  }
  if (!host && !part.links.empty()) host = &part.links.front();
  if (host) {
    s.link = host->name;
    if (host->hasBody) s.offsetZ = linkTop(*host) + sensorHeight(type) / 2;  // sits on top of the body
  }
  return s;
}

// ---------------------------------------------------------------------- adding

void addSensors(Robot& r, const std::vector<SensorConfig>& sensors) {
  if (sensors.empty()) return;
  auto error = [&r](const std::string& m) { r.buildIssues.push_back({Issue::Severity::Error, m}); };

  const Part base = r.flatten();
  std::set<std::string> names;  // every link and joint name in use
  for (const Link& l : base.links) names.insert(l.name);
  for (const Joint& j : base.joints) names.insert(j.name);

  std::set<std::string> sensorNames;
  std::set<SensorType> materialsAdded;
  for (const SensorConfig& s : sensors) {
    if (!validSensorName(s.name)) {
      error("Sensor name '" + s.name + "' must be non-empty and use only letters, digits and '_'.");
      continue;
    }
    if (!sensorNames.insert(s.name).second) {
      error("There is more than one sensor named '" + s.name + "'.");
      continue;
    }
    const Link* host = nullptr;
    for (const Link& l : base.links) {
      if (l.name == s.link) host = &l;
    }
    if (!host) {
      error("Sensor '" + s.name + "' is attached to the link '" + s.link + "', which this robot does not have.");
      continue;
    }
    const std::string linkName = s.name + "_link", jointName = s.name + "_joint";
    const std::string opticalLink = s.name + "_optical_frame", opticalJoint = s.name + "_optical_joint";
    bool clash = names.count(linkName) || names.count(jointName);
    if (s.type == SensorType::Camera) clash = clash || names.count(opticalLink) || names.count(opticalJoint);
    if (clash) {
      error("The names for sensor '" + s.name + "' (" + linkName + ", " + jointName + ") are already used by the robot.");
      continue;
    }
    names.insert(linkName);
    names.insert(jointName);

    const LinkAxis axis = linkAxis(*host);
    const std::string prefix = "sensor_" + s.name + "_";
    r.section(std::string(sensorLabel(s.type)) + " '" + s.name + "' on " + s.link);
    const Num distance = r.prop(prefix + "distance", s.distance,
                                "[m] from the root of " + s.link + ", along the link");
    const Num ox = r.prop(prefix + "offset_x", s.offsetX, "[m] extra shift in the frame of " + s.link);
    const Num oy = r.prop(prefix + "offset_y", s.offsetY, "[m]");
    const Num oz = r.prop(prefix + "offset_z", s.offsetZ, "[m]");
    const Num roll = r.prop(prefix + "roll", toRad(s.rollDeg), "[rad] (" + Num::format(s.rollDeg) + " deg)");
    const Num pitch = r.prop(prefix + "pitch", toRad(s.pitchDeg), "[rad] (" + Num::format(s.pitchDeg) + " deg)");
    const Num yaw = r.prop(prefix + "yaw", toRad(s.yawDeg), "[rad] (" + Num::format(s.yawDeg) + " deg)");

    const Marker marker = markerOf(s.type);
    Link body;
    body.name = linkName;
    body.hasBody = true;
    body.geometry = marker.geometry;
    body.material = std::string("sensor_") + sensorId(s.type);
    body.mass = Num(marker.mass);
    body.attachable = false;
    if (!materialsAdded.count(s.type)) {
      const auto c = sensorColor(s.type);
      r.addMaterial(body.material, c[0], c[1], c[2]);
      materialsAdded.insert(s.type);
    }
    r.root.links.push_back(body);

    Joint joint;
    joint.name = jointName;
    joint.type = JointType::Fixed;
    joint.parent = s.link;
    joint.child = linkName;
    joint.origin = Origin(Num(axis.direction.x) * distance + ox, Num(axis.direction.y) * distance + oy,
                          Num(axis.direction.z) * distance + oz, roll, pitch, yaw);
    r.root.joints.push_back(joint);

    if (s.type == SensorType::Camera) {
      // The optical frame of image pipelines: z forward, x right, y down.
      Link optical;
      optical.name = opticalLink;
      optical.hasBody = false;
      optical.attachable = false;
      r.root.links.push_back(optical);
      Joint oj;
      oj.name = opticalJoint;
      oj.type = JointType::Fixed;
      oj.parent = linkName;
      oj.child = opticalLink;
      oj.origin = Origin(0, 0, 0, -Num::pi() / 2, 0, -Num::pi() / 2);
      r.root.joints.push_back(oj);
      names.insert(opticalLink);
      names.insert(opticalJoint);
    }

    r.sensorMarks.push_back({s.name, s.type, linkName, s.link});
  }
}

}  // namespace urdfgen
