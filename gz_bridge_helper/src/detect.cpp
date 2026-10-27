// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include "gz_bridge_helper/detect.hpp"

#include <algorithm>
#include <cmath>
#include <map>

#include "gz_bridge_helper/bridge.hpp"

namespace gzb {

const char* sensorKindLabel(SensorKind k) {
  switch (k) {
    case SensorKind::Camera: return "camera";
    case SensorKind::Imu: return "IMU";
    case SensorKind::Lidar: return "lidar";
    case SensorKind::Gps: return "GPS";
    case SensorKind::Magnetometer: return "magnetometer";
    case SensorKind::AirPressure: return "air pressure sensor";
    case SensorKind::Altimeter: return "altimeter";
    case SensorKind::Unsupported: return "sensor";
  }
  return "sensor";
}

const char* sdfSensorType(SensorKind k) {
  switch (k) {
    case SensorKind::Camera: return "camera";
    case SensorKind::Imu: return "imu";
    case SensorKind::Lidar: return "gpu_lidar";
    case SensorKind::Gps: return "navsat";
    case SensorKind::Magnetometer: return "magnetometer";
    case SensorKind::AirPressure: return "air_pressure";
    case SensorKind::Altimeter: return "altimeter";
    case SensorKind::Unsupported: return "";
  }
  return "";
}

SensorKind sensorKindFromSdfType(const std::string& t) {
  if (t == "camera") return SensorKind::Camera;
  if (t == "imu") return SensorKind::Imu;
  if (t == "gpu_lidar" || t == "lidar" || t == "gpu_ray" || t == "ray") return SensorKind::Lidar;
  if (t == "navsat" || t == "gps") return SensorKind::Gps;
  if (t == "magnetometer") return SensorKind::Magnetometer;
  if (t == "air_pressure") return SensorKind::AirPressure;
  if (t == "altimeter") return SensorKind::Altimeter;
  return SensorKind::Unsupported;
}

const char* driveKindLabel(DriveKind k) {
  switch (k) {
    case DriveKind::Diff: return "differential drive";
    case DriveKind::Mecanum: return "mecanum drive";
    case DriveKind::Ackermann: return "Ackermann steering";
  }
  return "drive";
}

std::vector<std::string> DriveCandidate::joints() const {
  std::vector<std::string> out;
  auto add = [&out](const std::string& j) {
    if (!j.empty()) out.push_back(j);
  };
  for (const std::string& j : left) add(j);
  for (const std::string& j : right) add(j);
  add(frontLeft);
  add(frontRight);
  add(backLeft);
  add(backRight);
  add(leftSteering);
  add(rightSteering);
  return out;
}

std::set<std::string> Detection::defaultSelection() const {
  std::set<std::string> out;
  for (const Item& i : items) {
    if (i.selected) out.insert(i.id);
  }
  return out;
}

const Item* Detection::item(const std::string& id) const {
  for (const Item& i : items) {
    if (i.id == id) return &i;
  }
  return nullptr;
}

const SensorCandidate* Detection::sensor(const std::string& name) const {
  for (const SensorCandidate& s : sensors) {
    if (s.name == name) return &s;
  }
  return nullptr;
}

const DriveCandidate* Detection::drive(const std::string& id) const {
  for (const DriveCandidate& d : drives) {
    if (d.id == id) return &d;
  }
  return nullptr;
}

namespace {

const char* kWhere = "Detection";

std::vector<std::string> tokens(const std::string& name) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : name) {
    if (std::isalnum(static_cast<unsigned char>(c))) {
      cur.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    } else if (!cur.empty()) {
      out.push_back(cur);
      cur.clear();
    }
  }
  if (!cur.empty()) out.push_back(cur);
  return out;
}

bool hasToken(const std::string& name, const std::vector<std::string>& any) {
  for (const std::string& t : tokens(name)) {
    if (std::find(any.begin(), any.end(), t) != any.end()) return true;
  }
  return false;
}

std::string sanitize(const std::string& s) {
  std::string out;
  for (char c : s) out.push_back(std::isalnum(static_cast<unsigned char>(c)) || c == '_' ? c : '_');
  if (out.empty() || std::isdigit(static_cast<unsigned char>(out[0]))) out = "s_" + out;
  return out;
}

std::string f2(double v) {
  char buf[32];
  std::snprintf(buf, sizeof buf, "%.3f", v);
  return buf;
}

// ---------------------------------------------------------------------------- sensors

SensorKind kindFromLinkName(const std::string& link) {
  if (hasToken(link, {"optical"})) return SensorKind::Unsupported;
  if (hasToken(link, {"camera", "cam"})) return SensorKind::Camera;
  if (hasToken(link, {"imu"})) return SensorKind::Imu;
  if (hasToken(link, {"lidar", "laser", "scan", "rplidar", "hokuyo"})) return SensorKind::Lidar;
  if (hasToken(link, {"gps", "gnss", "navsat"})) return SensorKind::Gps;
  return SensorKind::Unsupported;
}

std::string frameFor(const UrdfModel& m, SensorKind kind, const std::string& name, const std::string& link) {
  if (kind != SensorKind::Camera) return link;
  const std::string stripped = endsWith(link, "_link") && link.size() > 5 ? link.substr(0, link.size() - 5) : link;
  for (const std::string& c : {name + "_optical_frame", name + "_optical_link", stripped + "_optical_frame", link + "_optical_frame"}) {
    if (m.link(c)) return c;
  }
  return link;
}

void detectSensors(const UrdfModel& m, Detection& d, Issues& issues) {
  std::set<std::string> names, linksWithSensor;
  for (const GazeboSensor& gs : m.sensors) {
    SensorCandidate c;
    c.existing = true;
    c.name = gs.name.empty() ? sanitize(gs.reference + "_sensor") : gs.name;
    c.sdfType = gs.type;
    c.kind = sensorKindFromSdfType(gs.type);
    c.link = gs.reference;
    if (!names.insert(c.name).second) {
      addIssue(issues, Severity::Error, "URDF", "There is more than one <sensor> named '" + c.name + "'; Gazebo needs unique sensor names.");
      continue;
    }
    if (c.link.empty() || !m.link(c.link)) {
      addIssue(issues, Severity::Warning, "URDF", "Sensor '" + c.name + "' is not inside a <gazebo reference=\"link\"> of a known link, so it cannot be placed.");
      continue;
    }
    linksWithSensor.insert(c.link);
    c.id = "sensor:" + c.name;
    c.frameLink = !gs.gzFrameId.empty() ? gs.gzFrameId : frameFor(m, c.kind, c.name, c.link);
    if (c.kind == SensorKind::Unsupported) {
      addIssue(issues, Severity::Info, "URDF",
               "The sensor '" + c.name + "' has the type '" + gs.type + "', which this helper does not bridge. Add its topics to your own ros_gz_bridge YAML if you need them.");
      continue;
    }
    c.needsTopic = gs.topic.empty();
    c.gzTopic = gs.topic.empty() ? "" : absoluteTopic(gs.topic);
    c.reason = "already in the URDF on link " + c.link;
    d.sensors.push_back(c);
  }

  for (const Link& l : m.links) {
    if (linksWithSensor.count(l.name)) continue;
    const SensorKind kind = kindFromLinkName(l.name);
    if (kind == SensorKind::Unsupported) continue;
    SensorCandidate c;
    c.kind = kind;
    c.link = l.name;
    std::string base = endsWith(l.name, "_link") && l.name.size() > 5 ? l.name.substr(0, l.name.size() - 5) : l.name;
    base = sanitize(base);
    c.name = base;
    for (int n = 2; names.count(c.name); ++n) c.name = base + "_" + std::to_string(n);
    names.insert(c.name);
    c.id = "sensor:" + c.name;
    c.sdfType = sdfSensorType(kind);
    c.frameLink = frameFor(m, kind, c.name, c.link);
    c.reason = "the name of the link '" + l.name + "' suggests a " + sensorKindLabel(kind);
    d.sensors.push_back(c);
  }
}

// ------------------------------------------------------------------------------ wheels

struct Wheel {
  const Joint* joint = nullptr;
  Vec3 pos;            // wheel centre in the root frame
  Vec3 axis;           // joint axis in the root frame
  double radius = 0;
  bool mecanum = false;
  const Joint* steer = nullptr;  // the vertical joint this wheel is mounted on, if any
  Vec3 steerPos;
};

double shapeRadius(const Geometry& g) {
  return (g.kind == Geometry::Kind::Cylinder || g.kind == Geometry::Kind::Sphere) ? g.a : 0.0;
}

/// Distance of a point from the line through `origin` along the unit vector `axis`.
double distanceFromAxis(const Vec3& p, const Vec3& origin, const Vec3& axis) {
  const Vec3 v = p - origin;
  const Vec3 along = axis * dot(v, axis);
  return norm(v - along);
}

/// The outer radius of a wheel: how far the surface of its cylinders and spheres reaches from the
/// axle, measured from the shape's centre (the wheel itself, and everything fixed to it: the
/// rollers of a mecanum wheel sit on a ring around the hub). Boxes and meshes on a wheel, such as
/// a sensor bolted to it, do not count.
double wheelRadiusOf(const UrdfModel& m, const Wheel& w, bool& hasRollers) {
  double r = 0;
  hasRollers = false;
  std::vector<std::string> stack{w.joint->child};
  std::set<std::string> seen;
  while (!stack.empty()) {
    const std::string name = stack.back();
    stack.pop_back();
    if (!seen.insert(name).second) continue;
    const Link* l = m.link(name);
    if (!l) continue;
    const double sr = shapeRadius(l->shape());
    if (sr > 0) r = std::max(r, distanceFromAxis(m.poseOf(name).apply(l->shapeXyz), w.pos, w.axis) + sr);
    if (name != w.joint->child && hasToken(name, {"roller"})) hasRollers = true;
    for (const Joint* c : m.childJoints(name)) {
      if (c->type == JointType::Fixed) stack.push_back(c->child);
    }
  }
  return r;
}

double mean(const std::vector<double>& v) {
  double s = 0;
  for (double x : v) s += x;
  return v.empty() ? 0 : s / static_cast<double>(v.size());
}

void detectDrive(const UrdfModel& m, Detection& d, Issues& issues) {
  std::vector<Wheel> wheels;
  for (const Joint* j : m.movableJoints()) {
    if (j->type != JointType::Continuous && j->type != JointType::Revolute) continue;
    if (hasToken(j->name, {"roller"}) || hasToken(j->child, {"roller"})) continue;
    Wheel w;
    w.joint = j;
    w.pos = m.jointPosition(*j);
    w.axis = m.jointAxisInRoot(*j);
    if (std::fabs(w.axis.z) > 0.5) continue;  // a wheel turns about a horizontal axle
    const Link* child = m.link(j->child);
    const bool named = hasToken(j->name, {"wheel", "tire", "tyre"}) || hasToken(j->child, {"wheel", "tire", "tyre"});
    const bool shaped = child && shapeRadius(child->shape()) > 0;
    if (!(named || (j->type == JointType::Continuous && shaped))) continue;
    bool rollers = false;
    w.radius = wheelRadiusOf(m, w, rollers);
    w.mecanum = rollers || hasToken(j->name, {"mecanum"}) || hasToken(j->child, {"mecanum"});
    // Mounted on a steering knuckle?
    if (const Joint* pj = m.parentJoint(j->parent)) {
      if ((pj->type == JointType::Revolute || pj->type == JointType::Continuous) && std::fabs(m.jointAxisInRoot(*pj).z) > 0.8) {
        w.steer = pj;
        w.steerPos = m.jointPosition(*pj);
      }
    }
    wheels.push_back(w);
  }
  // A joint on a link that hangs from another wheel is a roller of it, not a wheel.
  wheels.erase(std::remove_if(wheels.begin(), wheels.end(),
                              [&](const Wheel& w) {
                                for (const Wheel& o : wheels) {
                                  if (o.joint != w.joint && o.joint->child == w.joint->parent) return true;
                                }
                                return false;
                              }),
               wheels.end());
  if (wheels.empty()) return;

  const double eps = 1e-6;
  auto names = [](const std::vector<const Wheel*>& ws) {
    std::vector<std::string> out;
    for (const Wheel* w : ws) out.push_back(w->joint->name);
    return out;
  };

  std::set<const Joint*> steers;
  for (const Wheel& w : wheels) {
    if (w.steer) steers.insert(w.steer);
  }

  // Wheel axles must point to the robot's left (+y), otherwise a positive command drives backwards.
  for (const Wheel& w : wheels) {
    if (w.axis.y < 0.5) {
      addIssue(issues, Severity::Warning, kWhere,
               "Wheel joint '" + w.joint->name + "' has its axle along (" + f2(w.axis.x) + ", " + f2(w.axis.y) + ", " + f2(w.axis.z) +
                   ") in the robot frame. Gazebo's drive systems assume +y (REP 103): a positive wheel speed would roll this wheel backwards.");
    }
  }

  DriveCandidate dc;

  if (steers.size() >= 2) {
    dc.kind = DriveKind::Ackermann;
    dc.id = "drive:ackermann";
    std::vector<const Wheel*> driven, steered;
    for (const Wheel& w : wheels) (w.steer ? steered : driven).push_back(&w);
    if (driven.empty()) {
      for (const Wheel& w : wheels) driven.push_back(&w);
    }
    std::vector<const Wheel*> dl, dr;
    for (const Wheel* w : driven) {
      if (w->pos.y > eps) dl.push_back(w);
      else if (w->pos.y < -eps) dr.push_back(w);
    }
    const Joint *sl = nullptr, *sr = nullptr;
    for (const Joint* s : steers) {
      const double y = m.jointPosition(*s).y;
      if (y > eps && !sl) sl = s;
      else if (y < -eps && !sr) sr = s;
    }
    if (dl.empty() || dr.empty() || !sl || !sr) {
      addIssue(issues, Severity::Warning, kWhere, "Steering joints were found, but not one driven wheel and one steering joint on each side; no Ackermann drive is suggested.");
      return;
    }
    dc.left = names(dl);
    dc.right = names(dr);
    dc.leftSteering = sl->name;
    dc.rightSteering = sr->name;
    std::vector<double> yl, yr, xd, xs, radii;
    for (const Wheel* w : dl) yl.push_back(w->pos.y);
    for (const Wheel* w : dr) yr.push_back(w->pos.y);
    for (const Wheel* w : driven) {
      xd.push_back(w->pos.x);
      radii.push_back(w->radius);
    }
    for (const Wheel* w : steered) xs.push_back(w->steerPos.x);
    dc.wheelSeparation = std::fabs(mean(yl) - mean(yr));
    dc.kingpinWidth = std::fabs(m.jointPosition(*sl).y - m.jointPosition(*sr).y);
    dc.wheelBase = std::fabs(mean(xs) - mean(xd));
    dc.wheelRadius = mean(radii);
    double lim = 1e9;
    for (const Joint* s : {sl, sr}) {
      if (s->hasLimit) lim = std::min(lim, std::min(std::fabs(s->lower), std::fabs(s->upper)));
    }
    dc.steeringLimit = lim < 1e8 ? lim : 0;
    dc.reason = std::to_string(steers.size()) + " vertical steering joints with wheels on them, and " + std::to_string(driven.size()) +
                " unsteered wheel(s) to drive";
    d.drives.push_back(dc);
    return;
  }

  bool allMecanum = true, anyMecanum = false;
  for (const Wheel& w : wheels) {
    allMecanum = allMecanum && w.mecanum;
    anyMecanum = anyMecanum || w.mecanum;
  }

  if (allMecanum && wheels.size() == 4) {
    std::vector<double> xs;
    for (const Wheel& w : wheels) xs.push_back(w.pos.x);
    const double mx = mean(xs);
    const Wheel *fl = nullptr, *fr = nullptr, *bl = nullptr, *br = nullptr;
    int placed = 0;
    for (const Wheel& w : wheels) {
      const bool front = w.pos.x > mx, left = w.pos.y > 0;
      const Wheel*& slot = front ? (left ? fl : fr) : (left ? bl : br);
      if (!slot) {
        slot = &w;
        ++placed;
      }
    }
    if (placed == 4) {
      dc.kind = DriveKind::Mecanum;
      dc.id = "drive:mecanum";
      dc.frontLeft = fl->joint->name;
      dc.frontRight = fr->joint->name;
      dc.backLeft = bl->joint->name;
      dc.backRight = br->joint->name;
      dc.wheelSeparation = std::fabs((fl->pos.y + bl->pos.y) / 2 - (fr->pos.y + br->pos.y) / 2);
      dc.wheelBase = std::fabs((fl->pos.x + fr->pos.x) / 2 - (bl->pos.x + br->pos.x) / 2);
      dc.wheelRadius = (fl->radius + fr->radius + bl->radius + br->radius) / 4;
      dc.reason = "four wheels with rollers (mecanum), one in each corner";
      d.drives.push_back(dc);
      return;
    }
    addIssue(issues, Severity::Warning, kWhere, "Four mecanum wheels were found, but not one in each corner; they are treated as a differential drive.");
  } else if (anyMecanum) {
    addIssue(issues, Severity::Warning, kWhere,
             "The robot mixes mecanum and ordinary wheels (or has a number of mecanum wheels other than four). Gazebo's mecanum drive "
             "needs exactly four mecanum wheels, so a differential drive is suggested instead; it cannot move sideways.");
  }

  std::vector<const Wheel*> l, r;
  for (const Wheel& w : wheels) {
    if (w.pos.y > eps) l.push_back(&w);
    else if (w.pos.y < -eps) r.push_back(&w);
  }
  if (l.empty() || r.empty()) {
    addIssue(issues, Severity::Warning, kWhere, "Wheels were found, but not on both sides of the robot's x axis; no differential drive is suggested.");
    return;
  }
  dc.kind = DriveKind::Diff;
  dc.id = "drive:diff";
  dc.left = names(l);
  dc.right = names(r);
  std::vector<double> yl, yr, radii;
  for (const Wheel* w : l) yl.push_back(w->pos.y);
  for (const Wheel* w : r) yr.push_back(w->pos.y);
  double rmin = 1e9, rmax = 0;
  for (const Wheel& w : wheels) {
    radii.push_back(w.radius);
    if (w.radius > 0) {
      rmin = std::min(rmin, w.radius);
      rmax = std::max(rmax, w.radius);
    }
  }
  dc.wheelSeparation = std::fabs(mean(yl) - mean(yr));
  dc.wheelRadius = mean(radii);
  if (rmax > 0 && (rmax - rmin) / rmax > 0.02) {
    addIssue(issues, Severity::Warning, kWhere, "The wheels have different radii (" + f2(rmin) + " to " + f2(rmax) + " m); one wheel radius is used for the odometry.");
  }
  if (l.size() != r.size()) {
    addIssue(issues, Severity::Warning, kWhere, "There are " + std::to_string(l.size()) + " wheel(s) on the left and " + std::to_string(r.size()) + " on the right.");
  }
  dc.reason = std::to_string(wheels.size()) + " wheel joint(s): " + std::to_string(l.size()) + " left, " + std::to_string(r.size()) + " right" +
              (wheels.size() > 2 ? " (skid steering: all wheels of a side get the same speed)" : "");
  d.drives.push_back(dc);
}

// --------------------------------------------------------------------------- the item list

std::string describeSensor(const SensorCandidate& s) {
  std::string out = std::string(sensorKindLabel(s.kind)) + " on link " + s.link;
  switch (s.kind) {
    case SensorKind::Camera: out += ": Image and CameraInfo"; break;
    case SensorKind::Imu: out += ": Imu"; break;
    case SensorKind::Lidar: out += ": LaserScan and PointCloud2"; break;
    case SensorKind::Gps: out += ": NavSatFix"; break;
    case SensorKind::Magnetometer: out += ": MagneticField"; break;
    case SensorKind::AirPressure: out += ": FluidPressure"; break;
    case SensorKind::Altimeter: out += ": Altimeter"; break;
    default: break;
  }
  out += s.existing ? " (already in the URDF; it only gets bridged)" : " (" + s.reason + "; a <sensor> block is added to the copy of the URDF)";
  return out;
}

}  // namespace

Detection detect(const UrdfModel& m, const std::vector<ParsedYaml>& yamls, Issues& issues) {
  Detection d;
  if (!m.valid) return d;

  // ---- control mode
  d.ros2.urdfHasTag = !m.ros2control.empty();
  for (const ParsedYaml& y : yamls) {
    if (!y.is(YamlKind::Controllers)) continue;
    d.ros2.controllerFiles.push_back(y.path);
    for (const ControllerSpec& c : y.controllers.controllers) {
      const bool dup = std::any_of(d.ros2.controllers.begin(), d.ros2.controllers.end(), [&](const ControllerSpec& o) { return o.name == c.name; });
      if (dup) addIssue(issues, Severity::Warning, "YAML " + baseName(y.path), "The controller '" + c.name + "' is defined in more than one file; the first definition is used.");
      else d.ros2.controllers.push_back(c);
    }
  }
  d.mode = (d.ros2.urdfHasTag || !d.ros2.controllerFiles.empty()) ? ControlMode::Ros2Control : ControlMode::GzSystems;

  // ---- user switches (overrides) that decide the suggestions
  std::map<std::string, SensorOverride> sensorOv;
  DriveOverride driveOv;
  std::optional<bool> jointStatesOv;
  std::optional<std::vector<std::string>> positionOv;
  for (const ParsedYaml& y : yamls) {
    if (!y.is(YamlKind::Overrides)) continue;
    for (const auto& kv : y.overrides.sensors) sensorOv[kv.first] = kv.second;
    if (y.overrides.drive.enabled) driveOv.enabled = y.overrides.drive.enabled;
    if (y.overrides.jointStates) jointStatesOv = y.overrides.jointStates;
    if (y.overrides.positionControl) positionOv = y.overrides.positionControl;
  }

  detectSensors(m, d, issues);
  for (const SensorCandidate& s : d.sensors) {
    Item it;
    it.id = s.id;
    it.category = "Sensors";
    it.title = s.name + " (" + sensorKindLabel(s.kind) + ")";
    it.detail = describeSensor(s);
    it.existing = s.existing;
    auto ov = sensorOv.find(s.name);
    if (ov != sensorOv.end() && ov->second.enabled) it.selected = *ov->second.enabled;
    d.items.push_back(it);
    if (s.existing && s.needsTopic) {
      Item p;
      p.id = "sensortopic:" + s.name;
      p.category = "Sensors";
      p.title = s.name + ": add a <topic>";
      p.detail = "The sensor has no <topic>. Without one its Gazebo topic depends on how sdformat lumps links, so a <topic> is added to this sensor in the copy of the URDF; "
                 "without it the sensor cannot be bridged.";
      d.items.push_back(p);
    }
  }

  if (d.mode == ControlMode::Ros2Control) {
    Ros2ControlInfo& r = d.ros2;
    r.pluginPresent = m.hasGazeboPlugin("gz_ros2_control");
    if (!m.ros2control.empty()) r.hardwarePlugin = m.ros2control.front().hardwarePlugin;
    r.hardwareNeedsPatch = r.urdfHasTag && r.hardwarePlugin != "gz_ros2_control/GazeboSimSystem";
    if (!r.pluginPresent) {
      Item it;
      it.id = "ros2control:plugin";
      it.category = "ros2_control";
      it.title = "Add the gz_ros2_control plugin";
      it.detail = "The URDF has no <gazebo><plugin filename=\"gz_ros2_control-system\">. It is what starts the controller manager inside Gazebo. "
                  "It will be added to the copy of the URDF with the controller YAML file(s) as its <parameters>.";
      if (r.controllerFiles.empty()) it.detail += " (No controllers YAML was given, so there will be nothing to load: add one.)";
      d.items.push_back(it);
    }
    if (!r.urdfHasTag) {
      addIssue(issues, Severity::Warning, "URDF",
               "Controllers were given but the URDF has no <ros2_control> element describing the joints' command and state interfaces, "
               "so the controllers will have no joints to claim. Add one (hardware plugin gz_ros2_control/GazeboSimSystem).");
    }
    if (r.hardwareNeedsPatch) {
      Item it;
      it.id = "ros2control:hardware";
      it.category = "ros2_control";
      it.title = "Use gz_ros2_control/GazeboSimSystem as the hardware plugin";
      it.detail = "The <ros2_control> hardware plugin is '" + (r.hardwarePlugin.empty() ? std::string("(none)") : r.hardwarePlugin) +
                  "'. In Gazebo the joints must be driven by gz_ros2_control/GazeboSimSystem; the copy of the URDF gets that instead.";
      d.items.push_back(it);
    }
    if (m.hasGazeboPlugin("gz-sim-diff-drive") || m.hasGazeboPlugin("gz-sim-joint-state-publisher")) {
      addIssue(issues, Severity::Warning, "URDF", "The URDF has Gazebo drive/joint plugins as well as ros2_control; both would move the same joints.");
    }
    if (r.controllers.empty() && !r.controllerFiles.empty()) {
      addIssue(issues, Severity::Warning, "Detection", "No controllers were found in the controllers YAML.");
    }
    return d;
  }

  // ---- Gazebo systems: drive, joints, odometry
  if (!(driveOv.enabled && !*driveOv.enabled)) detectDrive(m, d, issues);
  std::set<std::string> claimed;
  for (const DriveCandidate& dc : d.drives) {
    Item it;
    it.id = dc.id;
    it.category = "Drive";
    it.title = std::string(driveKindLabel(dc.kind)) + " (" + (dc.kind == DriveKind::Mecanum ? "gz-sim MecanumDrive" : dc.kind == DriveKind::Ackermann ? "gz-sim AckermannSteering" : "gz-sim DiffDrive") + ")";
    it.detail = dc.reason + ". Wheel separation " + f2(dc.wheelSeparation) + " m, wheel radius " + f2(dc.wheelRadius) +
                " m (both measured from the URDF). Adds /cmd_vel (ROS to Gazebo), /odom and /tf (Gazebo to ROS).";
    if (driveOv.enabled) it.selected = *driveOv.enabled;
    d.items.push_back(it);
    for (const std::string& j : dc.joints()) claimed.insert(j);
  }

  const bool anyMovable = !m.movableJoints().empty();
  d.jointStatesOffered = anyMovable;
  if (anyMovable) {
    Item it;
    it.id = "joint_states";
    it.category = "Joints";
    it.title = "Joint states (gz-sim JointStatePublisher)";
    it.detail = "Publishes the position of all " + std::to_string(m.movableJoints().size()) +
                " movable joint(s) as /joint_states, which robot_state_publisher needs to draw moving links in TF.";
    if (jointStatesOv) it.selected = *jointStatesOv;
    d.items.push_back(it);
  }

  for (const Joint* j : m.movableJoints()) {
    if (j->type != JointType::Revolute && j->type != JointType::Prismatic) continue;
    if (claimed.count(j->name)) continue;
    JointControlCandidate jc;
    jc.id = "jointpos:" + j->name;
    jc.joint = j->name;
    jc.lower = j->lower;
    jc.upper = j->upper;
    d.jointControls.push_back(jc);
  }
  if (!d.jointControls.empty()) {
    for (const JointControlCandidate& jc : d.jointControls) {
      Item it;
      it.id = jc.id;
      it.category = "Joints";
      it.title = jc.joint + " (position control)";
      it.detail = "gz-sim JointPositionController: a std_msgs/Float64 on /joint/" + jc.joint + "/cmd_pos sets the angle (" + f2(jc.lower) + " to " + f2(jc.upper) +
                  "). A PID with gains from the file; tune them.";
      if (positionOv) it.selected = std::find(positionOv->begin(), positionOv->end(), jc.joint) != positionOv->end();
      d.items.push_back(it);
    }
    if (positionOv) {
      for (const std::string& j : *positionOv) {
        const bool known = std::any_of(d.jointControls.begin(), d.jointControls.end(), [&](const JointControlCandidate& c) { return c.joint == j; });
        if (!known) addIssue(issues, Severity::Warning, "YAML overrides", "joints.position_control lists '" + j + "', which is not a free revolute/prismatic joint of this URDF.");
      }
    }
  }

  // An odometry source for robots without a drive (the drive systems publish their own).
  for (const Link& l : m.links) {
    if (hasToken(l.name, {"odom", "odometry"}) && !hasToken(l.name, {"optical"})) {
      d.odometry.link = l.name;
      break;
    }
  }
  if (d.drives.empty()) {
    d.odometry.offered = true;
    Item it;
    it.id = "odometry";
    it.category = "Odometry";
    it.title = "Model odometry (gz-sim OdometryPublisher)";
    it.detail = "Publishes the pose of the model as /odom and the odom -> " + m.root + " transform on /tf. There is no wheel drive to do it" +
                (d.odometry.link.empty() ? "." : "; the link '" + d.odometry.link + "' is named like an odometry frame.");
    it.selected = !d.odometry.link.empty();
    d.items.push_back(it);
  } else if (!d.odometry.link.empty()) {
    addIssue(issues, Severity::Info, "Detection", "The link '" + d.odometry.link + "' is named like an odometry frame; the drive system publishes odometry itself, so nothing extra is added.");
  }
  return d;
}

}  // namespace gzb
