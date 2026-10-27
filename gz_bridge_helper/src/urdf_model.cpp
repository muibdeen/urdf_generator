// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include "gz_bridge_helper/urdf_model.hpp"

#include <tinyxml2.h>

#include <algorithm>
#include <cstdlib>
#include <functional>
#include <sstream>

namespace gzb {

using tinyxml2::XMLElement;

const char* jointTypeName(JointType t) {
  switch (t) {
    case JointType::Fixed: return "fixed";
    case JointType::Revolute: return "revolute";
    case JointType::Continuous: return "continuous";
    case JointType::Prismatic: return "prismatic";
    case JointType::Floating: return "floating";
    case JointType::Planar: return "planar";
  }
  return "fixed";
}

namespace {

const char* kWhere = "URDF";

std::string attr(const XMLElement* e, const char* name) {
  const char* v = e ? e->Attribute(name) : nullptr;
  return v ? v : "";
}

std::string childText(const XMLElement* e, const char* name) {
  const XMLElement* c = e ? e->FirstChildElement(name) : nullptr;
  if (!c) return "";
  const char* t = c->GetText();
  return t ? trim(t) : "";
}

bool parseNumbers(const std::string& text, std::vector<double>& out) {
  out.clear();
  std::istringstream in(text);
  std::string tok;
  while (in >> tok) {
    char* end = nullptr;
    const double v = std::strtod(tok.c_str(), &end);
    if (!end || *end != '\0') return false;
    out.push_back(v);
  }
  return true;
}

bool parseVec3(const std::string& text, Vec3& out) {
  std::vector<double> v;
  if (!parseNumbers(text, v) || v.size() != 3) return false;
  out = {v[0], v[1], v[2]};
  return true;
}

bool parseDouble(const std::string& text, double& out) {
  std::vector<double> v;
  if (!parseNumbers(text, v) || v.size() != 1) return false;
  out = v[0];
  return true;
}

Geometry parseGeometry(const XMLElement* geometry, const std::string& owner, Issues& issues) {
  Geometry g;
  if (!geometry) return g;
  const XMLElement* shape = geometry->FirstChildElement();
  if (!shape) return g;
  const std::string kind = shape->Name();
  std::vector<double> v;
  if (kind == "box") {
    if (parseNumbers(attr(shape, "size"), v) && v.size() == 3) {
      g.kind = Geometry::Kind::Box;
      g.a = v[0];
      g.b = v[1];
      g.c = v[2];
    } else {
      addIssue(issues, Severity::Error, kWhere, "Link '" + owner + "': <box> needs size=\"x y z\".");
    }
  } else if (kind == "cylinder") {
    double r = 0, l = 0;
    if (parseDouble(attr(shape, "radius"), r) && parseDouble(attr(shape, "length"), l)) {
      g.kind = Geometry::Kind::Cylinder;
      g.a = r;
      g.b = l;
    } else {
      addIssue(issues, Severity::Error, kWhere, "Link '" + owner + "': <cylinder> needs radius and length.");
    }
  } else if (kind == "sphere") {
    double r = 0;
    if (parseDouble(attr(shape, "radius"), r)) {
      g.kind = Geometry::Kind::Sphere;
      g.a = r;
    } else {
      addIssue(issues, Severity::Error, kWhere, "Link '" + owner + "': <sphere> needs a radius.");
    }
  } else if (kind == "mesh") {
    g.kind = Geometry::Kind::Mesh;
    g.uri = attr(shape, "filename");
    if (g.uri.empty()) addIssue(issues, Severity::Error, kWhere, "Link '" + owner + "': <mesh> has no filename.");
  } else {
    addIssue(issues, Severity::Warning, kWhere, "Link '" + owner + "': unknown geometry <" + kind + ">.");
  }
  return g;
}

JointType parseJointType(const std::string& t, bool& ok) {
  ok = true;
  if (t == "fixed") return JointType::Fixed;
  if (t == "revolute") return JointType::Revolute;
  if (t == "continuous") return JointType::Continuous;
  if (t == "prismatic") return JointType::Prismatic;
  if (t == "floating") return JointType::Floating;
  if (t == "planar") return JointType::Planar;
  ok = false;
  return JointType::Fixed;
}

bool looksLikeXacro(const XMLElement* root) {
  std::function<bool(const XMLElement*)> walk = [&](const XMLElement* e) {
    if (startsWith(e->Name(), "xacro:")) return true;
    for (const tinyxml2::XMLAttribute* a = e->FirstAttribute(); a; a = a->Next()) {
      const std::string n = a->Name(), v = a->Value();
      if (startsWith(n, "xmlns:xacro") || v.find("${") != std::string::npos || v.find("$(") != std::string::npos) return true;
    }
    for (const XMLElement* c = e->FirstChildElement(); c; c = c->NextSiblingElement()) {
      if (walk(c)) return true;
    }
    return false;
  };
  return walk(root);
}

void noteMesh(UrdfModel& m, const Geometry& g, const std::string& owner, Issues& issues) {
  if (g.kind != Geometry::Kind::Mesh || g.uri.empty()) return;
  const std::string pre = "package://";
  if (startsWith(g.uri, pre)) {
    const std::string rest = g.uri.substr(pre.size());
    const size_t slash = rest.find('/');
    if (slash != std::string::npos && slash > 0) m.meshPackages.insert(rest.substr(0, slash));
  } else if (startsWith(g.uri, "file://") || startsWith(g.uri, "/")) {
    addIssue(issues, Severity::Warning, kWhere,
             "Link '" + owner + "' loads the mesh '" + g.uri + "' from a fixed path; it will not be found on another computer. Use package://<package>/... instead.");
  } else if (!startsWith(g.uri, "model://")) {
    addIssue(issues, Severity::Warning, kWhere,
             "Link '" + owner + "' has the mesh '" + g.uri + "' without a package:// or model:// prefix; Gazebo may not find it.");
  }
}

void parseGazebo(const XMLElement* gz, UrdfModel& m) {
  const std::string reference = attr(gz, "reference");
  for (const XMLElement* s = gz->FirstChildElement("sensor"); s; s = s->NextSiblingElement("sensor")) {
    GazeboSensor sensor;
    sensor.reference = reference;
    sensor.name = attr(s, "name");
    sensor.type = attr(s, "type");
    sensor.topic = childText(s, "topic");
    sensor.gzFrameId = childText(s, "gz_frame_id");
    const std::string rate = childText(s, "update_rate");
    if (!rate.empty()) parseDouble(rate, sensor.updateRate);
    m.sensors.push_back(sensor);
  }
  for (const XMLElement* p = gz->FirstChildElement("plugin"); p; p = p->NextSiblingElement("plugin")) {
    GazeboPlugin plugin;
    plugin.reference = reference;
    plugin.filename = attr(p, "filename");
    plugin.name = attr(p, "name");
    plugin.parameters = childText(p, "parameters");
    m.plugins.push_back(plugin);
  }
}

void parseRos2Control(const XMLElement* rc, UrdfModel& m) {
  Ros2Control c;
  c.name = attr(rc, "name");
  c.type = attr(rc, "type");
  const XMLElement* hw = rc->FirstChildElement("hardware");
  c.hardwarePlugin = childText(hw, "plugin");
  for (const XMLElement* j = rc->FirstChildElement("joint"); j; j = j->NextSiblingElement("joint")) {
    Ros2ControlJoint joint;
    joint.name = attr(j, "name");
    for (const XMLElement* i = j->FirstChildElement("command_interface"); i; i = i->NextSiblingElement("command_interface")) {
      joint.commandInterfaces.push_back(attr(i, "name"));
    }
    for (const XMLElement* i = j->FirstChildElement("state_interface"); i; i = i->NextSiblingElement("state_interface")) {
      joint.stateInterfaces.push_back(attr(i, "name"));
    }
    c.joints.push_back(joint);
  }
  m.ros2control.push_back(c);
}

}  // namespace

// ------------------------------------------------------------------------------ queries

const Link* UrdfModel::link(const std::string& n) const {
  for (const Link& l : links) {
    if (l.name == n) return &l;
  }
  return nullptr;
}

const Joint* UrdfModel::joint(const std::string& n) const {
  for (const Joint& j : joints) {
    if (j.name == n) return &j;
  }
  return nullptr;
}

const Joint* UrdfModel::parentJoint(const std::string& l) const {
  for (const Joint& j : joints) {
    if (j.child == l) return &j;
  }
  return nullptr;
}

std::vector<const Joint*> UrdfModel::childJoints(const std::string& l) const {
  std::vector<const Joint*> out;
  for (const Joint& j : joints) {
    if (j.parent == l) out.push_back(&j);
  }
  return out;
}

std::vector<const Joint*> UrdfModel::movableJoints() const {
  std::vector<const Joint*> out;
  for (const Joint& j : joints) {
    if (j.movable()) out.push_back(&j);
  }
  return out;
}

Transform UrdfModel::poseOf(const std::string& l) const {
  auto it = poses.find(l);
  return it == poses.end() ? Transform() : it->second;
}

Vec3 UrdfModel::jointPosition(const Joint& j) const { return poseOf(j.child).t; }

Vec3 UrdfModel::jointAxisInRoot(const Joint& j) const { return normalized(poseOf(j.child).rotate(j.axis)); }

bool UrdfModel::hasGazeboPlugin(const std::string& sub) const {
  for (const GazeboPlugin& p : plugins) {
    if (p.filename.find(sub) != std::string::npos || p.name.find(sub) != std::string::npos) return true;
  }
  return false;
}

// ------------------------------------------------------------------------------- parsing

UrdfModel parseUrdf(const std::string& xml, Issues& issues) {
  UrdfModel m;
  tinyxml2::XMLDocument doc;
  if (doc.Parse(xml.data(), xml.size()) != tinyxml2::XML_SUCCESS) {
    std::string msg = std::string("The file is not well-formed XML: ") + doc.ErrorStr();
    addIssue(issues, Severity::Error, kWhere, msg);
    return m;
  }
  const XMLElement* robot = doc.FirstChildElement("robot");
  if (!robot) {
    addIssue(issues, Severity::Error, kWhere, "There is no <robot> element, so this is not a URDF file.");
    return m;
  }
  m.name = attr(robot, "name");
  if (m.name.empty()) addIssue(issues, Severity::Warning, kWhere, "The <robot> has no name; 'robot' is used.");
  if (m.name.empty()) m.name = "robot";

  m.xacro = looksLikeXacro(robot);
  if (m.xacro) {
    addIssue(issues, Severity::Error, kWhere,
             "This file still contains xacro (xacro: elements or ${...} expressions). Expand it first, for example: "
             "xacro robot.urdf.xacro > robot.urdf");
    return m;
  }

  std::set<std::string> linkNames, jointNames;
  for (const XMLElement* e = robot->FirstChildElement("link"); e; e = e->NextSiblingElement("link")) {
    Link l;
    l.name = attr(e, "name");
    if (l.name.empty()) {
      addIssue(issues, Severity::Error, kWhere, "A <link> has no name.");
      continue;
    }
    if (!linkNames.insert(l.name).second) {
      addIssue(issues, Severity::Error, kWhere, "There is more than one link named '" + l.name + "'.");
      continue;
    }
    if (const XMLElement* in = e->FirstChildElement("inertial")) {
      l.hasInertial = true;
      const XMLElement* mass = in->FirstChildElement("mass");
      if (!mass || !parseDouble(attr(mass, "value"), l.mass)) {
        addIssue(issues, Severity::Error, kWhere, "Link '" + l.name + "': <inertial> needs <mass value=\"...\"/>.");
      } else if (l.mass <= 0) {
        addIssue(issues, Severity::Warning, kWhere, "Link '" + l.name + "' has a mass of " + num(l.mass) + " kg.");
      }
    }
    const XMLElement* visualEl = e->FirstChildElement("visual");
    const XMLElement* collisionEl = e->FirstChildElement("collision");
    if (visualEl) l.visual = parseGeometry(visualEl->FirstChildElement("geometry"), l.name, issues);
    if (collisionEl) l.collision = parseGeometry(collisionEl->FirstChildElement("geometry"), l.name, issues);
    const XMLElement* shapeEl = l.collision.kind != Geometry::Kind::None ? collisionEl : visualEl;
    if (shapeEl) {
      if (const XMLElement* o = shapeEl->FirstChildElement("origin")) {
        if (o->Attribute("xyz") && !parseVec3(attr(o, "xyz"), l.shapeXyz)) {
          addIssue(issues, Severity::Error, kWhere, "Link '" + l.name + "': the shape's origin xyz must be three numbers.");
        }
        if (o->Attribute("rpy") && !parseVec3(attr(o, "rpy"), l.shapeRpy)) {
          addIssue(issues, Severity::Error, kWhere, "Link '" + l.name + "': the shape's origin rpy must be three numbers.");
        }
      }
    }
    noteMesh(m, l.visual, l.name, issues);
    if (l.collision.kind == Geometry::Kind::Mesh && l.collision.uri != l.visual.uri) noteMesh(m, l.collision, l.name, issues);
    m.links.push_back(l);
  }
  if (m.links.empty()) {
    addIssue(issues, Severity::Error, kWhere, "The robot has no links.");
    return m;
  }

  for (const XMLElement* e = robot->FirstChildElement("joint"); e; e = e->NextSiblingElement("joint")) {
    Joint j;
    j.name = attr(e, "name");
    if (j.name.empty()) {
      addIssue(issues, Severity::Error, kWhere, "A <joint> has no name.");
      continue;
    }
    if (!jointNames.insert(j.name).second) {
      addIssue(issues, Severity::Error, kWhere, "There is more than one joint named '" + j.name + "'.");
      continue;
    }
    bool typeOk = false;
    j.type = parseJointType(attr(e, "type"), typeOk);
    if (!typeOk) {
      addIssue(issues, Severity::Error, kWhere, "Joint '" + j.name + "' has the unknown type '" + attr(e, "type") + "'.");
      continue;
    }
    j.parent = attr(e->FirstChildElement("parent"), "link");
    j.child = attr(e->FirstChildElement("child"), "link");
    if (!linkNames.count(j.parent) || !linkNames.count(j.child)) {
      addIssue(issues, Severity::Error, kWhere,
               "Joint '" + j.name + "' connects '" + j.parent + "' and '" + j.child + "', and at least one of them is not a link of this robot.");
      continue;
    }
    if (const XMLElement* o = e->FirstChildElement("origin")) {
      if (o->Attribute("xyz") && !parseVec3(attr(o, "xyz"), j.xyz)) {
        addIssue(issues, Severity::Error, kWhere, "Joint '" + j.name + "': origin xyz must be three numbers.");
      }
      if (o->Attribute("rpy") && !parseVec3(attr(o, "rpy"), j.rpy)) {
        addIssue(issues, Severity::Error, kWhere, "Joint '" + j.name + "': origin rpy must be three numbers.");
      }
    }
    if (const XMLElement* a = e->FirstChildElement("axis")) {
      Vec3 ax;
      if (!parseVec3(attr(a, "xyz"), ax) || norm(ax) < 1e-9) {
        addIssue(issues, Severity::Error, kWhere, "Joint '" + j.name + "': the axis must be a non-zero vector.");
      } else {
        j.axis = normalized(ax);
      }
    }
    if (const XMLElement* l = e->FirstChildElement("limit")) {
      j.hasLimit = true;
      parseDouble(attr(l, "lower"), j.lower);
      parseDouble(attr(l, "upper"), j.upper);
      parseDouble(attr(l, "effort"), j.effort);
      parseDouble(attr(l, "velocity"), j.velocity);
    }
    if ((j.type == JointType::Revolute || j.type == JointType::Prismatic) && !j.hasLimit) {
      addIssue(issues, Severity::Error, kWhere, "Joint '" + j.name + "' is " + jointTypeName(j.type) + " and needs a <limit>.");
    }
    m.joints.push_back(j);
  }

  for (const XMLElement* e = robot->FirstChildElement("gazebo"); e; e = e->NextSiblingElement("gazebo")) {
    parseGazebo(e, m);
    const std::string ref = attr(e, "reference");
    if (!ref.empty() && !linkNames.count(ref) && !jointNames.count(ref)) {
      addIssue(issues, Severity::Warning, kWhere, "<gazebo reference=\"" + ref + "\"> refers to something that is neither a link nor a joint.");
    }
  }
  for (const XMLElement* e = robot->FirstChildElement("ros2_control"); e; e = e->NextSiblingElement("ros2_control")) {
    parseRos2Control(e, m);
  }

  // The links and joints must form one tree.
  std::map<std::string, int> parents;
  for (const Joint& j : m.joints) {
    if (++parents[j.child] > 1) {
      addIssue(issues, Severity::Error, kWhere, "Link '" + j.child + "' is the child of more than one joint, so the robot is not a tree.");
    }
  }
  std::vector<std::string> roots;
  for (const Link& l : m.links) {
    if (!parents.count(l.name)) roots.push_back(l.name);
  }
  if (roots.size() != 1) {
    std::string list;
    for (const std::string& r : roots) list += (list.empty() ? "" : ", ") + r;
    addIssue(issues, Severity::Error, kWhere,
             roots.empty() ? std::string("Every link is the child of a joint, so there is a loop and no root link.")
                           : "The robot has " + std::to_string(roots.size()) + " root links (" + list + "); it must have exactly one.");
    return m;
  }
  m.root = roots.front();

  // Poses of every link at joint position 0, by walking down from the root.
  m.poses[m.root] = Transform();
  std::vector<std::string> stack{m.root};
  while (!stack.empty()) {
    const std::string cur = stack.back();
    stack.pop_back();
    for (const Joint& j : m.joints) {
      if (j.parent != cur || m.poses.count(j.child)) continue;
      m.poses[j.child] = m.poses[cur] * Transform::fromXyzRpy(j.xyz, j.rpy);
      stack.push_back(j.child);
    }
  }
  if (m.poses.size() != m.links.size()) {
    addIssue(issues, Severity::Error, kWhere, "Some links are not connected to the root link '" + m.root + "' (a loop of joints?).");
    return m;
  }

  // Gazebo needs a mass for every part that has a shape.
  std::vector<std::string> massless;
  for (const Link& l : m.links) {
    if (!l.hasInertial && (l.visual.kind != Geometry::Kind::None || l.collision.kind != Geometry::Kind::None)) massless.push_back(l.name);
  }
  if (!massless.empty()) {
    std::string list;
    for (size_t i = 0; i < massless.size() && i < 4; ++i) list += (i ? ", " : "") + massless[i];
    if (massless.size() > 4) list += ", ...";
    addIssue(issues, Severity::Warning, kWhere,
             std::to_string(massless.size()) + " link(s) with a shape have no <inertial> (" + list +
                 "). Gazebo needs a mass and inertia to simulate a link; without them it is treated as massless.");
  }

  m.valid = true;
  return m;
}

}  // namespace gzb
