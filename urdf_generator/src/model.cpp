// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include <cmath>
#include <map>
#include <set>

#include "urdf_generator/model.hpp"

namespace urdfgen {

InertiaTensor computeInertia(const Link& l) {
  const double m = l.mass.value();
  const double a = l.geometry.a.value();
  const double b = l.geometry.b.value();
  const double c = l.geometry.c.value();
  InertiaTensor t;
  switch (l.geometry.kind) {
    case Geometry::Kind::Box:
      t.ixx = m * (b * b + c * c) / 12.0;
      t.iyy = m * (a * a + c * c) / 12.0;
      t.izz = m * (a * a + b * b) / 12.0;
      break;
    case Geometry::Kind::Cylinder:  // a = radius, b = length, axis along z
      t.ixx = t.iyy = m * (3.0 * a * a + b * b) / 12.0;
      t.izz = m * a * a / 2.0;
      break;
    case Geometry::Kind::Sphere:
      t.ixx = t.iyy = t.izz = 2.0 * m * a * a / 5.0;
      break;
  }
  return t;
}

bool hasErrors(const std::vector<Issue>& issues) {
  for (const auto& i : issues) {
    if (i.severity == Issue::Severity::Error) return true;
  }
  return false;
}

namespace {

bool validName(const std::string& s) {
  if (s.empty()) return false;
  for (char ch : s) {
    const bool ok = (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
                    (ch >= '0' && ch <= '9') || ch == '_' || ch == '-' || ch == '.';
    if (!ok) return false;
  }
  return true;
}

const char* kindName(Geometry::Kind k) {
  switch (k) {
    case Geometry::Kind::Box: return "box";
    case Geometry::Kind::Cylinder: return "cylinder";
    case Geometry::Kind::Sphere: return "sphere";
  }
  return "?";
}

}  // namespace

std::vector<Issue> validate(const Robot& robot) {
  std::vector<Issue> out;
  auto err = [&](const std::string& m) { out.push_back({Issue::Severity::Error, m}); };
  auto warn = [&](const std::string& m) { out.push_back({Issue::Severity::Warning, m}); };

  out.insert(out.end(), robot.buildIssues.begin(), robot.buildIssues.end());

  if (!validName(robot.name)) {
    err("Robot name '" + robot.name + "' must be non-empty and use only letters, digits, '_', '-' or '.'.");
  }

  Part p;
  try {
    p = robot.flatten();
  } catch (const std::exception& e) {
    err(std::string("Could not expand macros: ") + e.what());
    return out;
  }

  if (p.links.empty()) {
    err("The robot has no links.");
    return out;
  }

  // ---- links
  std::set<std::string> linkNames;
  for (const auto& l : p.links) {
    if (!validName(l.name)) {
      err("Link name '" + l.name + "' is empty or contains characters other than letters, digits, '_', '-', '.'.");
    }
    if (!linkNames.insert(l.name).second) err("Duplicate link name '" + l.name + "'.");
    if (!l.hasBody) continue;
    if (!(l.mass.value() > 0.0)) err("Link '" + l.name + "' needs a mass greater than zero.");
    const auto& g = l.geometry;
    bool dimsOk = true;
    switch (g.kind) {
      case Geometry::Kind::Box: dimsOk = g.a.value() > 0 && g.b.value() > 0 && g.c.value() > 0; break;
      case Geometry::Kind::Cylinder: dimsOk = g.a.value() > 0 && g.b.value() > 0; break;
      case Geometry::Kind::Sphere: dimsOk = g.a.value() > 0; break;
    }
    if (!dimsOk) err("Link '" + l.name + "' has a " + kindName(g.kind) + " with a non-positive dimension.");
  }

  // ---- joints
  std::set<std::string> jointNames;
  std::map<std::string, int> parentCount;
  std::map<std::string, std::vector<std::string>> children;
  for (const auto& j : p.joints) {
    if (!validName(j.name)) err("Joint name '" + j.name + "' is empty or contains invalid characters.");
    if (!jointNames.insert(j.name).second) err("Duplicate joint name '" + j.name + "'.");
    if (!linkNames.count(j.parent)) err("Joint '" + j.name + "' refers to unknown parent link '" + j.parent + "'.");
    if (!linkNames.count(j.child)) err("Joint '" + j.name + "' refers to unknown child link '" + j.child + "'.");
    if (j.parent == j.child) err("Joint '" + j.name + "' connects link '" + j.parent + "' to itself.");
    parentCount[j.child]++;
    children[j.parent].push_back(j.child);

    if (j.type != JointType::Fixed) {
      const double n2 = j.axis[0] * j.axis[0] + j.axis[1] * j.axis[1] + j.axis[2] * j.axis[2];
      if (!(n2 > 1e-12)) err("Joint '" + j.name + "' has a zero-length axis.");
    }
    if (j.type == JointType::Revolute || j.type == JointType::Prismatic) {
      if (!(j.lower.value() < j.upper.value())) {
        err("Joint '" + j.name + "' needs lower limit < upper limit.");
      }
      if (!(j.effort.value() > 0.0)) err("Joint '" + j.name + "' needs an effort limit greater than zero.");
      if (!(j.velocity.value() > 0.0)) err("Joint '" + j.name + "' needs a velocity limit greater than zero.");
    }
  }
  for (const auto& kv : parentCount) {
    if (kv.second > 1) err("Link '" + kv.first + "' is the child of more than one joint.");
  }

  // ---- tree shape: exactly one root, everything reachable from it
  std::vector<std::string> roots;
  for (const auto& l : p.links) {
    if (!parentCount.count(l.name)) roots.push_back(l.name);
  }
  if (roots.size() != 1) {
    err("The links must form a single tree, but found " + std::to_string(roots.size()) +
        " root link(s) (links that are nobody's child).");
  } else {
    std::set<std::string> seen;
    std::vector<std::string> stack{roots.front()};
    while (!stack.empty()) {
      const std::string cur = stack.back();
      stack.pop_back();
      if (!seen.insert(cur).second) continue;
      for (const auto& c : children[cur]) stack.push_back(c);
    }
    if (seen.size() != linkNames.size()) {
      err("Some links cannot be reached from the root link '" + roots.front() + "' (disconnected or cyclic).");
    }
  }

  // ---- soft checks
  double total = 0;
  for (const auto& l : p.links) {
    if (l.hasBody) total += l.mass.value();
  }
  if (total > 0 && total < 0.01) warn("Total mass is below 10 g; simulators may behave badly.");
  return out;
}

}  // namespace urdfgen
