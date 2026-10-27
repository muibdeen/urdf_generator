// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// A small in-memory robot model. Builders fill it in; the emitters turn it
// into xacro or plain URDF; validate() checks it.
#pragma once

#include <array>
#include <functional>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "urdf_generator/num.hpp"

namespace urdfgen {

struct Origin {
  Num x, y, z, roll, pitch, yaw;
  Origin() = default;
  Origin(Num x_, Num y_, Num z_, Num r_ = 0, Num p_ = 0, Num w_ = 0)
      : x(x_), y(y_), z(z_), roll(r_), pitch(p_), yaw(w_) {}
  bool isIdentity() const {
    return x.isZeroLiteral() && y.isZeroLiteral() && z.isZeroLiteral() &&
           roll.isZeroLiteral() && pitch.isZeroLiteral() && yaw.isZeroLiteral();
  }
};

struct Geometry {
  enum class Kind { Box, Cylinder, Sphere };
  Kind kind = Kind::Box;
  Num a, b, c;  // Box: size x,y,z.  Cylinder: a = radius, b = length.  Sphere: a = radius.

  static Geometry box(Num x, Num y, Num z) { return {Kind::Box, x, y, z}; }
  static Geometry cylinder(Num radius, Num length) { return {Kind::Cylinder, radius, length, 0}; }
  static Geometry sphere(Num radius) { return {Kind::Sphere, radius, 0, 0}; }
};

enum class SensorType { Camera, Imu, Lidar, Gps, Odometry };

/// A sensor that was added to a robot: the name, its type, the link that is the sensor
/// (its frame is the sensor frame) and the link it is mounted on. Used by the 3D view.
struct SensorMark {
  std::string name;
  SensorType type = SensorType::Camera;
  std::string link;    // the sensor's own link, e.g. "camera_link"
  std::string parent;  // the link it is attached to
};

/// One rigid body. Visual, collision and inertial all use the same primitive
/// shape and origin, so the three can never disagree.
struct Link {
  std::string name;
  bool hasBody = true;  // false: a bare frame (no visual/collision/inertial)
  Origin origin;
  Geometry geometry;
  std::string material;
  Num mass;
  bool attachable = true;  // false for decoration (mecanum rollers) that sensors cannot be mounted on
};

enum class JointType { Fixed, Revolute, Continuous, Prismatic };

struct Joint {
  std::string name;
  JointType type = JointType::Fixed;
  std::string parent, child;
  Origin origin;
  std::array<double, 3> axis{{0, 0, 1}};
  Num lower, upper, effort, velocity;  // used by Revolute and Prismatic only
};

struct Material {
  std::string name;
  double r = 0.5, g = 0.5, b = 0.5, a = 1.0;
};

/// Links and joints produced by a builder or by a macro expansion.
struct Part {
  std::vector<Link> links;
  std::vector<Joint> joints;
};

/// Arguments passed to a macro: strings (names) and numbers (placements).
struct MacroArgs {
  std::map<std::string, std::string> strs;
  std::map<std::string, Num> nums;

  MacroArgs& setStr(const std::string& key, const std::string& v) { strs[key] = v; return *this; }
  MacroArgs& setNum(const std::string& key, const Num& v) { nums[key] = v; return *this; }
  const std::string& str(const std::string& key) const { return strs.at(key); }
  const Num& num(const std::string& key) const { return nums.at(key); }
};

/// A repeated sub-structure (a leg, a wheel). It becomes an <xacro:macro> in
/// the xacro output and is expanded in place for the plain URDF output.
struct MacroDef {
  std::string name;
  std::vector<std::string> strParams, numParams;
  std::function<void(Part&, const MacroArgs&)> body;
};

struct MacroInstance {
  std::string macro;
  MacroArgs args;
};

struct Property {
  std::string name;
  Num value;
  std::string comment;
  bool isHeader = false;  // a section title, not a real property
};

struct Issue {
  enum class Severity { Error, Warning };
  Severity severity = Severity::Error;
  std::string message;
};

class Robot {
 public:
  explicit Robot(std::string robotName) : name(std::move(robotName)) {}

  std::string name;
  std::vector<Property> properties;
  std::vector<Material> materials;
  Part root;
  std::vector<MacroDef> macros;
  std::vector<MacroInstance> instances;
  /// Problems a builder noticed that the generic checks cannot see (reported by validate()).
  std::vector<Issue> buildIssues;
  /// Sensors that addSensors() put on the robot.
  std::vector<SensorMark> sensorMarks;

  /// Start a new group of properties in the xacro output.
  void section(const std::string& title) {
    Property p;
    p.isHeader = true;
    p.comment = title;
    properties.push_back(p);
  }

  /// Register a named xacro property and return a Num that refers to it.
  Num prop(const std::string& propName, double value, const std::string& comment = "") {
    for (const auto& p : properties) {
      if (!p.isHeader && p.name == propName) {
        throw std::logic_error("duplicate property: " + propName);
      }
    }
    Property p;
    p.name = propName;
    p.value = Num(value);
    p.comment = comment;
    properties.push_back(p);
    return Num::param(propName, value);
  }

  /// Register a named xacro property whose value is computed from other properties
  /// (written as ${expression}), and return a Num that refers to it.
  Num derived(const std::string& propName, const Num& expr, const std::string& comment = "") {
    for (const auto& p : properties) {
      if (!p.isHeader && p.name == propName) {
        throw std::logic_error("duplicate property: " + propName);
      }
    }
    Property p;
    p.name = propName;
    p.value = expr;
    p.comment = comment;
    properties.push_back(p);
    return Num::param(propName, expr.value());
  }

  void addMaterial(const std::string& matName, double r, double g, double b, double a = 1.0) {
    materials.push_back({matName, r, g, b, a});
  }

  void addMacro(const MacroDef& m) { macros.push_back(m); }
  void instantiate(const std::string& macroName, const MacroArgs& args) {
    instances.push_back({macroName, args});
  }

  const MacroDef& findMacro(const std::string& macroName) const {
    for (const auto& m : macros) {
      if (m.name == macroName) return m;
    }
    throw std::logic_error("unknown macro: " + macroName);
  }

  /// The macro body with symbolic arguments (what goes inside <xacro:macro>).
  Part macroBody(const MacroDef& m) const {
    MacroArgs a;
    for (const auto& s : m.strParams) a.setStr(s, "${" + s + "}");
    for (const auto& n : m.numParams) a.setNum(n, Num::param(n, 0.0));
    Part p;
    m.body(p, a);
    return p;
  }

  /// One macro instance expanded with its concrete arguments.
  Part expand(const MacroInstance& inst) const {
    Part p;
    findMacro(inst.macro).body(p, inst.args);
    return p;
  }

  /// Everything, with all macros expanded.
  Part flatten() const {
    Part out = root;
    for (const auto& inst : instances) {
      Part p = expand(inst);
      out.links.insert(out.links.end(), p.links.begin(), p.links.end());
      out.joints.insert(out.joints.end(), p.joints.begin(), p.joints.end());
    }
    return out;
  }
};

struct InertiaTensor {
  double ixx = 0, iyy = 0, izz = 0;
};

/// Diagonal inertia tensor of a link's primitive shape, in the shape's own frame.
InertiaTensor computeInertia(const Link& link);

/// Structural and numeric checks on the fully expanded robot.
std::vector<Issue> validate(const Robot& robot);
bool hasErrors(const std::vector<Issue>& issues);

}  // namespace urdfgen
