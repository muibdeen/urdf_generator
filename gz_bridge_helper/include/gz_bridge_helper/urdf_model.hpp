// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// What the helper needs to know about a URDF: its links and joints (with the geometry that
// tells a wheel from a body), the Gazebo sensors and plugins already written into it, and any
// <ros2_control> hardware description. The file is only read; it is never modified.
#pragma once

#include <map>
#include <set>
#include <string>
#include <vector>

#include "gz_bridge_helper/common.hpp"

namespace gzb {

struct Geometry {
  enum class Kind { None, Box, Cylinder, Sphere, Mesh };
  Kind kind = Kind::None;
  double a = 0, b = 0, c = 0;  // box: x y z; cylinder: radius, length; sphere: radius
  std::string uri;             // mesh file name as written in the URDF
};

struct Link {
  std::string name;
  bool hasInertial = false;
  double mass = 0;
  Geometry visual;
  Geometry collision;
  Vec3 shapeXyz, shapeRpy;  // <origin> of shape() in the link frame
  /// The geometry that physically stands for the link: the collision shape, else the visual one.
  const Geometry& shape() const { return collision.kind != Geometry::Kind::None ? collision : visual; }
};

enum class JointType { Fixed, Revolute, Continuous, Prismatic, Floating, Planar };
const char* jointTypeName(JointType t);

struct Joint {
  std::string name;
  JointType type = JointType::Fixed;
  std::string parent, child;
  Vec3 xyz, rpy;           // origin of the child frame in the parent frame
  Vec3 axis{1, 0, 0};      // in the child frame
  bool hasLimit = false;
  double lower = 0, upper = 0, effort = 0, velocity = 0;
  bool movable() const { return type != JointType::Fixed; }
};

/// A <sensor> found inside a <gazebo> block of the URDF.
struct GazeboSensor {
  std::string reference;  // the link of <gazebo reference="...">
  std::string name, type;
  std::string topic;      // empty when the sensor has no <topic>
  std::string gzFrameId;  // empty when it has no <gz_frame_id>
  double updateRate = -1;  // -1 when not given
};

/// A <plugin> found inside a <gazebo> block of the URDF.
struct GazeboPlugin {
  std::string reference;  // empty for a plugin of the whole robot
  std::string filename, name;
  std::string parameters;  // text of a <parameters> child (gz_ros2_control), if any
};

struct Ros2ControlJoint {
  std::string name;
  std::vector<std::string> commandInterfaces, stateInterfaces;
};

struct Ros2Control {
  std::string name, type;
  std::string hardwarePlugin;
  std::vector<Ros2ControlJoint> joints;
};

struct UrdfModel {
  std::string name;
  std::vector<Link> links;
  std::vector<Joint> joints;
  std::vector<GazeboSensor> sensors;
  std::vector<GazeboPlugin> plugins;
  std::vector<Ros2Control> ros2control;
  std::string root;                       // the link that no joint has as its child
  std::set<std::string> meshPackages;     // N of every package://N/... mesh reference
  bool xacro = false;                     // the file still contains xacro (not a plain URDF)
  bool valid = false;                     // a robot with a single tree of links was read
  std::map<std::string, Transform> poses;  // pose of every link in the root frame, all joints at 0

  const Link* link(const std::string& n) const;
  const Joint* joint(const std::string& n) const;
  /// The joint that has `link` as its child (nullptr for the root).
  const Joint* parentJoint(const std::string& link) const;
  std::vector<const Joint*> childJoints(const std::string& link) const;
  std::vector<const Joint*> movableJoints() const;
  Transform poseOf(const std::string& link) const;
  /// Pose of the joint frame (= the pose of its child link) in the root frame.
  Vec3 jointPosition(const Joint& j) const;
  /// Joint axis expressed in the root frame.
  Vec3 jointAxisInRoot(const Joint& j) const;
  bool hasGazeboPlugin(const std::string& filenameSubstring) const;
};

/// Parses URDF text. Problems are appended to `issues`; check `valid` before using the model.
UrdfModel parseUrdf(const std::string& xml, Issues& issues);

}  // namespace gzb
