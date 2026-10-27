// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// Turns a Robot into world-space triangle meshes for the 3D view: forward
// kinematics for a given joint pose (every joint at zero unless told otherwise),
// tessellation of the box / cylinder / sphere primitives, and the bounds of the
// result. No Qt in here, so it is unit tested on its own.
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "urdf_generator/model.hpp"

namespace urdfgen {

struct Vec3 {
  double x = 0, y = 0, z = 0;
};

inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(const Vec3& a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double length(const Vec3& a) { return std::sqrt(dot(a, a)); }
inline Vec3 normalized(const Vec3& a) {
  const double l = length(a);
  return l > 0 ? a * (1.0 / l) : a;
}

/// Rigid transform: rotation (row-major 3x3) and translation.
struct Transform {
  std::array<double, 9> r{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
  Vec3 t;

  Vec3 rotate(const Vec3& v) const {
    return {r[0] * v.x + r[1] * v.y + r[2] * v.z, r[3] * v.x + r[4] * v.y + r[5] * v.z,
            r[6] * v.x + r[7] * v.y + r[8] * v.z};
  }
  Vec3 apply(const Vec3& p) const { return rotate(p) + t; }
  /// this * o : apply o first, then this.
  Transform operator*(const Transform& o) const;
  /// URDF <origin xyz rpy>: R = Rz(yaw) * Ry(pitch) * Rx(roll).
  static Transform fromOrigin(const Origin& o);
};

/// Indexed triangle mesh. Triangles are counter-clockwise seen from outside, and
/// every vertex has a unit normal pointing outwards.
struct Mesh {
  std::vector<Vec3> vertices;
  std::vector<Vec3> normals;
  std::vector<std::array<std::uint32_t, 3>> triangles;
};

/// The primitive in its own frame (cylinder axis along z, everything centred on the origin).
Mesh tessellate(const Geometry& g);

struct SceneBody {
  std::string link;
  Mesh mesh;                     // already in the world frame
  std::array<float, 3> color{{0.7f, 0.7f, 0.7f}};  // from the link's material
};

struct SceneFrame {
  std::string link;
  Transform pose;  // link frame in the world frame
};

/// A sensor of the robot (see sensors.hpp) in the world frame, for drawing its axes.
struct SceneSensor {
  std::string name;
  SensorType type = SensorType::Camera;
  std::string link;    // the sensor's own link
  std::string parent;  // the link it is mounted on
  Transform pose;      // sensor frame in the world frame (x forward, y left, z up)
  Vec3 root;           // origin of the parent link's frame in the world: where the sensor slides from
  std::array<float, 3> color{{0.7f, 0.7f, 0.7f}};
};

struct Scene {
  std::vector<SceneBody> bodies;
  std::vector<SceneFrame> frames;  // every link, the sensors' links included
  std::vector<SceneSensor> sensors;
  Vec3 min, max;  // bounding box of the bodies (of the frames if there are no bodies)
  /// Height of the lowest point of the robot (sensors left out) with every joint at its rest
  /// value. The 3D view puts its ground grid here, so the grid stays put while the joints move.
  double groundZ = 0;

  bool empty() const { return bodies.empty() && frames.empty(); }
  Vec3 center() const { return (min + max) * 0.5; }
  double radius() const { return 0.5 * length(max - min); }
};

/// Joint values for posing a robot: joint name -> radians (revolute, continuous) or
/// metres (prismatic). Joints that are not listed stay at their rest value; names that do
/// not belong to a movable joint are ignored.
using JointPose = std::map<std::string, double>;

/// A joint the user can move: everything that is not fixed.
struct MovableJoint {
  std::string name;
  JointType type = JointType::Revolute;
  double lower = 0, upper = 0;       // continuous joints: -pi .. pi (their values are never clamped)
  std::array<double, 3> axis{{0, 0, 1}};
  /// Joints that hang below the same first movable joint (the three joints of one leg, a
  /// steering joint and its wheel) share a chain number, counted from 0 in order of appearance.
  int chain = 0;
};

/// The movable joints of the fully expanded robot, in the order they appear in the URDF.
std::vector<MovableJoint> movableJoints(const Robot& robot);

/// The value `v` limited to the joint's range (unchanged for continuous joints).
double clampJointValue(const MovableJoint& joint, double v);

/// The joint's own motion for value `q`: a rotation about the axis or a translation along it.
/// The child frame is  parent * origin * jointMotion(joint, q).
Transform jointMotion(const Joint& joint, double q);

/// World-space scene of the robot in the given pose. Values are limited to the joint ranges;
/// the default (an empty pose) is every joint at zero, or at the nearest limit if zero is out of range.
Scene buildScene(const Robot& robot, const JointPose& pose = JointPose());

}  // namespace urdfgen
