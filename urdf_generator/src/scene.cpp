// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include "urdf_generator/scene.hpp"

#include <algorithm>
#include <functional>
#include <map>
#include <set>

namespace urdfgen {

Transform Transform::operator*(const Transform& o) const {
  Transform out;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out.r[static_cast<std::size_t>(3 * i + j)] =
          r[static_cast<std::size_t>(3 * i)] * o.r[static_cast<std::size_t>(j)] +
          r[static_cast<std::size_t>(3 * i + 1)] * o.r[static_cast<std::size_t>(3 + j)] +
          r[static_cast<std::size_t>(3 * i + 2)] * o.r[static_cast<std::size_t>(6 + j)];
    }
  }
  out.t = rotate(o.t) + t;
  return out;
}

Transform Transform::fromOrigin(const Origin& o) {
  const double cr = std::cos(o.roll.value()), sr = std::sin(o.roll.value());
  const double cp = std::cos(o.pitch.value()), sp = std::sin(o.pitch.value());
  const double cy = std::cos(o.yaw.value()), sy = std::sin(o.yaw.value());
  Transform T;
  T.r = {{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,  //
          sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,  //
          -sp,     cp * sr,                cp * cr}};
  T.t = {o.x.value(), o.y.value(), o.z.value()};
  return T;
}

namespace {

std::uint32_t addVertex(Mesh& m, const Vec3& p, const Vec3& n) {
  m.vertices.push_back(p);
  m.normals.push_back(n);
  return static_cast<std::uint32_t>(m.vertices.size() - 1);
}

/// Adds a triangle wound to agree with the vertex normals; skips degenerate ones
/// (the coincident vertices at the poles of a sphere).
void addTriangle(Mesh& m, std::uint32_t a, std::uint32_t b, std::uint32_t c) {
  const Vec3 g = cross(m.vertices[b] - m.vertices[a], m.vertices[c] - m.vertices[a]);
  if (length(g) < 1e-14) return;
  if (dot(g, m.normals[a] + m.normals[b] + m.normals[c]) < 0) std::swap(b, c);
  m.triangles.push_back({{a, b, c}});
}

double& at(Vec3& v, int i) { return i == 0 ? v.x : (i == 1 ? v.y : v.z); }

Mesh box(double sx, double sy, double sz) {
  Mesh m;
  const double half[3] = {sx / 2, sy / 2, sz / 2};
  for (int axis = 0; axis < 3; ++axis) {
    for (int sign = -1; sign <= 1; sign += 2) {
      const int u = (axis + 1) % 3, v = (axis + 2) % 3;
      Vec3 n;
      at(n, axis) = sign;
      std::uint32_t idx[4];
      const int su[4] = {-1, 1, 1, -1}, sv[4] = {-1, -1, 1, 1};
      for (int k = 0; k < 4; ++k) {
        Vec3 p;
        at(p, axis) = sign * half[axis];
        at(p, u) = su[k] * half[u];
        at(p, v) = sv[k] * half[v];
        idx[k] = addVertex(m, p, n);
      }
      addTriangle(m, idx[0], idx[1], idx[2]);
      addTriangle(m, idx[0], idx[2], idx[3]);
    }
  }
  return m;
}

Mesh cylinder(double radius, double length_) {
  constexpr int kSegments = 28;
  Mesh m;
  const double h = length_ / 2;
  std::vector<std::uint32_t> bottom(kSegments), top(kSegments);
  for (int i = 0; i < kSegments; ++i) {
    const double a = 2 * kPi * i / kSegments;
    const Vec3 radial{std::cos(a), std::sin(a), 0};
    bottom[static_cast<std::size_t>(i)] = addVertex(m, {radial.x * radius, radial.y * radius, -h}, radial);
    top[static_cast<std::size_t>(i)] = addVertex(m, {radial.x * radius, radial.y * radius, h}, radial);
  }
  for (int i = 0; i < kSegments; ++i) {
    const std::size_t a = static_cast<std::size_t>(i), b = static_cast<std::size_t>((i + 1) % kSegments);
    addTriangle(m, bottom[a], bottom[b], top[b]);
    addTriangle(m, bottom[a], top[b], top[a]);
  }
  for (int cap = 0; cap < 2; ++cap) {  // cap 0: bottom (normal -z), cap 1: top (normal +z)
    const double z = cap == 0 ? -h : h;
    const Vec3 n{0, 0, cap == 0 ? -1.0 : 1.0};
    const std::uint32_t centre = addVertex(m, {0, 0, z}, n);
    std::vector<std::uint32_t> ring(kSegments);
    for (int i = 0; i < kSegments; ++i) {
      const double a = 2 * kPi * i / kSegments;
      ring[static_cast<std::size_t>(i)] = addVertex(m, {std::cos(a) * radius, std::sin(a) * radius, z}, n);
    }
    for (int i = 0; i < kSegments; ++i) {
      addTriangle(m, centre, ring[static_cast<std::size_t>(i)],
                  ring[static_cast<std::size_t>((i + 1) % kSegments)]);
    }
  }
  return m;
}

Mesh sphere(double radius) {
  constexpr int kRings = 14, kSegments = 28;
  Mesh m;
  std::vector<std::vector<std::uint32_t>> grid(kRings + 1, std::vector<std::uint32_t>(kSegments));
  for (int r = 0; r <= kRings; ++r) {
    const double polar = kPi * r / kRings;
    for (int s = 0; s < kSegments; ++s) {
      const double a = 2 * kPi * s / kSegments;
      const Vec3 n{std::sin(polar) * std::cos(a), std::sin(polar) * std::sin(a), std::cos(polar)};
      grid[static_cast<std::size_t>(r)][static_cast<std::size_t>(s)] = addVertex(m, n * radius, n);
    }
  }
  for (int r = 0; r < kRings; ++r) {
    for (int s = 0; s < kSegments; ++s) {
      const std::size_t s2 = static_cast<std::size_t>((s + 1) % kSegments);
      const auto& row0 = grid[static_cast<std::size_t>(r)];
      const auto& row1 = grid[static_cast<std::size_t>(r + 1)];
      const std::size_t s1 = static_cast<std::size_t>(s);
      addTriangle(m, row0[s1], row1[s1], row1[s2]);
      addTriangle(m, row0[s1], row1[s2], row0[s2]);
    }
  }
  return m;
}

}  // namespace

Mesh tessellate(const Geometry& g) {
  switch (g.kind) {
    case Geometry::Kind::Box: return box(g.a.value(), g.b.value(), g.c.value());
    case Geometry::Kind::Cylinder: return cylinder(g.a.value(), g.b.value());
    case Geometry::Kind::Sphere: return sphere(g.a.value());
  }
  return Mesh();
}

namespace {

double rangeLow(const Joint& j) { return j.type == JointType::Continuous ? -kPi : j.lower.value(); }
double rangeHigh(const Joint& j) { return j.type == JointType::Continuous ? kPi : j.upper.value(); }

bool isMovable(const Joint& j) { return j.type != JointType::Fixed; }

double clampToRange(const Joint& j, double v) {
  if (j.type == JointType::Fixed) return 0.0;
  if (j.type == JointType::Continuous) return v;
  const double lo = j.lower.value(), hi = j.upper.value();
  return lo > hi ? v : std::max(lo, std::min(hi, v));  // an inverted range is a validation error elsewhere
}

/// Pose of every link (in the frame of the root link) for the clamped joint values.
std::map<std::string, Transform> solvePoses(const Part& part, const JointPose& pose) {
  std::map<std::string, std::vector<const Joint*>> kids;
  std::set<std::string> children;
  for (const Joint& j : part.joints) {
    kids[j.parent].push_back(&j);
    children.insert(j.child);
  }
  std::map<std::string, Transform> poses;
  std::function<void(const std::string&, const Transform&)> visit =
      [&](const std::string& link, const Transform& T) {
        if (!poses.emplace(link, T).second) return;  // guards against cycles
        for (const Joint* j : kids[link]) {
          double q = 0.0;
          const auto it = pose.find(j->name);
          if (it != pose.end()) q = it->second;
          visit(j->child, T * Transform::fromOrigin(j->origin) * jointMotion(*j, clampToRange(*j, q)));
        }
      };
  for (const Link& l : part.links) {
    if (!children.count(l.name)) visit(l.name, Transform());
  }
  return poses;
}

}  // namespace

double clampJointValue(const MovableJoint& joint, double v) {
  if (joint.type == JointType::Continuous) return v;
  return joint.lower > joint.upper ? v : std::max(joint.lower, std::min(joint.upper, v));
}

Transform jointMotion(const Joint& joint, double q) {
  Transform T;
  if (joint.type == JointType::Fixed) return T;
  const Vec3 axis = normalized({joint.axis[0], joint.axis[1], joint.axis[2]});
  if (length(axis) < 0.5) return T;  // no usable axis: nothing moves
  if (joint.type == JointType::Prismatic) {
    T.t = axis * q;
    return T;
  }
  const double c = std::cos(q), s = std::sin(q), t = 1.0 - c;
  T.r = {{t * axis.x * axis.x + c, t * axis.x * axis.y - s * axis.z, t * axis.x * axis.z + s * axis.y,  //
          t * axis.x * axis.y + s * axis.z, t * axis.y * axis.y + c, t * axis.y * axis.z - s * axis.x,  //
          t * axis.x * axis.z - s * axis.y, t * axis.y * axis.z + s * axis.x, t * axis.z * axis.z + c}};
  return T;
}

std::vector<MovableJoint> movableJoints(const Robot& robot) {
  const Part part = robot.flatten();
  std::map<std::string, const Joint*> parentJoint;  // child link -> the joint that carries it
  for (const Joint& j : part.joints) parentJoint[j.child] = &j;

  std::map<const Joint*, int> chainOf;
  int chains = 0;
  // The chain of a movable joint is the chain of the nearest movable joint above it, if any.
  std::function<int(const Joint&)> chainFor = [&](const Joint& j) -> int {
    const auto known = chainOf.find(&j);
    if (known != chainOf.end()) return known->second;
    chainOf[&j] = -1;  // guards against cycles while we look upwards
    int chain = -1;
    std::string link = j.parent;
    for (std::size_t hops = 0; hops <= part.joints.size(); ++hops) {
      const auto up = parentJoint.find(link);
      if (up == parentJoint.end()) break;
      if (isMovable(*up->second)) {
        chain = chainFor(*up->second);
        break;
      }
      link = up->second->parent;
    }
    if (chain < 0) chain = chains++;
    chainOf[&j] = chain;
    return chain;
  };

  std::vector<MovableJoint> out;
  for (const Joint& j : part.joints) {
    if (!isMovable(j)) continue;
    MovableJoint m;
    m.name = j.name;
    m.type = j.type;
    m.lower = rangeLow(j);
    m.upper = rangeHigh(j);
    m.axis = j.axis;
    m.chain = chainFor(j);
    out.push_back(m);
  }
  return out;
}

Scene buildScene(const Robot& robot, const JointPose& jointPose) {
  Scene scene;
  const Part part = robot.flatten();

  const std::map<std::string, Transform> pose = solvePoses(part, jointPose);
  const std::map<std::string, Transform> rest = solvePoses(part, JointPose());

  std::map<std::string, std::array<float, 3>> colors;
  for (const Material& m : robot.materials) {
    colors[m.name] = {{static_cast<float>(m.r), static_cast<float>(m.g), static_cast<float>(m.b)}};
  }

  auto poseOf = [](const std::map<std::string, Transform>& poses, const std::string& link) {
    const auto it = poses.find(link);
    return it != poses.end() ? it->second : Transform();
  };

  std::set<std::string> sensorLinks;
  for (const SensorMark& m : robot.sensorMarks) sensorLinks.insert(m.link);

  bool haveGround = false;
  double groundZ = 0.0;
  for (const Link& l : part.links) {
    const Transform linkPose = poseOf(pose, l.name);
    scene.frames.push_back({l.name, linkPose});
    if (!l.hasBody) continue;
    const bool isSensor = sensorLinks.count(l.name) > 0;

    SceneBody body;
    body.link = l.name;
    body.mesh = tessellate(l.geometry);

    // The lowest point of the rest pose, for the ground grid.
    // (A sensor does not count: moving one must not move the ground.)
    if (!isSensor) {
      const Transform restWorld = poseOf(rest, l.name) * Transform::fromOrigin(l.origin);
      for (const Vec3& v : body.mesh.vertices) {
        const double z = restWorld.apply(v).z;
        if (!haveGround || z < groundZ) groundZ = z;
        haveGround = true;
      }
    }

    const Transform world = linkPose * Transform::fromOrigin(l.origin);
    for (Vec3& v : body.mesh.vertices) v = world.apply(v);
    for (Vec3& n : body.mesh.normals) n = normalized(world.rotate(n));
    const auto c = colors.find(l.material);
    if (c != colors.end()) body.color = c->second;
    scene.bodies.push_back(std::move(body));
  }

  for (const SensorMark& m : robot.sensorMarks) {
    SceneSensor s;
    s.name = m.name;
    s.type = m.type;
    s.link = m.link;
    s.parent = m.parent;
    s.pose = poseOf(pose, m.link);
    s.root = poseOf(pose, m.parent).t;
    for (const SceneBody& b : scene.bodies) {
      if (b.link == m.link) s.color = b.color;
    }
    scene.sensors.push_back(std::move(s));
  }

  // Bounds.
  bool first = true;
  auto grow = [&](const Vec3& p) {
    if (first) {
      scene.min = scene.max = p;
      first = false;
      return;
    }
    scene.min = {std::min(scene.min.x, p.x), std::min(scene.min.y, p.y), std::min(scene.min.z, p.z)};
    scene.max = {std::max(scene.max.x, p.x), std::max(scene.max.y, p.y), std::max(scene.max.z, p.z)};
  };
  for (const SceneBody& b : scene.bodies) {
    for (const Vec3& v : b.mesh.vertices) grow(v);
  }
  if (first) {
    for (const SceneFrame& f : scene.frames) grow(f.pose.t);
  }
  if (!haveGround) {  // no bodies: use the lowest rest-pose frame
    bool any = false;
    for (const SceneFrame& f : scene.frames) {
      const double z = poseOf(rest, f.link).t.z;
      if (!any || z < groundZ) groundZ = z;
      any = true;
    }
  }
  scene.groundZ = groundZ;
  return scene;
}

}  // namespace urdfgen
