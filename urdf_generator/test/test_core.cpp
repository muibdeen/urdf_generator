// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// Self-contained tests for the core library (no test framework needed).
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "urdf_generator/builders.hpp"
#include "urdf_generator/emitter.hpp"
#include "urdf_generator/render.hpp"
#include "urdf_generator/scene.hpp"
#include "urdf_generator/sensors.hpp"

using namespace urdfgen;

static int g_checks = 0;
static int g_failed = 0;

#define CHECK(cond)                                                          \
  do {                                                                       \
    ++g_checks;                                                              \
    if (!(cond)) {                                                           \
      ++g_failed;                                                            \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);   \
    }                                                                        \
  } while (0)

#define CHECK_NEAR(a, b)                                                                      \
  do {                                                                                        \
    ++g_checks;                                                                               \
    const double va = (a), vb = (b);                                                          \
    if (!(std::fabs(va - vb) < 1e-9)) {                                                       \
      ++g_failed;                                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s = %.12g, expected %.12g\n", __FILE__, __LINE__,    \
                   #a, va, vb);                                                               \
    }                                                                                         \
  } while (0)

#define CHECK_TOL(a, b, tol)                                                                  \
  do {                                                                                        \
    ++g_checks;                                                                               \
    const double va = (a), vb = (b);                                                          \
    if (!(std::fabs(va - vb) <= (tol))) {                                                     \
      ++g_failed;                                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s = %.9g, expected %.9g (+/- %g)\n", __FILE__,       \
                   __LINE__, #a, va, vb, static_cast<double>(tol));                           \
    }                                                                                         \
  } while (0)

namespace {

std::size_t countOf(const std::string& text, const std::string& needle) {
  std::size_t n = 0, pos = 0;
  while ((pos = text.find(needle, pos)) != std::string::npos) {
    ++n;
    pos += needle.size();
  }
  return n;
}

// Minimal XML well-formedness check: balanced, correctly nested tags.
bool wellFormed(const std::string& s) {
  std::vector<std::string> stack;
  std::size_t i = 0;
  while ((i = s.find('<', i)) != std::string::npos) {
    if (s.compare(i, 4, "<!--") == 0) {
      const std::size_t end = s.find("-->", i + 4);
      if (end == std::string::npos) return false;
      i = end + 3;
      continue;
    }
    if (s.compare(i, 2, "<?") == 0) {
      const std::size_t end = s.find("?>", i);
      if (end == std::string::npos) return false;
      i = end + 2;
      continue;
    }
    const std::size_t end = s.find('>', i);
    if (end == std::string::npos) return false;
    std::string tag = s.substr(i + 1, end - i - 1);
    const bool closing = !tag.empty() && tag[0] == '/';
    const bool selfClosing = !tag.empty() && tag.back() == '/';
    if (closing) tag.erase(0, 1);
    if (selfClosing) tag.pop_back();
    const std::string name = tag.substr(0, tag.find_first_of(" \t\n"));
    if (closing) {
      if (stack.empty() || stack.back() != name) return false;
      stack.pop_back();
    } else if (!selfClosing) {
      stack.push_back(name);
    }
    i = end + 1;
  }
  return stack.empty();
}

const Link* findLink(const Part& p, const std::string& name) {
  for (const auto& l : p.links) {
    if (l.name == name) return &l;
  }
  return nullptr;
}

const Joint* findJoint(const Part& p, const std::string& name) {
  for (const auto& j : p.joints) {
    if (j.name == name) return &j;
  }
  return nullptr;
}

// ------------------------------------------------------------------ tests

void testNum() {
  const Num a = Num::param("a", 2.0), b = Num::param("b", 3.0);
  CHECK((a * 3).expr() == "a*3");
  CHECK_NEAR((a * 3).value(), 6.0);
  CHECK((a * 3).xacro() == "${a*3}");
  CHECK((a * 3).urdf() == "6");
  CHECK(((a + 1) * 2).expr() == "(a+1)*2");
  CHECK((a - (b - 1)).expr() == "a-(b-1)");
  CHECK((a / (b * 2)).expr() == "a/(b*2)");
  CHECK((-(a + b)).expr() == "-(a+b)");
  CHECK((-a / 2).expr() == "-a/2");
  CHECK((a + -1).expr() == "a-1");

  CHECK((Num(2) * Num(3)).isLiteral());
  CHECK_NEAR((Num(2) * Num(3)).value(), 6.0);
  CHECK((a * 1).expr() == "a");
  CHECK((a + 0).expr() == "a");
  CHECK((a * 0).isZeroLiteral());
  CHECK((a / 1).expr() == "a");
  CHECK((0 - a).expr() == "-a");
  CHECK((-(-a)).expr() == "a");   // no "--a"
  CHECK_NEAR((-(-a)).value(), 2.0);
  CHECK((-(-(a + b))).expr() == "(a+b)");
  CHECK_NEAR((-(-(a + b))).value(), 5.0);
  CHECK((-a).expr() == "-a");
  CHECK_NEAR(nacos(Num(0.5)).value(), kPi / 3);
  CHECK(nacos(a / 4).expr() == "acos(a/4)");
  CHECK_NEAR(nacos(Num(1.0 + 1e-12)).value(), 0.0);  // rounding noise outside [-1, 1] is harmless

  const Num d60 = Num::deg(60);
  CHECK_NEAR(d60.value(), kPi / 3);
  CHECK(d60.expr() == "radians(60)");
  CHECK(ncos(d60).expr() == "cos(radians(60))");
  CHECK_NEAR(ncos(d60).value(), 0.5);
  CHECK(Num::deg(0).isZeroLiteral());
  CHECK(ncos(Num::deg(0)).urdf() == "1");
  CHECK_NEAR(nsin(Num::deg(180)).value(), 0.0);
  CHECK(natan2(a, b).expr() == "atan2(a,b)");
  CHECK_NEAR(nsqrt(a * a).value(), 2.0);
  CHECK((Num::pi() / 2).expr() == "pi/2");

  CHECK(Num::format(1e-15) == "0");
  CHECK(Num::format(0.05) == "0.05");
  CHECK(Num::format(-0.0) == "0");
  CHECK(Num::format(1.0) == "1");
}

void testInertia() {
  Link box;
  box.mass = Num(12.0);
  box.geometry = Geometry::box(1, 1, 1);
  CHECK_NEAR(computeInertia(box).ixx, 2.0);

  Link cyl;
  cyl.mass = Num(2.0);
  cyl.geometry = Geometry::cylinder(0.5, 1.0);
  CHECK_NEAR(computeInertia(cyl).izz, 2.0 * 0.25 / 2.0);
  CHECK_NEAR(computeInertia(cyl).ixx, 2.0 * (3 * 0.25 + 1.0) / 12.0);

  Link sph;
  sph.mass = Num(5.0);
  sph.geometry = Geometry::sphere(1.0);
  CHECK_NEAR(computeInertia(sph).iyy, 2.0);
}

struct Expected {
  RobotType type;
  std::size_t links, joints;
};

void testDefaults() {
  const std::vector<Expected> expected = {
      {RobotType::Hexapod, 25, 24},  {RobotType::Arm, 5, 4},        {RobotType::DiffDrive, 6, 5},
      {RobotType::FourWheel, 6, 5},  {RobotType::Ackermann, 8, 7},  {RobotType::Quadruped, 17, 16},
      {RobotType::Biped, 7, 6},
  };
  CHECK(expected.size() == allRobotTypes().size());
  for (const auto& e : expected) {
    const Robot r = buildDefault(e.type);
    const std::string id = robotTypeId(e.type);
    const auto issues = validate(r);
    for (const auto& i : issues) std::fprintf(stderr, "  [%s] %s\n", id.c_str(), i.message.c_str());
    CHECK(!hasErrors(issues));
    const Part p = r.flatten();
    if (p.links.size() != e.links || p.joints.size() != e.joints) {
      std::fprintf(stderr, "  [%s] links %zu (want %zu), joints %zu (want %zu)\n", id.c_str(),
                   p.links.size(), e.links, p.joints.size(), e.joints);
    }
    CHECK(p.links.size() == e.links);
    CHECK(p.joints.size() == e.joints);
    CHECK(p.links.size() == p.joints.size() + 1);  // a tree

    const std::string urdf = emitUrdf(r);
    const std::string xacro = emitXacro(r);
    CHECK(wellFormed(urdf));
    CHECK(wellFormed(xacro));
    CHECK(urdf.find("${") == std::string::npos);
    CHECK(urdf.find("xacro") == std::string::npos);
    CHECK(xacro.find("xmlns:xacro=\"http://www.ros.org/wiki/xacro\"") != std::string::npos);
    CHECK(countOf(xacro, "<xacro:property ") > 5);
    CHECK(countOf(xacro, "<link name=") <= countOf(urdf, "<link name="));
    CHECK(countOf(urdf, "<link name=") == e.links);
    CHECK(countOf(urdf, "<joint name=") == e.joints);
    CHECK(countOf(urdf, "<inertial>") > 0);
    CHECK(countOf(urdf, "<collision>") == countOf(urdf, "<visual>"));
  }
}

// Every material that is defined is used by some link, and every material a link
// uses is defined (an undefined material name makes RViz and Gazebo complain).
void testMaterialsUsed() {
  auto check = [](const Robot& r) {
    std::set<std::string> defined, used;
    for (const auto& m : r.materials) defined.insert(m.name);
    for (const auto& l : r.flatten().links) {
      if (l.hasBody && !l.material.empty()) used.insert(l.material);
    }
    if (defined != used) {
      std::fprintf(stderr, "  [%s] materials defined and used differ\n", r.name.c_str());
    }
    CHECK(defined == used);
  };
  for (RobotType t : allRobotTypes()) check(buildDefault(t));
  DiffDriveConfig noCasters;
  noCasters.casters = false;
  check(buildDiffDrive(noCasters));
  DiffDriveConfig mecLeft;
  mecLeft.leftWheel = WheelKind::Mecanum;
  check(buildDiffDrive(mecLeft));
  FourWheelConfig mecFour;
  mecFour.rearRightWheel = WheelKind::Mecanum;
  check(buildFourWheel(mecFour));
  AckermannConfig mecAck;
  mecAck.frontLeftWheel = WheelKind::Mecanum;
  check(buildAckermann(mecAck));
}

void testMacrosInXacro() {
  const Robot hex = buildDefault(RobotType::Hexapod);
  const std::string x = emitXacro(hex);
  CHECK(x.find("<xacro:macro name=\"hexapod_leg\"") != std::string::npos);
  CHECK(countOf(x, "<xacro:hexapod_leg ") == 6);
  CHECK(x.find("<xacro:macro name=\"cylinder_inertia\"") != std::string::npos);
  CHECK(x.find("<xacro:macro name=\"box_inertia\"") == std::string::npos);  // unused, not emitted
  CHECK(x.find("mount_x=\"${body_radius*cos(radians(60))}\"") != std::string::npos);
  CHECK(x.find("<xacro:property name=\"coxa_length\" value=\"0.06\"/>") != std::string::npos);

  const Robot quad = buildDefault(RobotType::Quadruped);
  CHECK(countOf(emitXacro(quad), "<xacro:quadruped_leg ") == 4);
  const Robot diff = buildDefault(RobotType::DiffDrive);
  CHECK(countOf(emitXacro(diff), "<xacro:drive_wheel ") == 2);
  CHECK(countOf(emitXacro(diff), "<xacro:caster ") == 2);
  const Robot ack = buildDefault(RobotType::Ackermann);
  CHECK(countOf(emitXacro(ack), "<xacro:steered_wheel ") == 2);
  CHECK(countOf(emitXacro(ack), "<xacro:drive_wheel ") == 2);
}

void testHexapodGeometry() {
  HexapodConfig c;
  c.coxaLength = 0.07;
  c.femurLength = 0.11;
  c.tibiaLength = 0.17;
  c.bodyRadius = 0.2;
  const Part p = buildHexapod(c).flatten();
  const Joint* j2 = findJoint(p, "leg2_coxa_joint");
  CHECK(j2 != nullptr);
  if (j2) {
    CHECK_NEAR(j2->origin.x.value(), 0.2 * std::cos(kPi / 3));
    CHECK_NEAR(j2->origin.y.value(), 0.2 * std::sin(kPi / 3));
    CHECK_NEAR(j2->origin.yaw.value(), kPi / 3);
  }
  const Joint* fem = findJoint(p, "leg1_femur_joint");
  CHECK(fem != nullptr);
  if (fem) CHECK_NEAR(fem->origin.x.value(), 0.07);
  const Link* tib = findLink(p, "leg4_tibia_link");
  CHECK(tib != nullptr);
  if (tib) CHECK_NEAR(tib->geometry.b.value(), 0.17);
  const Joint* cox = findJoint(p, "leg3_coxa_joint");
  CHECK(cox != nullptr);
  if (cox) CHECK_NEAR(cox->upper.value(), 45 * kPi / 180);
}

void testArm() {
  ArmConfig c;
  c.links.clear();
  ArmLinkConfig polar;
  polar.mode = OffsetMode::Polar;
  polar.a = 0.1;   // r
  polar.b = 90;    // theta, degrees
  polar.c = 0.2;   // z
  polar.axis = ArmAxis::X;
  c.links.push_back(polar);
  ArmLinkConfig slide;
  slide.type = ArmJointType::Prismatic;
  slide.axis = ArmAxis::Z;
  slide.a = 0;
  slide.b = 0;
  slide.c = 0.15;
  slide.lower = 0;
  slide.upper = 0.3;
  c.links.push_back(slide);
  const Robot r = buildArm(c);
  CHECK(!hasErrors(validate(r)));
  const Part p = r.flatten();
  const Joint* j1 = findJoint(p, "joint_1");
  const Joint* j2 = findJoint(p, "joint_2");
  CHECK(j1 && j2);
  if (j1 && j2) {
    CHECK_NEAR(j1->origin.x.value(), 0.0);
    CHECK_NEAR(j1->origin.y.value(), 0.1);
    CHECK_NEAR(j1->origin.z.value(), 0.2);
    CHECK(j1->type == JointType::Revolute);
    CHECK(j2->type == JointType::Prismatic);
    CHECK_NEAR(j2->upper.value(), 0.3);  // metres, not converted
    CHECK_NEAR(j1->lower.value(), -170 * kPi / 180);
    CHECK(j1->axis[0] == 1.0 && j1->axis[2] == 0.0);
  }
  const std::string x = emitXacro(r);
  CHECK(x.find("joint1_theta") != std::string::npos);
  CHECK(x.find("<origin xyz=\"${joint1_r*cos(joint1_theta)} ${joint1_r*sin(joint1_theta)} ${joint1_z}\"") !=
        std::string::npos);

  // Link 1 runs from joint 1 to joint 2: straight up (z) by 0.15.
  const Link* l1 = findLink(p, "link_1");
  CHECK(l1 && l1->geometry.kind == Geometry::Kind::Cylinder);
  if (l1) {
    CHECK_NEAR(l1->geometry.b.value(), 0.15);
    CHECK_NEAR(l1->origin.z.value(), 0.075);
    CHECK_NEAR(l1->origin.pitch.value(), 0.0);
  }

  // Coincident joints give a sphere instead of a zero-length cylinder.
  ArmConfig d;
  d.links.clear();
  ArmLinkConfig first, second;
  first.c = 0.1;
  second.a = second.b = second.c = 0;
  second.axis = ArmAxis::Y;
  d.links = {first, second};
  d.toolZ = 0.1;
  const Part q = buildArm(d).flatten();
  const Link* ball = findLink(q, "link_1");
  CHECK(ball && ball->geometry.kind == Geometry::Kind::Sphere);

  // A tilted link: pitch/yaw must point the cylinder at the next joint.
  ArmConfig t;
  t.links.clear();
  ArmLinkConfig a1, a2;
  a1.a = 0; a1.b = 0; a1.c = 0.1;
  a2.a = 0.1; a2.b = 0.1; a2.c = 0.1;  // next joint at (0.1, 0.1, 0.1) in link 1's frame
  t.links = {a1, a2};
  const Part tp = buildArm(t).flatten();
  const Link* tl = findLink(tp, "link_1");
  CHECK(tl != nullptr);
  if (tl) {
    CHECK_NEAR(tl->geometry.b.value(), std::sqrt(0.03));
    const double pitch = tl->origin.pitch.value(), yaw = tl->origin.yaw.value();
    // Rz(yaw) * Ry(pitch) applied to +z:
    const double dx = std::sin(pitch) * std::cos(yaw), dy = std::sin(pitch) * std::sin(yaw),
                 dz = std::cos(pitch);
    const double len = std::sqrt(0.03);
    CHECK_NEAR(dx * len, 0.1);
    CHECK_NEAR(dy * len, 0.1);
    CHECK_NEAR(dz * len, 0.1);
  }

  // No links at all still produces a valid robot (base + tool frame).
  ArmConfig e;
  e.links.clear();
  const Robot er = buildArm(e);
  CHECK(!hasErrors(validate(er)));
  CHECK(findJoint(er.flatten(), "tool_joint")->parent == "base_link");
}

void testWheeled() {
  DiffDriveConfig c;
  c.chassis.shape = ChassisShape::Cylinder;
  c.chassis.radius = 0.2;
  c.wheelRadius = 0.08;
  c.wheelZ = -0.02;
  c.wheelY = 0.15;
  const Robot r = buildDiffDrive(c);
  CHECK(!hasErrors(validate(r)));
  const Part p = r.flatten();
  const Joint* fp = findJoint(p, "base_footprint_joint");
  CHECK(fp != nullptr);
  if (fp) CHECK_NEAR(fp->origin.z.value(), 0.08 + 0.02);  // base_link sits one radius + offset above the ground
  const Joint* lw = findJoint(p, "left_wheel_joint");
  const Joint* rw = findJoint(p, "right_wheel_joint");
  CHECK(lw && rw);
  if (lw && rw) {
    CHECK(lw->type == JointType::Continuous);
    CHECK_NEAR(lw->origin.y.value(), 0.15);
    CHECK_NEAR(rw->origin.y.value(), -0.15);
    CHECK_NEAR(lw->origin.z.value(), -0.02);
  }
  // Casters touch the same ground plane as the wheels.
  const Joint* cj = findJoint(p, "front_caster_joint");
  CHECK(cj != nullptr);
  if (cj) {
    const double groundZ = -0.02 - 0.08;
    CHECK_NEAR(cj->origin.z.value() - c.casterRadius, groundZ);
    CHECK_NEAR(cj->origin.x.value(), 0.2 - c.casterRadius);
  }
  CHECK(findLink(p, "base_link")->geometry.kind == Geometry::Kind::Cylinder);
  CHECK(emitXacro(r).find("chassis_radius") != std::string::npos);

  DiffDriveConfig nc;
  nc.casters = false;
  CHECK(buildDiffDrive(nc).flatten().links.size() == 4);

  const Part f = buildFourWheel({}).flatten();
  CHECK(findJoint(f, "rear_right_wheel_joint") != nullptr);
  CHECK_NEAR(findJoint(f, "rear_right_wheel_joint")->origin.x.value(), -0.10);
  CHECK_NEAR(findJoint(f, "rear_right_wheel_joint")->origin.y.value(), -0.14);

  AckermannConfig ac;
  ac.wheelbase = 0.5;
  ac.trackWidth = 0.4;
  ac.steerLimitDeg = 25;
  const Part a = buildAckermann(ac).flatten();
  const Joint* st = findJoint(a, "front_left_steer_joint");
  CHECK(st != nullptr);
  if (st) {
    CHECK(st->type == JointType::Revolute);
    CHECK_NEAR(st->origin.x.value(), 0.25);
    CHECK_NEAR(st->origin.y.value(), 0.2);
    CHECK_NEAR(st->upper.value(), 25 * kPi / 180);
  }
  CHECK(findJoint(a, "rear_left_wheel_joint")->parent == "base_link");
  CHECK(findJoint(a, "front_left_wheel_joint")->parent == "front_left_steer_link");
  CHECK_NEAR(findJoint(a, "rear_right_wheel_joint")->origin.x.value(), -0.25);
}

void testLegged() {
  const Part q = buildQuadruped({}).flatten();
  const Joint* rr = findJoint(q, "rear_right_coxa_joint");
  CHECK(rr != nullptr);
  if (rr) {
    CHECK_NEAR(rr->origin.x.value(), -0.15);
    CHECK_NEAR(rr->origin.y.value(), -0.10);
  }
  // Right-hand coxa links point to -y, left to +y.
  CHECK(findJoint(q, "front_right_femur_joint")->origin.y.value() < 0);
  CHECK(findJoint(q, "front_left_femur_joint")->origin.y.value() > 0);

  const Part b = buildBiped({}).flatten();
  CHECK_NEAR(findJoint(b, "left_hip_joint")->origin.y.value(), 0.07);
  CHECK_NEAR(findJoint(b, "right_hip_joint")->origin.y.value(), -0.07);
  CHECK_NEAR(findJoint(b, "left_hip_joint")->origin.z.value(), -0.125);
  CHECK_NEAR(findJoint(b, "left_knee_joint")->lower.value(), 0.0);
  CHECK_NEAR(findJoint(b, "left_ankle_joint")->origin.z.value(), -0.20);
}

void testValidation() {
  // Everything wrong at once.
  Robot r("bad name");
  Link a;
  a.name = "a";
  a.geometry = Geometry::box(1, 1, 1);
  a.mass = Num(1.0);
  Link a2 = a;                // duplicate name
  Link b = a;
  b.name = "b";
  b.mass = Num(0.0);          // no mass
  Link c = a;
  c.name = "c";
  c.geometry = Geometry::cylinder(-1, 1);  // negative radius
  r.root.links = {a, a2, b, c};
  Joint j1;
  j1.name = "j1";
  j1.type = JointType::Revolute;
  j1.parent = "a";
  j1.child = "ghost";         // unknown link
  j1.lower = Num(1.0);
  j1.upper = Num(0.0);        // lower > upper; effort and velocity are zero
  Joint j2 = j1;              // duplicate joint name
  j2.child = "b";
  r.root.joints = {j1, j2};
  const auto issues = validate(r);
  CHECK(hasErrors(issues));
  auto has = [&](const std::string& needle) {
    for (const auto& i : issues) {
      if (i.message.find(needle) != std::string::npos) return true;
    }
    return false;
  };
  CHECK(has("Robot name"));
  CHECK(has("Duplicate link name 'a'"));
  CHECK(has("Duplicate joint name 'j1'"));
  CHECK(has("needs a mass"));
  CHECK(has("non-positive dimension"));
  CHECK(has("unknown child link 'ghost'"));
  CHECK(has("lower limit < upper limit"));
  CHECK(has("effort"));
  CHECK(has("velocity"));
  CHECK(has("root link"));

  // A cycle: a -> b -> a (no root at all).
  Robot cyc("cyc");
  Link x = a, y = a;
  x.name = "x";
  y.name = "y";
  cyc.root.links = {x, y};
  Joint f1, f2;
  f1.name = "f1"; f1.parent = "x"; f1.child = "y";
  f2.name = "f2"; f2.parent = "y"; f2.child = "x";
  cyc.root.joints = {f1, f2};
  CHECK(hasErrors(validate(cyc)));

  // Two disconnected pieces.
  Robot two("two");
  Link p = a, q = a;
  p.name = "p";
  q.name = "q";
  two.root.links = {p, q};
  CHECK(hasErrors(validate(two)));

  // A zero-length revolute axis.
  Robot ax("ax");
  Link p1 = a, p2 = a;
  p1.name = "p1";
  p2.name = "p2";
  ax.root.links = {p1, p2};
  Joint jj;
  jj.name = "jj"; jj.type = JointType::Continuous; jj.parent = "p1"; jj.child = "p2";
  jj.axis = {{0, 0, 0}};
  ax.root.joints = {jj};
  CHECK(hasErrors(validate(ax)));

  // Bad user input is reported, not crashed on.
  DiffDriveConfig bad;
  bad.wheelRadius = -1;
  CHECK(hasErrors(validate(buildDiffDrive(bad))));
  HexapodConfig badHex;
  badHex.coxaLimitDeg = 0;  // lower == upper
  CHECK(hasErrors(validate(buildHexapod(badHex))));
  HexapodConfig noName;
  noName.name = "";
  CHECK(hasErrors(validate(buildHexapod(noName))));

  // Duplicate property names are a programming error.
  Robot dup("dup");
  dup.prop("x", 1.0);
  bool threw = false;
  try {
    dup.prop("x", 2.0);
  } catch (const std::logic_error&) {
    threw = true;
  }
  CHECK(threw);
}

void testUrdfValues() {
  // The flat URDF carries resolved numbers.
  const std::string u = emitUrdf(buildDefault(RobotType::Hexapod));
  CHECK(u.find("<cylinder radius=\"0.008\" length=\"0.06\"/>") != std::string::npos);
  CHECK(u.find("<joint name=\"leg1_coxa_joint\" type=\"revolute\">") != std::string::npos);
  CHECK(u.find("<limit lower=\"-0.78539816\" upper=\"0.78539816\" effort=\"5\" velocity=\"3\"/>") !=
        std::string::npos);
  CHECK(u.find("<material name=\"leg_orange\">") != std::string::npos);
  CHECK(u.find("<mass value=\"0.05\"/>") != std::string::npos);
  CHECK(u.find("<inertia ") != std::string::npos);
  CHECK(u.find("<robot name=\"hexapod\">") != std::string::npos);
}


// ---------------------------------------------------------------- 3D scene

void testTessellation() {
  auto check = [](const Geometry& g, double expectedVolume, double relTol) {
    const Mesh m = tessellate(g);
    CHECK(!m.triangles.empty());
    CHECK(m.vertices.size() == m.normals.size());
    bool outward = true, unit = true, inRange = true;
    double volume = 0;
    for (const auto& t : m.triangles) {
      if (t[0] >= m.vertices.size() || t[1] >= m.vertices.size() || t[2] >= m.vertices.size()) {
        inRange = false;
        continue;
      }
      const Vec3 &a = m.vertices[t[0]], &b = m.vertices[t[1]], &c = m.vertices[t[2]];
      // The winding must agree with the normals, and the signed volume must come out positive.
      if (dot(cross(b - a, c - a), m.normals[t[0]] + m.normals[t[1]] + m.normals[t[2]]) <= 0) outward = false;
      volume += dot(a, cross(b, c)) / 6.0;
    }
    for (const Vec3& n : m.normals) {
      if (std::fabs(length(n) - 1.0) > 1e-9) unit = false;
    }
    CHECK(inRange);
    CHECK(outward);
    CHECK(unit);
    CHECK_TOL(volume, expectedVolume, expectedVolume * relTol);
  };
  check(Geometry::box(0.3, 0.2, 0.1), 0.006, 1e-9);
  const double r = 0.05, h = 0.2;
  check(Geometry::cylinder(r, h), 0.5 * 28 * r * r * std::sin(2 * kPi / 28) * h, 1e-9);
  check(Geometry::sphere(0.1), 4.0 / 3.0 * kPi * 0.001, 0.05);  // an inscribed polyhedron
}

const SceneFrame* frameOf(const Scene& s, const std::string& link) {
  for (const auto& f : s.frames) {
    if (f.link == link) return &f;
  }
  return nullptr;
}

void testTransforms() {
  Origin yaw90(0, 0, 0, 0, 0, Num(kPi / 2));
  Vec3 v = Transform::fromOrigin(yaw90).rotate({1, 0, 0});
  CHECK_NEAR(v.x, 0.0); CHECK_NEAR(v.y, 1.0); CHECK_NEAR(v.z, 0.0);
  Origin roll90(0, 0, 0, Num(kPi / 2), 0, 0);
  v = Transform::fromOrigin(roll90).rotate({0, 1, 0});
  CHECK_NEAR(v.x, 0.0); CHECK_NEAR(v.y, 0.0); CHECK_NEAR(v.z, 1.0);
  Origin pitch90(0, 0, 0, 0, Num(kPi / 2), 0);
  v = Transform::fromOrigin(pitch90).rotate({0, 0, 1});
  CHECK_NEAR(v.x, 1.0); CHECK_NEAR(v.y, 0.0); CHECK_NEAR(v.z, 0.0);

  // Composition: (A * B).apply(p) == A.apply(B.apply(p)); URDF order is Rz * Ry * Rx.
  const Transform A = Transform::fromOrigin(Origin(0.1, 0.2, 0.3, 0.4, 0.5, 0.6));
  const Transform B = Transform::fromOrigin(Origin(-0.3, 0.7, 0.2, -0.2, 0.9, 0.1));
  const Vec3 p{0.5, -0.4, 0.8};
  const Vec3 lhs = (A * B).apply(p), rhs = A.apply(B.apply(p));
  CHECK_NEAR(lhs.x, rhs.x); CHECK_NEAR(lhs.y, rhs.y); CHECK_NEAR(lhs.z, rhs.z);
  const Origin rpy(0, 0, 0, 0.4, 0.5, 0.6);
  const Vec3 w = Transform::fromOrigin(rpy).rotate({0.3, 0.2, 0.1});
  // Reference: rotate about x by roll, then y by pitch, then z by yaw (all about fixed axes).
  auto rx = [](Vec3 u, double a) { return Vec3{u.x, u.y * std::cos(a) - u.z * std::sin(a), u.y * std::sin(a) + u.z * std::cos(a)}; };
  auto ry = [](Vec3 u, double a) { return Vec3{u.x * std::cos(a) + u.z * std::sin(a), u.y, -u.x * std::sin(a) + u.z * std::cos(a)}; };
  auto rz = [](Vec3 u, double a) { return Vec3{u.x * std::cos(a) - u.y * std::sin(a), u.x * std::sin(a) + u.y * std::cos(a), u.z}; };
  const Vec3 ref = rz(ry(rx({0.3, 0.2, 0.1}, 0.4), 0.5), 0.6);
  CHECK_NEAR(w.x, ref.x); CHECK_NEAR(w.y, ref.y); CHECK_NEAR(w.z, ref.z);
}

void testScene() {
  // Biped: both feet hang straight below the hips.
  const Scene bip = buildScene(buildDefault(RobotType::Biped));
  const SceneFrame* lf = frameOf(bip, "left_foot_link");
  const SceneFrame* rf = frameOf(bip, "right_foot_link");
  CHECK(lf && rf);
  if (lf && rf) {
    CHECK_NEAR(lf->pose.t.x, 0.0); CHECK_NEAR(lf->pose.t.y, 0.07); CHECK_NEAR(lf->pose.t.z, -0.525);
    CHECK_NEAR(rf->pose.t.y, -0.07);
  }
  CHECK(bip.bodies.size() == 7);  // torso, 2 x (thigh, shank, foot)

  // Hexapod: leg 2 is rotated 60 degrees; the foot of leg 1 is at the end of coxa + femur.
  const Scene hex = buildScene(buildDefault(RobotType::Hexapod));
  const SceneFrame* leg2 = frameOf(hex, "leg2_coxa_link");
  CHECK(leg2 != nullptr);
  if (leg2) {
    CHECK_NEAR(leg2->pose.t.x, 0.12 * std::cos(kPi / 3)); CHECK_NEAR(leg2->pose.t.y, 0.12 * std::sin(kPi / 3));
    CHECK_NEAR(leg2->pose.r[0], std::cos(kPi / 3)); CHECK_NEAR(leg2->pose.r[3], std::sin(kPi / 3));
  }
  const SceneFrame* foot1 = frameOf(hex, "leg1_foot_link");
  CHECK(foot1 != nullptr);
  if (foot1) { CHECK_NEAR(foot1->pose.t.x, 0.28); CHECK_NEAR(foot1->pose.t.z, -0.15); }

  // Wheeled robots stand on the ground plane z = 0 and have no body for base_footprint.
  const Scene dd = buildScene(buildDefault(RobotType::DiffDrive));
  CHECK_TOL(dd.min.z, 0.0, 1e-9);
  CHECK(dd.bodies.size() == 5);
  CHECK(dd.frames.size() == 6);
  CHECK_NEAR(frameOf(dd, "base_link")->pose.t.z, 0.08);
  for (const auto& b : dd.bodies) {
    if (b.link == "base_link") {
      CHECK_NEAR(b.color[0], 0.50f); CHECK_NEAR(b.color[1], 0.55f); CHECK_NEAR(b.color[2], 0.62f);
    }
  }
  CHECK(dd.min.x < dd.max.x && dd.min.y < dd.max.y && dd.min.z < dd.max.z);
  CHECK(dd.radius() > 0.1);

  // Arm: the tool frame is at the end of the chain.
  const Scene arm = buildScene(buildDefault(RobotType::Arm));
  CHECK_NEAR(frameOf(arm, "tool0")->pose.t.z, 0.5);

  // Cylinders lie where the builder put them: the hexapod coxa runs from the hip along +x.
  for (const auto& b : hex.bodies) {
    if (b.link == "leg1_coxa_link") {
      double minX = 1e9, maxX = -1e9;
      for (const Vec3& v : b.mesh.vertices) { minX = std::min(minX, v.x); maxX = std::max(maxX, v.x); }
      CHECK_NEAR(minX, 0.12); CHECK_NEAR(maxX, 0.18);
    }
  }

  // A robot made only of frames still has sensible bounds.
  Robot bare("bare");
  Link f;
  f.name = "only_frame";
  f.hasBody = false;
  bare.root.links.push_back(f);
  const Scene bs = buildScene(bare);
  CHECK(bs.bodies.empty());
  CHECK(bs.frames.size() == 1);
  CHECK_NEAR(bs.radius(), 0.0);

  // Every default robot yields a non-empty scene with finite bounds.
  for (RobotType t : allRobotTypes()) {
    const Scene s = buildScene(buildDefault(t));
    CHECK(!s.bodies.empty());
    CHECK(std::isfinite(s.radius()) && s.radius() > 0.05);
  }
}

// ---------------------------------------------------------------------- poses

/// A deterministic pose that moves every joint to somewhere inside its range, so that
/// the numbers are identical when the Python test compares them with another URDF library.
JointPose demoPose(const Robot& r) {
  JointPose pose;
  int i = 0;
  for (const MovableJoint& m : movableJoints(r)) {
    if (m.type == JointType::Continuous) {
      pose[m.name] = 0.5 * (i + 1);
    } else {
      const double frac = 0.25 + 0.5 * ((i * 7) % 5) / 4.0;
      pose[m.name] = m.lower + (m.upper - m.lower) * frac;
    }
    ++i;
  }
  return pose;
}

const SceneBody* bodyOf(const Scene& s, const std::string& link) {
  for (const auto& b : s.bodies) {
    if (b.link == link) return &b;
  }
  return nullptr;
}

void testMovableJoints() {
  struct Expect { RobotType type; std::size_t joints; int chains; };
  const Expect expected[] = {{RobotType::Hexapod, 18, 6},   {RobotType::Quadruped, 12, 4},
                             {RobotType::Biped, 6, 2},      {RobotType::DiffDrive, 2, 2},
                             {RobotType::FourWheel, 4, 4},  {RobotType::Ackermann, 6, 4}};
  for (const Expect& e : expected) {
    const std::vector<MovableJoint> mj = movableJoints(buildDefault(e.type));
    CHECK(mj.size() == e.joints);
    int maxChain = -1;
    for (const MovableJoint& m : mj) {
      maxChain = std::max(maxChain, m.chain);
      CHECK(m.type != JointType::Fixed);
      CHECK(m.lower < m.upper);
      CHECK_NEAR(length({m.axis[0], m.axis[1], m.axis[2]}), 1.0);
    }
    CHECK(maxChain + 1 == e.chains);
  }

  // The three joints of one hexapod leg share a chain; the next leg starts a new one.
  const std::vector<MovableJoint> hex = movableJoints(buildDefault(RobotType::Hexapod));
  CHECK(hex[0].name == "leg1_coxa_joint" && hex[1].name == "leg1_femur_joint" && hex[2].name == "leg1_tibia_joint");
  CHECK(hex[0].chain == 0 && hex[1].chain == 0 && hex[2].chain == 0);
  CHECK(hex[3].name == "leg2_coxa_joint" && hex[3].chain == 1);
  CHECK(hex[17].chain == 5);
  for (const MovableJoint& m : hex) CHECK(m.name.find("foot") == std::string::npos);  // fixed joints are left out

  // Ackermann: a steering joint and its wheel are one chain; the rear wheels are chains of their own.
  const std::vector<MovableJoint> ack = movableJoints(buildDefault(RobotType::Ackermann));
  std::map<std::string, int> chainOfJoint;
  for (const MovableJoint& m : ack) chainOfJoint[m.name] = m.chain;
  CHECK(chainOfJoint.at("front_left_steer_joint") == chainOfJoint.at("front_left_wheel_joint"));
  CHECK(chainOfJoint.at("front_left_steer_joint") != chainOfJoint.at("front_right_steer_joint"));
  CHECK(chainOfJoint.at("rear_left_wheel_joint") != chainOfJoint.at("rear_right_wheel_joint"));
  for (const MovableJoint& m : ack) {
    if (m.type == JointType::Continuous) {
      CHECK_NEAR(m.lower, -kPi);
      CHECK_NEAR(m.upper, kPi);
    }
  }
  CHECK(clampJointValue(ack[0], 1e6) <= ack[0].upper + 1e-12 || ack[0].type == JointType::Continuous);

  // Arms: one joint per link, in order, all of one chain.
  const std::vector<MovableJoint> arm = movableJoints(buildDefault(RobotType::Arm));
  CHECK(!arm.empty() && arm[0].name == "joint_1");
  for (const MovableJoint& m : arm) CHECK(m.chain == 0);
}

void testJointMotion() {
  Joint j;
  j.type = JointType::Revolute;
  j.axis = {{0, 0, 1}};
  const Transform quarter = jointMotion(j, kPi / 2);
  const Vec3 p = quarter.apply({1, 0, 0});
  CHECK_NEAR(p.x, 0.0); CHECK_NEAR(p.y, 1.0); CHECK_NEAR(p.z, 0.0);

  // The axis is normalised, like URDF parsers do; a zero axis moves nothing.
  j.axis = {{0, 0, 3}};
  CHECK_NEAR(jointMotion(j, kPi / 2).apply({1, 0, 0}).y, 1.0);
  j.axis = {{0, 0, 0}};
  CHECK_NEAR(jointMotion(j, 1.0).apply({1, 2, 3}).x, 1.0);

  // An arbitrary axis gives a proper rotation that leaves the axis alone.
  j.axis = {{1, 2, -2}};
  const Transform t = jointMotion(j, 0.8);
  const Vec3 axis = normalized({1, 2, -2});
  const Vec3 fixedPoint = t.rotate(axis);
  CHECK_NEAR(fixedPoint.x, axis.x); CHECK_NEAR(fixedPoint.y, axis.y); CHECK_NEAR(fixedPoint.z, axis.z);
  for (int a = 0; a < 3; ++a) {
    for (int b = 0; b < 3; ++b) {
      double d = 0;
      for (int k = 0; k < 3; ++k) {
        d += t.r[static_cast<std::size_t>(3 * a + k)] * t.r[static_cast<std::size_t>(3 * b + k)];
      }
      CHECK_NEAR(d, a == b ? 1.0 : 0.0);
    }
  }
  const double det = t.r[0] * (t.r[4] * t.r[8] - t.r[5] * t.r[7]) - t.r[1] * (t.r[3] * t.r[8] - t.r[5] * t.r[6]) +
                     t.r[2] * (t.r[3] * t.r[7] - t.r[4] * t.r[6]);
  CHECK_NEAR(det, 1.0);
  CHECK_NEAR(length(t.t), 0.0);

  // Prismatic joints translate along the (normalised) axis and do not rotate.
  j.type = JointType::Prismatic;
  j.axis = {{0, 0, 2}};
  const Transform slide = jointMotion(j, 0.25);
  CHECK_NEAR(slide.t.z, 0.25); CHECK_NEAR(slide.t.x, 0.0); CHECK_NEAR(slide.r[0], 1.0); CHECK_NEAR(slide.r[8], 1.0);

  // Fixed joints do nothing.
  j.type = JointType::Fixed;
  CHECK_NEAR(jointMotion(j, 5.0).t.z, 0.0);
}

void testPose() {
  const Robot hexRobot = buildDefault(RobotType::Hexapod);
  const Scene rest = buildScene(hexRobot);
  const Scene same = buildScene(hexRobot, JointPose());
  CHECK(rest.bodies.size() == same.bodies.size());
  CHECK_NEAR(rest.min.z, same.min.z);
  CHECK_NEAR(rest.groundZ, rest.min.z);  // with every joint at zero the lowest point is the ground

  // Coxa yaw swings the whole leg about the coxa joint at (0.12, 0, 0).
  const double yaw = 30 * kPi / 180, pitch = 0.3;
  JointPose pose;
  pose["leg1_coxa_joint"] = yaw;
  const Scene turned = buildScene(hexRobot, pose);
  const SceneFrame* foot = frameOf(turned, "leg1_foot_link");
  CHECK(foot != nullptr);
  if (foot) {
    CHECK_NEAR(foot->pose.t.x, 0.12 + 0.16 * std::cos(yaw));
    CHECK_NEAR(foot->pose.t.y, 0.16 * std::sin(yaw));
    CHECK_NEAR(foot->pose.t.z, -0.15);
  }
  // Other legs are untouched.
  CHECK_NEAR(frameOf(turned, "leg2_foot_link")->pose.t.x, frameOf(rest, "leg2_foot_link")->pose.t.x);
  CHECK_NEAR(frameOf(turned, "leg2_foot_link")->pose.t.y, frameOf(rest, "leg2_foot_link")->pose.t.y);

  // Femur pitch about y at (0.18, 0, 0) raises or lowers the foot.
  pose.clear();
  pose["leg1_femur_joint"] = pitch;
  const Scene pitched = buildScene(hexRobot, pose);
  const SceneFrame* foot2 = frameOf(pitched, "leg1_foot_link");
  if (foot2) {
    CHECK_NEAR(foot2->pose.t.x, 0.18 + 0.10 * std::cos(pitch) - 0.15 * std::sin(pitch));
    CHECK_NEAR(foot2->pose.t.z, -0.10 * std::sin(pitch) - 0.15 * std::cos(pitch));
  }
  CHECK(pitched.min.z != rest.min.z);
  CHECK_NEAR(pitched.groundZ, rest.groundZ);  // the ground does not move with the pose
  CHECK_NEAR(turned.groundZ, rest.groundZ);

  // Values beyond the limits are clamped; fixed joints and unknown names are ignored.
  const std::vector<MovableJoint> mj = movableJoints(hexRobot);
  pose.clear();
  pose["leg1_coxa_joint"] = 100.0;
  const Scene over = buildScene(hexRobot, pose);
  pose["leg1_coxa_joint"] = mj[0].upper;
  const Scene atLimit = buildScene(hexRobot, pose);
  CHECK_NEAR(frameOf(over, "leg1_foot_link")->pose.t.x, frameOf(atLimit, "leg1_foot_link")->pose.t.x);
  CHECK_NEAR(frameOf(over, "leg1_foot_link")->pose.t.y, frameOf(atLimit, "leg1_foot_link")->pose.t.y);
  pose.clear();
  pose["leg1_coxa_joint"] = -100.0;
  const Scene under = buildScene(hexRobot, pose);
  CHECK_NEAR(frameOf(under, "leg1_foot_link")->pose.t.y, 0.16 * std::sin(mj[0].lower));
  pose.clear();
  pose["leg1_foot_joint"] = 1.0;   // a fixed joint
  pose["no_such_joint"] = 1.0;
  const Scene ignored = buildScene(hexRobot, pose);
  CHECK_NEAR(frameOf(ignored, "leg1_foot_link")->pose.t.x, frameOf(rest, "leg1_foot_link")->pose.t.x);

  // A prismatic joint slides its whole subtree.
  ArmConfig ac;
  ac.links.clear();
  ArmLinkConfig slide;
  slide.type = ArmJointType::Prismatic;
  slide.axis = ArmAxis::Z;
  slide.a = 0; slide.b = 0; slide.c = 0.15;
  slide.lower = 0; slide.upper = 0.3;
  ac.links.push_back(slide);
  const Robot armRobot = buildArm(ac);
  const double toolRest = frameOf(buildScene(armRobot), "tool0")->pose.t.z;
  pose.clear();
  pose["joint_1"] = 0.2;
  CHECK_NEAR(frameOf(buildScene(armRobot, pose), "tool0")->pose.t.z, toolRest + 0.2);
  pose["joint_1"] = 5.0;  // metres, clamped to the 0.3 m stroke
  CHECK_NEAR(frameOf(buildScene(armRobot, pose), "tool0")->pose.t.z, toolRest + 0.3);

  // Continuous joints are not clamped: a wheel can turn any number of times without moving its centre.
  const Robot dd = buildDefault(RobotType::DiffDrive);
  const Scene ddRest = buildScene(dd);
  pose.clear();
  pose["left_wheel_joint"] = 100.0;
  const Scene spun = buildScene(dd, pose);
  const SceneFrame* w = frameOf(spun, "left_wheel_link");
  const SceneFrame* w0 = frameOf(ddRest, "left_wheel_link");
  CHECK(w && w0);
  if (w && w0) {
    CHECK_NEAR(w->pose.t.x, w0->pose.t.x); CHECK_NEAR(w->pose.t.y, w0->pose.t.y); CHECK_NEAR(w->pose.t.z, w0->pose.t.z);
    CHECK_NEAR(w->pose.r[0], std::cos(100.0)); CHECK_NEAR(w->pose.r[2], std::sin(100.0));
    CHECK_NEAR(w->pose.r[4], 1.0);  // it rolls about its own y axis
  }
  // The wheel is a cylinder, so spinning it leaves the overall bounds as they were.
  CHECK_NEAR(spun.min.z, ddRest.min.z);
  CHECK_NEAR(spun.groundZ, 0.0);

  // A bare-frame robot has a ground height as well.
  Robot bare("bare");
  Link f;
  f.name = "only_frame";
  f.hasBody = false;
  bare.root.links.push_back(f);
  CHECK_NEAR(buildScene(bare, JointPose()).groundZ, 0.0);

  // For every default robot in a pose that moves every joint: the meshes follow the link frames,
  // the bounds contain every vertex, and a rigid chain keeps its link lengths.
  for (RobotType t : allRobotTypes()) {
    const Robot r = buildDefault(t);
    const Part part = r.flatten();
    const JointPose dp = demoPose(r);
    const Scene s = buildScene(r, dp);
    const Scene n = buildScene(r);
    CHECK(s.bodies.size() == n.bodies.size() && s.frames.size() == n.frames.size());
    CHECK_NEAR(s.groundZ, n.groundZ);
    for (const Link& l : part.links) {
      if (!l.hasBody) continue;
      const SceneBody* b = bodyOf(s, l.name);
      const SceneFrame* fr = frameOf(s, l.name);
      CHECK(b && fr);
      if (!b || !fr) continue;
      Vec3 centroid;
      for (const Vec3& v : b->mesh.vertices) centroid = centroid + v;
      centroid = centroid * (1.0 / static_cast<double>(b->mesh.vertices.size()));
      const Vec3 expected = fr->pose.apply(Transform::fromOrigin(l.origin).t);
      CHECK_TOL(centroid.x, expected.x, 1e-9);
      CHECK_TOL(centroid.y, expected.y, 1e-9);
      CHECK_TOL(centroid.z, expected.z, 1e-9);
      for (const Vec3& v : b->mesh.vertices) {
        CHECK(v.x >= s.min.x - 1e-12 && v.x <= s.max.x + 1e-12 && v.z >= s.min.z - 1e-12 && v.z <= s.max.z + 1e-12);
      }
    }
    for (const Joint& j : part.joints) {  // the distance from a joint to its child frame never changes
      const SceneFrame* pf = frameOf(s, j.parent);
      const SceneFrame* cf = frameOf(s, j.child);
      const SceneFrame* pn = frameOf(n, j.parent);
      const SceneFrame* cn = frameOf(n, j.child);
      if (!pf || !cf || !pn || !cn || j.type == JointType::Prismatic) continue;
      CHECK_TOL(length(cf->pose.t - pf->pose.t), length(cn->pose.t - pn->pose.t), 1e-9);
    }
    bool anyMoved = false;
    for (std::size_t i = 0; i < s.frames.size(); ++i) {
      if (length(s.frames[i].pose.t - n.frames[i].pose.t) > 1e-6) anyMoved = true;
      if (std::fabs(s.frames[i].pose.r[0] - n.frames[i].pose.r[0]) > 1e-6) anyMoved = true;
    }
    CHECK(anyMoved || movableJoints(r).empty());
  }
}

// ------------------------------------------------------------------ mecanum

const double kInvSqrt2 = 0.70710678118654752;

/// Number of rollers of the mecanum wheel `prefix` (0 for a plain wheel).
int rollerCount(const Part& p, const std::string& prefix) {
  int n = 0;
  for (const Link& l : p.links) {
    if (l.name.rfind(prefix + "_roller_", 0) == 0) ++n;
  }
  return n;
}

double wheelMassOf(const Part& p, const std::string& prefix) {
  double m = 0;
  for (const Link& l : p.links) {
    if (l.hasBody && (l.name == prefix + "_wheel_link" || l.name.rfind(prefix + "_roller_", 0) == 0)) {
      m += l.mass.value();
    }
  }
  return m;
}

/// Geometry of every roller of one mecanum wheel, in a scene where every joint is at rest:
/// they sit on a circle round the axle, evenly spaced with one at the bottom, each axis lies in
/// the tangent plane at 45 degrees to the axle, and the bottom roller points along (hand, 1).
void checkMecanumWheel(const Scene& sc, const std::string& prefix, int rollers, double ringR, double hand) {
  const SceneFrame* hub = frameOf(sc, prefix + "_wheel_link");
  CHECK(hub != nullptr);
  if (!hub) return;
  bool sawBottom = false;
  for (int k = 1; k <= rollers; ++k) {
    const SceneFrame* f = frameOf(sc, prefix + "_roller_" + std::to_string(k) + "_link");
    CHECK(f != nullptr);
    if (!f) continue;
    // Into the wheel frame (x forward, y along the axle, z up when the wheel is at rest).
    const Vec3 d = f->pose.t - hub->pose.t;
    const Vec3 p{hub->pose.r[0] * d.x + hub->pose.r[3] * d.y + hub->pose.r[6] * d.z,
                 hub->pose.r[1] * d.x + hub->pose.r[4] * d.y + hub->pose.r[7] * d.z,
                 hub->pose.r[2] * d.x + hub->pose.r[5] * d.y + hub->pose.r[8] * d.z};
    const Vec3 world{f->pose.r[2], f->pose.r[5], f->pose.r[8]};  // the roller's cylinder axis
    const Vec3 a{hub->pose.r[0] * world.x + hub->pose.r[3] * world.y + hub->pose.r[6] * world.z,
                 hub->pose.r[1] * world.x + hub->pose.r[4] * world.y + hub->pose.r[7] * world.z,
                 hub->pose.r[2] * world.x + hub->pose.r[5] * world.y + hub->pose.r[8] * world.z};

    const double phi = kPi + 2 * kPi * (k - 1) / rollers;  // angle from the top, so roller 1 is at the bottom
    CHECK_TOL(p.x, ringR * std::sin(phi), 1e-9);
    CHECK_TOL(p.y, 0.0, 1e-9);
    CHECK_TOL(p.z, ringR * std::cos(phi), 1e-9);

    const Vec3 radial{std::sin(phi), 0, std::cos(phi)};
    const Vec3 tangent{std::cos(phi), 0, -std::sin(phi)};
    CHECK_TOL(length(a), 1.0, 1e-9);
    CHECK_TOL(std::fabs(a.y), kInvSqrt2, 1e-9);        // 45 degrees to the axle
    CHECK_TOL(dot(a, radial), 0.0, 1e-9);               // in the tangent plane of the rim
    CHECK_TOL(dot(a, tangent), -hand * kInvSqrt2, 1e-9);  // and slanted the way the handedness says
    if (k == 1) {
      sawBottom = true;
      CHECK_TOL(p.x, 0.0, 1e-9);
      CHECK_TOL(p.z, -ringR, 1e-9);
      CHECK_TOL(a.x, hand * kInvSqrt2, 1e-9);  // top view: (hand, 1) / sqrt 2
      CHECK_TOL(a.y, kInvSqrt2, 1e-9);
      CHECK_TOL(a.z, 0.0, 1e-9);
    }
  }
  CHECK(sawBottom);
}

void testMecanum() {
  using K = WheelKind;
  const double sq = std::sqrt(2.0);

  // ---- all-mecanum 4-wheeler, square layout: the slant pattern of the standard layout
  FourWheelConfig fc;
  fc.wheelX = 0.12; fc.wheelY = 0.12;
  fc.frontLeftWheel = fc.frontRightWheel = fc.rearLeftWheel = fc.rearRightWheel = K::Mecanum;
  const Robot four = buildFourWheel(fc);
  CHECK(!hasErrors(validate(four)));
  const Part fp = four.flatten();
  CHECK(fp.links.size() == 2 + 4 * (1 + 8));   // base_footprint, base_link, hub + 8 rollers per wheel
  CHECK(fp.joints.size() == fp.links.size() - 1);
  const Scene fs = buildScene(four);
  const double rollerR = fc.wheelWidth * 0.25;
  const double ring = fc.wheelRadius - rollerR;
  struct W { const char* prefix; double hand, sx, sy; };
  const W wheels[4] = {{"front_left", 1, 1, 1}, {"front_right", -1, 1, -1}, {"rear_left", -1, -1, 1}, {"rear_right", 1, -1, -1}};
  for (const W& w : wheels) {
    CHECK(rollerCount(fp, w.prefix) == 8);
    checkMecanumWheel(fs, w.prefix, 8, ring, w.hand);
    // The ground roller's axis points at the middle of the robot (square layout): the axis line
    // through the wheel passes through the chassis centre.
    const SceneFrame* ground = frameOf(fs, std::string(w.prefix) + "_roller_1_link");
    const Vec3 axis{ground->pose.r[2], ground->pose.r[5], ground->pose.r[8]};
    const double cross2 = ground->pose.t.x * axis.y - ground->pose.t.y * axis.x;
    CHECK_TOL(cross2, 0.0, 1e-9);
    // Swapping a wheel does not move it, and the total wheel mass is unchanged.
    CHECK_NEAR(wheelMassOf(fp, w.prefix), fc.wheelMass);
    FourWheelConfig plain = fc;
    plain.frontLeftWheel = plain.frontRightWheel = plain.rearLeftWheel = plain.rearRightWheel = K::Standard;
    const Scene ps = buildScene(buildFourWheel(plain));
    const SceneFrame* a = frameOf(fs, std::string(w.prefix) + "_wheel_link");
    const SceneFrame* b = frameOf(ps, std::string(w.prefix) + "_wheel_link");
    CHECK(a && b);
    if (a && b) {
      CHECK_NEAR(a->pose.t.x, b->pose.t.x); CHECK_NEAR(a->pose.t.y, b->pose.t.y); CHECK_NEAR(a->pose.t.z, b->pose.t.z);
    }
  }
  // Every wheel still stands on the ground plane (a roller touches it at the bottom).
  CHECK_TOL(fs.min.z, 0.0, 5e-4);
  CHECK_NEAR(fs.groundZ, fs.min.z);
  // The wheel joints are the only movable joints: four, one chain each, named as for plain wheels.
  const std::vector<MovableJoint> mj = movableJoints(four);
  CHECK(mj.size() == 4);
  for (const MovableJoint& m : mj) {
    CHECK(m.type == JointType::Continuous);
    CHECK(m.name.find("_wheel_joint") != std::string::npos);
  }
  // Materials and xacro structure.
  const std::string fx = emitXacro(four);
  CHECK(fx.find("<xacro:macro name=\"mecanum_wheel\"") != std::string::npos);
  CHECK(fx.find("<xacro:macro name=\"drive_wheel\"") == std::string::npos);  // no plain wheel is used
  CHECK(countOf(fx, "<xacro:mecanum_wheel ") == 4);
  CHECK(fx.find("hand=\"1\"") != std::string::npos && fx.find("hand=\"-1\"") != std::string::npos);
  CHECK(fx.find("<xacro:property name=\"roller_radius\" value=\"${wheel_width*0.25}\"/>") != std::string::npos);
  CHECK(fx.find("name=\"roller_grey\"") != std::string::npos);
  CHECK(countOf(emitUrdf(four), "<link name=\"front_left_roller_") == 8);

  // ---- the default four-wheeler (not square): same slant per wheel, 9 rollers, mixed with plain wheels
  FourWheelConfig mixed;
  mixed.frontRightWheel = K::Mecanum;
  mixed.rearLeftWheel = K::Mecanum;
  mixed.mecanumRollers = 9;
  const Robot mixedRobot = buildFourWheel(mixed);
  CHECK(!hasErrors(validate(mixedRobot)));
  const Part mp = mixedRobot.flatten();
  CHECK(rollerCount(mp, "front_left") == 0 && rollerCount(mp, "rear_right") == 0);
  CHECK(rollerCount(mp, "front_right") == 9 && rollerCount(mp, "rear_left") == 9);
  CHECK(mp.links.size() == 2 + 4 + 2 * 9);
  const Scene ms = buildScene(mixedRobot);
  checkMecanumWheel(ms, "front_right", 9, mixed.wheelRadius - mixed.wheelWidth * 0.25, -1);
  checkMecanumWheel(ms, "rear_left", 9, mixed.wheelRadius - mixed.wheelWidth * 0.25, -1);
  const std::string mx = emitXacro(mixedRobot);
  CHECK(mx.find("<xacro:macro name=\"mecanum_wheel\"") != std::string::npos);
  CHECK(mx.find("<xacro:macro name=\"drive_wheel\"") != std::string::npos);
  CHECK(countOf(mx, "<xacro:mecanum_wheel ") == 2 && countOf(mx, "<xacro:drive_wheel ") == 2);
  CHECK(movableJoints(mixedRobot).size() == 4);

  // ---- a robot without mecanum wheels has no trace of them
  for (RobotType t : {RobotType::DiffDrive, RobotType::FourWheel, RobotType::Ackermann}) {
    const std::string plain = emitXacro(buildDefault(t));
    CHECK(plain.find("mecanum") == std::string::npos);
    CHECK(plain.find("roller") == std::string::npos);
    CHECK(plain.find("hand") == std::string::npos);
  }

  // ---- differential drive: the slant follows the side of the centre line the wheel is on
  DiffDriveConfig dc;
  dc.leftWheel = dc.rightWheel = K::Mecanum;
  dc.mecanumRollers = 6;
  const Scene ds = buildScene(buildDiffDrive(dc));
  const double dRing = dc.wheelRadius - dc.wheelWidth * 0.25;
  checkMecanumWheel(ds, "left", 6, dRing, 1);    // wheel axle at x = 0 counts as a front axle
  checkMecanumWheel(ds, "right", 6, dRing, -1);
  CHECK_TOL(ds.min.z, 0.0, 5e-4);
  dc.wheelX = -0.04;   // axle behind the centre: left becomes a rear-left wheel, right a rear-right wheel
  const Scene rearDs = buildScene(buildDiffDrive(dc));
  checkMecanumWheel(rearDs, "left", 6, dRing, -1);
  checkMecanumWheel(rearDs, "right", 6, dRing, 1);
  DiffDriveConfig half;
  half.leftWheel = K::Mecanum;
  const Part hp = buildDiffDrive(half).flatten();
  CHECK(rollerCount(hp, "left") == 8 && rollerCount(hp, "right") == 0);
  CHECK_NEAR(wheelMassOf(hp, "left"), half.wheelMass);
  CHECK_NEAR(wheelMassOf(hp, "right"), half.wheelMass);

  // ---- Ackermann: mecanum front wheels hang below the steering joints, and steer with them
  AckermannConfig ac;
  ac.frontLeftWheel = ac.frontRightWheel = ac.rearLeftWheel = ac.rearRightWheel = K::Mecanum;
  ac.mecanumRollers = 7;
  const Robot ack = buildAckermann(ac);
  CHECK(!hasErrors(validate(ack)));
  const Part ap = ack.flatten();
  CHECK(findJoint(ap, "front_left_wheel_joint")->parent == "front_left_steer_link");
  CHECK(findJoint(ap, "rear_left_wheel_joint")->parent == "base_link");
  CHECK(findJoint(ap, "front_left_steer_joint")->type == JointType::Revolute);
  const double aRing = ac.wheelRadius - ac.wheelWidth * 0.25;
  const Scene as = buildScene(ack);
  checkMecanumWheel(as, "front_left", 7, aRing, 1);
  checkMecanumWheel(as, "front_right", 7, aRing, -1);
  checkMecanumWheel(as, "rear_left", 7, aRing, -1);
  checkMecanumWheel(as, "rear_right", 7, aRing, 1);
  CHECK_TOL(as.min.z, 0.0, 1e-3);
  const std::string ax = emitXacro(ack);
  CHECK(ax.find("<xacro:macro name=\"steered_mecanum_wheel\"") != std::string::npos);
  CHECK(ax.find("<xacro:macro name=\"steered_wheel\"") == std::string::npos);
  CHECK(ax.find("<xacro:macro name=\"drive_wheel\"") == std::string::npos);
  const std::vector<MovableJoint> amj = movableJoints(ack);
  CHECK(amj.size() == 6);  // 2 steering joints + 4 wheel joints
  std::map<std::string, int> chain;
  for (const MovableJoint& m : amj) chain[m.name] = m.chain;
  CHECK(chain.at("front_left_steer_joint") == chain.at("front_left_wheel_joint"));  // the roller joints are fixed and add none

  // Steering turns the whole front wheel with its rollers; the rollers stay where they are on the hub.
  JointPose steer;
  steer["front_left_steer_joint"] = 0.4;
  steer["front_left_wheel_joint"] = 1.1;
  const Scene turned = buildScene(ack, steer);
  const SceneFrame* hubRest = frameOf(as, "front_left_wheel_link");
  const SceneFrame* hubTurned = frameOf(turned, "front_left_wheel_link");
  CHECK(hubRest && hubTurned);
  for (int k = 1; k <= 7; ++k) {
    const std::string name = "front_left_roller_" + std::to_string(k) + "_link";
    const SceneFrame* r0 = frameOf(as, name);
    const SceneFrame* r1 = frameOf(turned, name);
    CHECK(r0 && r1);
    if (r0 && r1 && hubRest && hubTurned) {
      CHECK_TOL(length(r1->pose.t - hubTurned->pose.t), length(r0->pose.t - hubRest->pose.t), 1e-9);
    }
  }
  CHECK(hubTurned && std::fabs(hubTurned->pose.r[0] - hubRest->pose.r[0]) > 0.01);  // it really turned

  // ---- a wheel too wide for its radius cannot have rollers round the hub: one clear error
  FourWheelConfig wide;
  wide.frontLeftWheel = K::Mecanum;
  wide.wheelRadius = 0.03;
  wide.wheelWidth = 0.07;
  {
    const std::vector<Issue> issues = validate(buildFourWheel(wide));
    CHECK(hasErrors(issues));
    bool said = false;
    for (const Issue& i : issues) said = said || i.message.find("mecanum wheel must be narrower") != std::string::npos;
    CHECK(said);
    wide.frontLeftWheel = K::Standard;  // the same wheel as a plain wheel is fine
    CHECK(!hasErrors(validate(buildFourWheel(wide))));
  }

  // ---- limits on the roller count, and the same wheels with different counts have the same footprint
  FourWheelConfig few;
  few.frontLeftWheel = K::Mecanum;
  few.mecanumRollers = 1;
  CHECK(rollerCount(buildFourWheel(few).flatten(), "front_left") == 3);
  few.mecanumRollers = 500;
  CHECK(rollerCount(buildFourWheel(few).flatten(), "front_left") == 16);
  // Whatever the number of rollers, the wheels stand on the ground plane to within a millimetre.
  for (int n = 3; n <= 16; ++n) {
    FourWheelConfig all;
    all.frontLeftWheel = all.frontRightWheel = all.rearLeftWheel = all.rearRightWheel = K::Mecanum;
    all.mecanumRollers = n;
    const Scene sc = buildScene(buildFourWheel(all));
    CHECK_TOL(sc.min.z, 0.0, 1e-3);
  }

  // ---- roller dimensions: inside the wheel's width, and the pieces add up to the wheel mass
  const Part pf = four.flatten();
  for (const Link& l : pf.links) {
    if (l.name == "front_left_roller_1_link") {
      CHECK_NEAR(l.geometry.a.value(), fc.wheelWidth * 0.25);
      CHECK_NEAR(l.geometry.b.value(), sq * fc.wheelWidth - 2 * fc.wheelWidth * 0.25);
      // The slanted roller reaches exactly the sides of the wheel: length/sqrt2 + diameter/sqrt2 = width.
      CHECK_TOL((l.geometry.b.value() + 2 * l.geometry.a.value()) / sq, fc.wheelWidth, 1e-9);
      CHECK_NEAR(l.mass.value(), fc.wheelMass * 0.4 / 8);
    }
    if (l.name == "front_left_wheel_link") {
      CHECK_NEAR(l.geometry.a.value(), fc.wheelRadius - fc.wheelWidth * 0.45);  // the rollers sit on the hub rim
      CHECK_NEAR(l.mass.value(), fc.wheelMass * 0.6);
    }
  }

  // ---- every kind of mecanum wheel keeps its rollers rigid on the hub, in every pose
  const Robot robots[] = {four, mixedRobot, ack, buildDiffDrive(dc)};
  for (const Robot& r : robots) {
    const Part part = r.flatten();
    const Scene rest = buildScene(r);
    const Scene posed = buildScene(r, demoPose(r));
    for (const Joint& j : part.joints) {
      if (j.type != JointType::Fixed || j.name.find("_roller_") == std::string::npos) continue;
      const SceneFrame *p0 = frameOf(rest, j.parent), *c0 = frameOf(rest, j.child);
      const SceneFrame *p1 = frameOf(posed, j.parent), *c1 = frameOf(posed, j.child);
      CHECK(p0 && c0 && p1 && c1);
      if (p0 && c0 && p1 && c1) {
        CHECK_TOL(length(c1->pose.t - p1->pose.t), length(c0->pose.t - p0->pose.t), 1e-9);
        double dr = 0;  // the roller's axis keeps its angle to the hub's axis (y)
        const Vec3 aRest{c0->pose.r[2], c0->pose.r[5], c0->pose.r[8]}, hubRest{p0->pose.r[1], p0->pose.r[4], p0->pose.r[7]};
        const Vec3 aPosed{c1->pose.r[2], c1->pose.r[5], c1->pose.r[8]}, hubPosed{p1->pose.r[1], p1->pose.r[4], p1->pose.r[7]};
        dr = std::fabs(dot(aRest, hubRest) - dot(aPosed, hubPosed));
        CHECK_TOL(dr, 0.0, 1e-9);
      }
    }
  }
}

// ------------------------------------------------------------------- sensors

bool hasIssue(const std::vector<Issue>& issues, const std::string& needle) {
  for (const Issue& i : issues) {
    if (i.severity == Issue::Severity::Error && i.message.find(needle) != std::string::npos) return true;
  }
  return false;
}

Vec3 col(const Transform& T, int c) { return {T.r[static_cast<std::size_t>(c)], T.r[static_cast<std::size_t>(3 + c)], T.r[static_cast<std::size_t>(6 + c)]}; }

void checkVec(const Vec3& a, const Vec3& b, double tol = 1e-9) {
  CHECK_TOL(a.x, b.x, tol);
  CHECK_TOL(a.y, b.y, tol);
  CHECK_TOL(a.z, b.z, tol);
}

Link cylinderLink(double radius, double length, const Origin& o) {
  Link l;
  l.name = "probe";
  l.origin = o;
  l.geometry = Geometry::cylinder(radius, length);
  l.mass = 1.0;
  return l;
}

Link boxLink(double x, double y, double z, const Origin& o) {
  Link l;
  l.name = "probe";
  l.origin = o;
  l.geometry = Geometry::box(x, y, z);
  l.mass = 1.0;
  return l;
}

void testLinkAxis() {
  // The legs of the default hexapod: coxa and femur point along +x, the tibia hangs down -z.
  const Robot hex = buildDefault(RobotType::Hexapod);
  const Part part = hex.flatten();
  struct Expect { const char* link; Vec3 dir; double lower, upper; };
  const Expect expected[] = {{"leg1_coxa_link", {1, 0, 0}, 0, 0.06},
                             {"leg1_femur_link", {1, 0, 0}, 0, 0.10},
                             {"leg1_tibia_link", {0, 0, -1}, 0, 0.15},
                             {"leg4_tibia_link", {0, 0, -1}, 0, 0.15},
                             {"base_link", {0, 0, 1}, -0.025, 0.025}};
  for (const Expect& e : expected) {
    const Link* l = findLink(part, e.link);
    CHECK(l != nullptr);
    if (!l) continue;
    const LinkAxis a = linkAxis(*l);
    checkVec(a.direction, e.dir);
    CHECK_NEAR(a.lower, e.lower);
    CHECK_NEAR(a.upper, e.upper);
  }
  // The foot is a small sphere: its axis is z and it is centred on the root.
  const Link* foot = findLink(part, "leg1_foot_link");
  CHECK(foot != nullptr);
  if (foot) {
    const LinkAxis a = linkAxis(*foot);
    checkVec(a.direction, {0, 0, 1});
    CHECK_NEAR(a.lower, -0.012);
    CHECK_NEAR(a.upper, 0.012);
    CHECK_NEAR(linkTop(*foot), 0.012);
  }
  // A bare frame has nothing to slide along.
  const Link* footprint = findLink(buildDefault(RobotType::DiffDrive).flatten(), "base_footprint");
  CHECK(footprint != nullptr && !footprint->hasBody);
  if (footprint) {
    const LinkAxis a = linkAxis(*footprint);
    checkVec(a.direction, {1, 0, 0});
    CHECK_NEAR(a.lower, 0.0);
    CHECK_NEAR(a.upper, 0.0);
    CHECK_NEAR(linkTop(*footprint), 0.0);
  }
  const Link* tibia = findLink(part, "leg1_tibia_link");
  if (tibia) CHECK_TOL(linkTop(*tibia), 0.0, 1e-9);
  const Link* base = findLink(part, "base_link");
  if (base) CHECK_NEAR(linkTop(*base), 0.025);

  // Wheels roll about y: the axle is the link's axis and the wheel is centred on the root.
  const Part dd = buildDefault(RobotType::DiffDrive).flatten();
  const Link* wheel = findLink(dd, "left_wheel_link");
  CHECK(wheel != nullptr);
  if (wheel) {
    const LinkAxis a = linkAxis(*wheel);
    checkVec(a.direction, {0, 1, 0});
    CHECK_NEAR(a.lower, -0.015);
    CHECK_NEAR(a.upper, 0.015);
    CHECK_NEAR(linkTop(*wheel), 0.05);  // the wheel radius
  }
  // A box chassis: its longest side (x), centred.
  const Link* chassis = findLink(dd, "base_link");
  CHECK(chassis != nullptr);
  if (chassis) {
    const LinkAxis a = linkAxis(*chassis);
    checkVec(a.direction, {1, 0, 0});
    CHECK_NEAR(a.lower, -0.15);
    CHECK_NEAR(a.upper, 0.15);
  }

  // Hand-made links.
  const double t = 0.5;  // a cylinder tilted about x, its middle 0.1 m up the tilted axis
  const Vec3 tilted{0, -std::sin(t), std::cos(t)};
  {
    const Link l = cylinderLink(0.01, 0.2, Origin(0, 0.1 * tilted.y, 0.1 * tilted.z, t, 0, 0));
    const LinkAxis a = linkAxis(l);
    checkVec(a.direction, tilted);
    CHECK_NEAR(a.lower, 0.0);
    CHECK_NEAR(a.upper, 0.2);
  }
  {
    const Link l = cylinderLink(0.01, 0.2, Origin(0, -0.1 * tilted.y, -0.1 * tilted.z, t, 0, 0));  // behind the root
    const LinkAxis a = linkAxis(l);
    checkVec(a.direction, tilted * -1.0);  // flipped, so the body is at positive distances
    CHECK_NEAR(a.lower, 0.0);
    CHECK_NEAR(a.upper, 0.2);
  }
  {
    const LinkAxis a = linkAxis(boxLink(0.05, 0.3, 0.1, Origin(0, 0.15, 0)));
    checkVec(a.direction, {0, 1, 0});
    CHECK_NEAR(a.lower, 0.0);
    CHECK_NEAR(a.upper, 0.3);
  }
  {
    const LinkAxis a = linkAxis(boxLink(0.05, 0.3, 0.1, Origin(0, -0.15, 0)));
    checkVec(a.direction, {0, -1, 0});
    CHECK_NEAR(a.lower, 0.0);
    CHECK_NEAR(a.upper, 0.3);
  }
  {
    const LinkAxis a = linkAxis(boxLink(0.2, 0.2, 0.1, Origin(0, 0, 0)));  // tie: x before y before z
    checkVec(a.direction, {1, 0, 0});
    CHECK_NEAR(a.lower, -0.1);
    CHECK_NEAR(a.upper, 0.1);
  }
  {
    const LinkAxis a = linkAxis(boxLink(0.1, 0.1, 0.3, Origin(0, 0, 0)));
    checkVec(a.direction, {0, 0, 1});
    CHECK_NEAR(a.lower, -0.15);
    CHECK_NEAR(a.upper, 0.15);
  }
  {
    const LinkAxis a = linkAxis(cylinderLink(0.01, 0.1, Origin(0, 0, 0.5)));  // far from the root: 0 stays reachable
    checkVec(a.direction, {0, 0, 1});
    CHECK_NEAR(a.lower, 0.0);
    CHECK_NEAR(a.upper, 0.55);
  }
  {
    Link s;
    s.name = "ball";
    s.geometry = Geometry::sphere(0.05);
    s.mass = 1;
    const LinkAxis a = linkAxis(s);
    checkVec(a.direction, {0, 0, 1});
    CHECK_NEAR(a.lower, -0.05);
    CHECK_NEAR(a.upper, 0.05);
    CHECK_TOL(linkTop(s), 0.05, 1e-9);
  }
  // Whatever the link, the range contains 0 and the direction is a unit vector.
  for (RobotType type : allRobotTypes()) {
    for (const Link& l : buildDefault(type).flatten().links) {
      const LinkAxis a = linkAxis(l);
      CHECK(a.lower <= 0.0 && a.upper >= 0.0);
      CHECK_TOL(length(a.direction), 1.0, 1e-9);
    }
  }
}

void testSensorNames() {
  CHECK(allSensorTypes().size() == 5);
  const std::set<std::string> ids = {"camera", "imu", "lidar", "gps", "odometry"};
  std::set<std::string> got, labels;
  for (SensorType t : allSensorTypes()) {
    got.insert(sensorId(t));
    labels.insert(sensorLabel(t));
    CHECK(sensorHeight(t) > 0.0);
  }
  CHECK(got == ids);
  CHECK(labels == (std::set<std::string>{"Camera", "IMU", "LiDAR", "GPS", "Odometry"}));
  // Every type has its own colour.
  std::set<std::array<float, 3>> colours;
  for (SensorType t : allSensorTypes()) colours.insert(sensorColor(t));
  CHECK(colours.size() == 5);
}

void testDefaultSensor() {
  const Robot hex = buildDefault(RobotType::Hexapod);
  SensorConfig cam = defaultSensor(hex, SensorType::Camera, {});
  CHECK(cam.name == "camera");
  CHECK(cam.link == "base_link");
  CHECK_NEAR(cam.offsetZ, 0.025 + sensorHeight(SensorType::Camera) / 2);  // sits on top of the body
  CHECK_NEAR(cam.distance, 0.0);
  CHECK_NEAR(cam.rollDeg + cam.pitchDeg + cam.yawDeg, 0.0);
  const SensorConfig cam2 = defaultSensor(hex, SensorType::Camera, {cam});
  CHECK(cam2.name == "camera_2");
  const SensorConfig cam3 = defaultSensor(hex, SensorType::Camera, {cam, cam2});
  CHECK(cam3.name == "camera_3");
  const SensorConfig lidar = defaultSensor(hex, SensorType::Lidar, {cam, cam2});
  CHECK(lidar.name == "lidar");
  CHECK_NEAR(lidar.offsetZ, 0.025 + sensorHeight(SensorType::Lidar) / 2);

  // The default sensor of every robot type is valid and sits on the top of the main body.
  for (RobotType type : allRobotTypes()) {
    Robot r = buildDefault(type);
    for (SensorType st : allSensorTypes()) {
      const SensorConfig s = defaultSensor(r, st, {});
      CHECK(s.link == "base_link");
      Robot with = buildDefault(type);
      addSensors(with, {s});
      CHECK(!hasErrors(validate(with)));
      const Scene sc = buildScene(with);
      CHECK(sc.sensors.size() == 1);
      const SceneBody* body = bodyOf(sc, s.name + "_link");
      const SceneBody* host = bodyOf(sc, "base_link");
      CHECK(body != nullptr && host != nullptr);
      if (body && host) {
        double sensorLow = 1e9, hostTop = -1e9;
        for (const Vec3& v : body->mesh.vertices) sensorLow = std::min(sensorLow, v.z);
        for (const Vec3& v : host->mesh.vertices) hostTop = std::max(hostTop, v.z);
        CHECK_TOL(sensorLow, hostTop, 1e-9);  // resting on the body
      }
    }
  }
}

void testAttachableLinks() {
  const std::vector<std::string> hex = attachableLinks(buildDefault(RobotType::Hexapod));
  CHECK(hex.size() == 1 + 6 * 4);  // base + coxa, femur, tibia, foot per leg
  CHECK(std::find(hex.begin(), hex.end(), "leg3_tibia_link") != hex.end());
  CHECK(std::find(hex.begin(), hex.end(), "base_link") != hex.end());

  // The rollers of a mecanum wheel are decoration: the wheel hub is the link to mount on.
  FourWheelConfig fw;
  fw.frontLeftWheel = WheelKind::Mecanum;
  fw.mecanumRollers = 6;
  const Robot r = buildFourWheel(fw);
  const std::vector<std::string> names = attachableLinks(r);
  const Part part = r.flatten();
  int rollers = 0;
  for (const Link& l : part.links) {
    if (l.name.find("roller") != std::string::npos) {
      ++rollers;
      CHECK(!l.attachable);
      CHECK(std::find(names.begin(), names.end(), l.name) == names.end());
    } else {
      CHECK(l.attachable);
      CHECK(std::find(names.begin(), names.end(), l.name) != names.end());
    }
  }
  CHECK(rollers == 6);
  CHECK(std::find(names.begin(), names.end(), "front_left_wheel_link") != names.end());
}

void testSensorPlacement() {
  Robot hex = buildDefault(RobotType::Hexapod);
  const std::size_t linksBefore = hex.flatten().links.size(), jointsBefore = hex.flatten().joints.size();

  SensorConfig cam;
  cam.name = "shin_cam";
  cam.link = "leg1_tibia_link";
  cam.distance = 0.05;
  cam.offsetX = 0.01; cam.offsetZ = 0.02;
  cam.rollDeg = 10; cam.pitchDeg = 20; cam.yawDeg = 30;
  addSensors(hex, {cam});
  CHECK(!hasErrors(validate(hex)));
  CHECK(hex.flatten().links.size() == linksBefore + 2);    // the sensor and the camera's optical frame
  CHECK(hex.flatten().joints.size() == jointsBefore + 2);
  CHECK(hex.sensorMarks.size() == 1);

  const Part part = hex.flatten();
  const Link* body = findLink(part, "shin_cam_link");
  const Joint* joint = findJoint(part, "shin_cam_joint");
  CHECK(body != nullptr && joint != nullptr);
  if (body && joint) {
    CHECK(body->hasBody && body->mass.value() > 0.0 && !body->attachable);
    CHECK(joint->type == JointType::Fixed);
    CHECK(joint->parent == "leg1_tibia_link" && joint->child == "shin_cam_link");
    // The tibia hangs along -z, so 5 cm along the link is z = -0.05; plus the offset.
    CHECK_NEAR(joint->origin.x.value(), 0.01);
    CHECK_NEAR(joint->origin.y.value(), 0.0);
    CHECK_NEAR(joint->origin.z.value(), -0.05 + 0.02);
    CHECK_NEAR(joint->origin.roll.value(), 10 * kPi / 180);
    CHECK_NEAR(joint->origin.pitch.value(), 20 * kPi / 180);
    CHECK_NEAR(joint->origin.yaw.value(), 30 * kPi / 180);
  }

  // In the world: the tibia frame carries the sensor.
  const Scene sc = buildScene(hex);
  const SceneFrame* tibia = frameOf(sc, "leg1_tibia_link");
  const SceneFrame* frame = frameOf(sc, "shin_cam_link");
  CHECK(tibia != nullptr && frame != nullptr);
  CHECK(sc.sensors.size() == 1);
  if (tibia && frame && !sc.sensors.empty()) {
    const Transform local = Transform::fromOrigin(Origin(0.01, 0, -0.03, 10 * kPi / 180, 20 * kPi / 180, 30 * kPi / 180));
    const Transform expected = tibia->pose * local;
    checkVec(frame->pose.t, expected.t);
    for (int i = 0; i < 9; ++i) CHECK_NEAR(frame->pose.r[static_cast<std::size_t>(i)], expected.r[static_cast<std::size_t>(i)]);
    const SceneSensor& s = sc.sensors[0];
    CHECK(s.name == "shin_cam" && s.type == SensorType::Camera);
    CHECK(s.link == "shin_cam_link" && s.parent == "leg1_tibia_link");
    checkVec(s.root, tibia->pose.t);  // the root of the link: where its frame is
    checkVec(s.pose.t, frame->pose.t);
    const auto c = sensorColor(SensorType::Camera);
    CHECK_TOL(s.color[0], c[0], 1e-6);
    CHECK_TOL(s.color[2], c[2], 1e-6);
  }

  // The sensor frame follows the leg when it moves: yaw the coxa and the whole thing swings round.
  JointPose pose;
  pose["leg1_coxa_joint"] = 0.4;
  pose["leg1_tibia_joint"] = -0.3;
  const Scene moved = buildScene(hex, pose);
  const SceneFrame* tibia2 = frameOf(moved, "leg1_tibia_link");
  const SceneFrame* frame2 = frameOf(moved, "shin_cam_link");
  if (tibia2 && frame2 && !moved.sensors.empty()) {
    const Transform local = Transform::fromOrigin(Origin(0.01, 0, -0.03, 10 * kPi / 180, 20 * kPi / 180, 30 * kPi / 180));
    checkVec(frame2->pose.t, (tibia2->pose * local).t);
    checkVec(moved.sensors[0].root, tibia2->pose.t);
    // The distance from the root does not change when the leg moves.
    CHECK_NEAR(length(moved.sensors[0].pose.t - moved.sensors[0].root), length(sc.sensors[0].pose.t - sc.sensors[0].root));
    CHECK(length(moved.sensors[0].pose.t - sc.sensors[0].pose.t) > 0.01);
  }
  CHECK_NEAR(moved.groundZ, sc.groundZ);

  // The camera's optical frame: z forward (the sensor's x), x right (-y), y down (-z).
  const SceneFrame* opt = frameOf(sc, "shin_cam_optical_frame");
  CHECK(opt != nullptr);
  if (opt && frame) {
    checkVec(col(opt->pose, 2), col(frame->pose, 0));
    checkVec(col(opt->pose, 0), col(frame->pose, 1) * -1.0);
    checkVec(col(opt->pose, 1), col(frame->pose, 2) * -1.0);
    checkVec(opt->pose.t, frame->pose.t);
  }
  const Part flat = hex.flatten();
  CHECK(findLink(flat, "shin_cam_optical_frame") && !findLink(flat, "shin_cam_optical_frame")->hasBody);

  // Only a camera has an optical frame.
  Robot imuBot = buildDefault(RobotType::Hexapod);
  SensorConfig imu = defaultSensor(imuBot, SensorType::Imu, {});
  addSensors(imuBot, {imu});
  CHECK(findLink(imuBot.flatten(), "imu_optical_frame") == nullptr);
  CHECK(findLink(imuBot.flatten(), "imu_link") != nullptr);

  // Zero distance and offset: the sensor is at the root of the link, along its axis.
  Robot r0 = buildDefault(RobotType::Hexapod);
  SensorConfig at0;
  at0.type = SensorType::Gps;
  at0.name = "gps0";
  at0.link = "leg2_femur_link";
  addSensors(r0, {at0});
  const Scene s0 = buildScene(r0);
  checkVec(frameOf(s0, "gps0_link")->pose.t, frameOf(s0, "leg2_femur_link")->pose.t);
  checkVec(s0.sensors[0].root, s0.sensors[0].pose.t);

  // Sliding along the link: the same sensor at 3 distances is on one straight line.
  Vec3 p[3];
  const double d[3] = {0.0, 0.04, 0.09};
  for (int i = 0; i < 3; ++i) {
    Robot r = buildDefault(RobotType::Hexapod);
    SensorConfig s;
    s.type = SensorType::Lidar;
    s.name = "lid";
    s.link = "leg2_femur_link";
    s.distance = d[i];
    addSensors(r, {s});
    p[i] = frameOf(buildScene(r), "lid_link")->pose.t;
  }
  CHECK_NEAR(length(p[1] - p[0]), 0.04);
  CHECK_NEAR(length(p[2] - p[0]), 0.09);
  checkVec(normalized(p[1] - p[0]), normalized(p[2] - p[0]));
  // ... and that line is the femur: from the knee joint (its root) towards the next joint 0.1 m on.
  const Scene full = buildScene(buildDefault(RobotType::Hexapod));
  const Vec3 femurRoot = frameOf(full, "leg2_femur_link")->pose.t;
  const Vec3 tibiaRoot = frameOf(full, "leg2_tibia_link")->pose.t;
  checkVec(p[0], femurRoot);
  checkVec(normalized(p[2] - p[0]), normalized(tibiaRoot - femurRoot));
}

void testSensorEmitters() {
  Robot r = buildDefault(RobotType::Hexapod);
  SensorConfig cam;
  cam.name = "shin_cam";
  cam.link = "leg1_tibia_link";
  cam.distance = 0.05;
  cam.offsetX = 0.01; cam.offsetZ = 0.02;
  cam.yawDeg = 30;
  SensorConfig lidar = defaultSensor(r, SensorType::Lidar, {cam});
  lidar.rollDeg = 90;
  addSensors(r, {cam, lidar});
  CHECK(!hasErrors(validate(r)));

  const std::string x = emitXacro(r);
  const std::string u = emitUrdf(r);
  CHECK(wellFormed(x) && wellFormed(u));
  // Placement is a set of xacro properties, so it can be changed in the file.
  for (const char* name : {"sensor_shin_cam_distance", "sensor_shin_cam_offset_x", "sensor_shin_cam_offset_y",
                           "sensor_shin_cam_offset_z", "sensor_shin_cam_roll", "sensor_shin_cam_pitch",
                           "sensor_shin_cam_yaw", "sensor_lidar_distance", "sensor_lidar_roll"}) {
    CHECK(countOf(x, std::string("<xacro:property name=\"") + name + "\"") == 1);
    CHECK(countOf(u, name) == 0);  // the plain URDF has numbers only
  }
  CHECK(x.find("name=\"sensor_shin_cam_distance\" value=\"0.05\"") != std::string::npos);
  CHECK(x.find("name=\"sensor_shin_cam_yaw\" value=\"0.5235987") != std::string::npos);
  CHECK(x.find("30 deg") != std::string::npos);
  CHECK(x.find("Camera 'shin_cam' on leg1_tibia_link") != std::string::npos);
  CHECK(x.find("LiDAR 'lidar' on base_link") != std::string::npos);
  // Along the tibia (which hangs down) is -z; along +x/+y the offsets alone.
  CHECK(x.find("xyz=\"${sensor_shin_cam_offset_x} ${sensor_shin_cam_offset_y} "
               "${-sensor_shin_cam_distance+sensor_shin_cam_offset_z}\"") != std::string::npos);
  CHECK(x.find("rpy=\"${sensor_shin_cam_roll} ${sensor_shin_cam_pitch} ${sensor_shin_cam_yaw}\"") != std::string::npos);

  for (const char* name : {"shin_cam_link", "shin_cam_optical_frame", "lidar_link"}) {
    CHECK(countOf(u, std::string("<link name=\"") + name + "\"") == 1);
  }
  CHECK(countOf(u, "<joint name=\"shin_cam_joint\" type=\"fixed\">") == 1);
  CHECK(countOf(u, "<joint name=\"shin_cam_optical_joint\" type=\"fixed\">") == 1);
  CHECK(countOf(u, "<joint name=\"lidar_joint\" type=\"fixed\">") == 1);
  CHECK(u.find("<link name=\"imu_link\"") == std::string::npos);
  // Each sensor link has visual, collision and inertial like every other body.
  CHECK(countOf(u, "<collision>") == countOf(u, "<visual>"));
  CHECK(countOf(u, "<inertial>") == countOf(u, "<visual>") + 0);  // bare frames have none of the three
  CHECK(u.find("<material name=\"sensor_camera\">") != std::string::npos);
  CHECK(u.find("<material name=\"sensor_lidar\">") != std::string::npos);
  CHECK(u.find("<material name=\"sensor_gps\">") == std::string::npos);  // only the types in use

  // No sensors: nothing at all in the output.
  const std::string plain = emitXacro(buildDefault(RobotType::Hexapod));
  CHECK(plain.find("sensor_") == std::string::npos);
}

void testSensorMaterials() {
  // Materials are defined once per type that is used, never for the others.
  Robot r = buildDefault(RobotType::DiffDrive);
  SensorConfig a = defaultSensor(r, SensorType::Camera, {});
  SensorConfig b = defaultSensor(r, SensorType::Camera, {a});
  SensorConfig c = defaultSensor(r, SensorType::Odometry, {a, b});
  addSensors(r, {a, b, c});
  std::set<std::string> defined, used;
  for (const auto& m : r.materials) CHECK(defined.insert(m.name).second);
  for (const auto& l : r.flatten().links) {
    if (l.hasBody && !l.material.empty()) used.insert(l.material);
  }
  CHECK(defined == used);
  CHECK(defined.count("sensor_camera") == 1 && defined.count("sensor_odometry") == 1);
  CHECK(defined.count("sensor_imu") == 0);
}

void testSensorIssues() {
  auto with = [](const std::vector<SensorConfig>& sensors) {
    Robot r = buildDefault(RobotType::Hexapod);
    addSensors(r, sensors);
    return r;
  };
  SensorConfig ok;
  ok.name = "cam";
  CHECK(!hasErrors(validate(with({ok}))));

  SensorConfig missing = ok;
  missing.link = "no_such_link";
  Robot r1 = with({missing});
  CHECK(hasIssue(validate(r1), "no_such_link"));
  CHECK(r1.sensorMarks.empty());               // not added
  CHECK(r1.flatten().links.size() == buildDefault(RobotType::Hexapod).flatten().links.size());

  SensorConfig same = ok;  // a second sensor with the same name
  Robot r2 = with({ok, same});
  CHECK(hasIssue(validate(r2), "more than one sensor named 'cam'"));
  CHECK(r2.sensorMarks.size() == 1);

  for (const char* bad : {"", "my cam", "cam-1", "cam.1", "caméra"}) {
    SensorConfig s = ok;
    s.name = bad;
    Robot r = with({s});
    CHECK(hasIssue(validate(r), "letters, digits"));
    CHECK(r.sensorMarks.empty());
  }
  for (const char* good : {"cam", "Cam_1", "_x", "9lives", "a"}) {
    SensorConfig s = ok;
    s.name = good;
    CHECK(!hasErrors(validate(with({s}))));
  }

  // A sensor whose link or joint name is already used by the robot.
  SensorConfig clash = ok;
  clash.name = "leg1_coxa";  // would make the link leg1_coxa_link
  Robot r3 = with({clash});
  CHECK(hasIssue(validate(r3), "already used"));
  CHECK(r3.sensorMarks.empty());
  SensorConfig clash2 = ok;
  clash2.name = "base";  // base_link
  CHECK(hasIssue(validate(with({clash2})), "already used"));
  // A camera also claims the names of its optical frame.
  SensorConfig a = ok, b = ok;
  a.name = "cam";
  b.name = "cam_optical";  // its joint, cam_optical_joint, is the one of cam's optical frame
  Robot r5 = with({a, b});
  CHECK(hasIssue(validate(r5), "already used"));
  CHECK(r5.sensorMarks.size() == 1);
  CHECK(hasErrors(validate(with({b, a}))));  // the order does not matter
  SensorConfig c = ok;
  c.name = "cam_optical_frame";
  CHECK(!hasErrors(validate(with({c}))));  // cam_optical_frame_link: no clash either

  // A sensor cannot be mounted on another sensor (it is not one of the robot's own links).
  SensorConfig onSensor = ok;
  onSensor.name = "cam2";
  onSensor.link = "cam_link";
  CHECK(hasIssue(validate(with({ok, onSensor})), "cam_link"));

  // A bad sensor does not stop the good ones.
  SensorConfig good = ok;
  good.name = "good";
  Robot r4 = with({missing, good});
  CHECK(r4.sensorMarks.size() == 1 && r4.sensorMarks[0].name == "good");
  CHECK(hasErrors(validate(r4)));
}

void testSensorEverywhere() {
  // A sensor on every attachable link of every robot: valid, and in the scene at the right place.
  for (RobotType type : allRobotTypes()) {
    const Robot base = buildDefault(type);
    for (const std::string& link : attachableLinks(base)) {
      Robot r = buildDefault(type);
      SensorConfig s;
      s.type = SensorType::Imu;
      s.name = "probe";
      s.link = link;
      const LinkAxis axis = linkAxis(*findLink(r.flatten(), link));
      s.distance = 0.5 * (axis.lower + axis.upper);
      addSensors(r, {s});
      if (hasErrors(validate(r))) {
        std::fprintf(stderr, "  sensor on %s of %s does not validate\n", link.c_str(), r.name.c_str());
      }
      CHECK(!hasErrors(validate(r)));
      const Scene sc = buildScene(r);
      const SceneFrame* host = frameOf(sc, link);
      const SceneFrame* f = frameOf(sc, "probe_link");
      CHECK(host != nullptr && f != nullptr);
      if (host && f) {
        const Vec3 local = host->pose.rotate(axis.direction * s.distance);
        checkVec(f->pose.t - host->pose.t, local, 1e-9);
      }
    }
  }
}

void testSensorMany() {
  Robot r = buildDefault(RobotType::DiffDrive);
  std::vector<SensorConfig> list;
  for (SensorType t : allSensorTypes()) {
    SensorConfig s = defaultSensor(r, t, list);
    s.offsetX = 0.02 * static_cast<double>(list.size());
    list.push_back(s);
  }
  list.push_back(defaultSensor(r, SensorType::Camera, list));
  CHECK(list.back().name == "camera_2");
  list.back().link = "left_wheel_link";
  addSensors(r, list);
  CHECK(!hasErrors(validate(r)));
  CHECK(r.sensorMarks.size() == 6);
  const Scene sc = buildScene(r);
  CHECK(sc.sensors.size() == 6);
  for (std::size_t i = 0; i < list.size(); ++i) {  // in the order they were given
    CHECK(sc.sensors[i].name == list[i].name);
    CHECK(sc.sensors[i].type == list[i].type);
    CHECK(sc.sensors[i].parent == list[i].link);
  }
  // Bounds include the sensors' bodies.
  Robot bare = buildDefault(RobotType::DiffDrive);
  Robot high = buildDefault(RobotType::DiffDrive);
  SensorConfig tall = defaultSensor(high, SensorType::Lidar, {});
  tall.offsetZ = 1.0;
  addSensors(high, {tall});
  CHECK(buildScene(high).max.z > buildScene(bare).max.z + 0.9);
  // ... but the ground does not follow a sensor, even one far below the robot.
  Robot low = buildDefault(RobotType::DiffDrive);
  SensorConfig under = defaultSensor(low, SensorType::Gps, {});
  under.offsetZ = -1.0;
  addSensors(low, {under});
  CHECK_NEAR(buildScene(low).groundZ, buildScene(bare).groundZ);
  CHECK(buildScene(low).min.z < buildScene(bare).min.z - 0.5);
}

void testSensorOnMecanum() {
  FourWheelConfig fw;
  fw.frontLeftWheel = WheelKind::Mecanum;
  Robot r = buildFourWheel(fw);
  SensorConfig s;
  s.type = SensorType::Odometry;
  s.name = "hub_odom";
  s.link = "front_left_wheel_link";
  s.distance = 0.02;
  addSensors(r, {s});
  CHECK(!hasErrors(validate(r)));
  const Scene sc = buildScene(r);
  const SceneFrame* wheel = frameOf(sc, "front_left_wheel_link");
  const SceneFrame* f = frameOf(sc, "hub_odom_link");
  if (wheel && f) checkVec(f->pose.t - wheel->pose.t, col(wheel->pose, 1) * 0.02);  // 2 cm along the axle (y)
  // The wheel spins, and the sensor spins with it.
  JointPose pose;
  pose["front_left_wheel_joint"] = 1.0;
  const Scene spun = buildScene(r, pose);
  const SceneFrame* f2 = frameOf(spun, "hub_odom_link");
  const SceneFrame* w2 = frameOf(spun, "front_left_wheel_link");
  if (f2 && w2) {
    checkVec(f2->pose.t - w2->pose.t, col(w2->pose, 1) * 0.02);
    for (int i = 0; i < 3; ++i) checkVec(col(f2->pose, i), col(w2->pose, i));
  }
}

// -------------------------------------------------------------------- camera

void testCamera() {
  Camera c;
  c.azimuthDeg = 0;
  c.elevationDeg = 0;
  c.distance = 2.0;
  const Vec3 e = c.eye();
  CHECK_NEAR(e.x, 2.0); CHECK_NEAR(e.y, 0.0); CHECK_NEAR(e.z, 0.0);
  Vec3 right, up, back;
  c.axes(&right, &up, &back);
  CHECK_NEAR(right.y, 1.0); CHECK_NEAR(up.z, 1.0); CHECK_NEAR(back.x, 1.0);

  Camera::Screen s;
  CHECK(c.project({0, 0, 0}, 400, 300, &s));
  CHECK_NEAR(s.x, 200.0); CHECK_NEAR(s.y, 150.0); CHECK_NEAR(s.depth, 2.0);
  CHECK(c.project({0, 0.1, 0}, 400, 300, &s));
  CHECK(s.x > 200.0 && std::fabs(s.y - 150.0) < 1e-9);
  CHECK(c.project({0, 0, 0.1}, 400, 300, &s));
  CHECK(s.y < 150.0);
  CHECK(!c.project({3, 0, 0}, 400, 300, &s));  // behind the camera

  Camera::Screen a, b;
  CHECK(c.projectSegment({0, 0, 0}, {5, 0, 0.5}, 400, 300, &a, &b));  // second end is behind the camera
  CHECK_TOL(b.depth, c.nearPlane(), 1e-9);
  CHECK(!c.projectSegment({3, 0, 0}, {5, 0, 0}, 400, 300, &a, &b));   // entirely behind

  // Elevation is clamped so the basis never degenerates.
  Camera top = c;
  top.elevationDeg = 90;
  top.axes(&right, &up, &back);
  CHECK(std::isfinite(right.x) && std::isfinite(up.z));
  CHECK_NEAR(length(right), 1.0);

  // fit() always frames the whole scene, for wide and narrow viewports.
  for (RobotType t : allRobotTypes()) {
    const Scene sc = buildScene(buildDefault(t));
    for (double aspect : {0.4, 1.0, 1.7, 3.0}) {
      const int w = static_cast<int>(400 * aspect), h = 400;
      const Camera fitted = Camera::fit(sc, aspect, -50, 25);
      if (!fitted.contains(sc, w, h)) {
        std::fprintf(stderr, "  [%s] aspect %.1f: scene not inside the fitted view\n", robotTypeId(t).c_str(), aspect);
      }
      CHECK(fitted.contains(sc, w, h));

      // ...and fills most of the viewport in the limiting direction, instead of floating small in the middle.
      double minX = 1e18, maxX = -1e18, minY = 1e18, maxY = -1e18;
      for (int i = 0; i < 8; ++i) {
        const Vec3 corner{(i & 1) ? sc.max.x : sc.min.x, (i & 2) ? sc.max.y : sc.min.y, (i & 4) ? sc.max.z : sc.min.z};
        Camera::Screen p;
        CHECK(fitted.project(corner, w, h, &p));
        minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y); maxY = std::max(maxY, p.y);
      }
      const double fill = std::max((maxX - minX) / w, (maxY - minY) / h);
      if (fill < 0.8) {
        std::fprintf(stderr, "  [%s] aspect %.1f: fills only %.0f%% of the view\n", robotTypeId(t).c_str(), aspect, fill * 100);
      }
      CHECK(fill > 0.8 && fill <= 1.0);
    }
  }
}

// ------------------------------------------------------------------ renderer

Scene makeScene(const std::vector<std::pair<Geometry, Vec3>>& items, const std::vector<std::array<float, 3>>& colors) {
  Scene s;
  for (std::size_t i = 0; i < items.size(); ++i) {
    SceneBody b;
    b.link = "body" + std::to_string(i);
    b.mesh = tessellate(items[i].first);
    for (Vec3& v : b.mesh.vertices) v = v + items[i].second;
    b.color = colors[i];
    s.bodies.push_back(b);
  }
  s.min = {-1, -1, -1};
  s.max = {1, 1, 1};
  return s;
}

std::uint32_t pixelAt(const Image& img, int x, int y) {
  return img.pixels[static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(x)];
}
int red(std::uint32_t p) { return (p >> 16) & 255; }
int green(std::uint32_t p) { return (p >> 8) & 255; }
int blue(std::uint32_t p) { return p & 255; }

std::size_t covered(const Image& img) {
  std::size_t n = 0;
  for (std::uint32_t p : img.pixels) n += p != 0;
  return n;
}

void testRenderer() {
  const std::array<float, 3> kRed{{1, 0, 0}}, kGreen{{0, 1, 0}}, kGrey{{0.7f, 0.7f, 0.7f}};
  Camera cam;
  cam.azimuthDeg = 0;
  cam.elevationDeg = 0;
  cam.distance = 4;

  // The z-buffer, not the drawing order, decides what is in front.
  const Geometry cube = Geometry::box(0.4, 0.4, 0.4);
  for (int order = 0; order < 2; ++order) {
    std::vector<std::pair<Geometry, Vec3>> items = {{cube, {0.5, 0, 0}}, {cube, {-0.5, 0, 0}}};
    std::vector<std::array<float, 3>> cols = {kRed, kGreen};
    if (order == 1) {
      std::swap(items[0], items[1]);
      std::swap(cols[0], cols[1]);
    }
    const Image img = renderScene(makeScene(items, cols), cam, 200, 200);
    const std::uint32_t c = pixelAt(img, 100, 100);
    CHECK(red(c) > 100 && green(c) < 10);  // the nearer (red) cube hides the green one
  }

  // Every primitive is visible from all six directions (so the winding and culling are right).
  const Geometry shapes[] = {Geometry::box(0.5, 0.3, 0.2), Geometry::cylinder(0.2, 0.5), Geometry::sphere(0.25)};
  const double views[6][2] = {{0, 0}, {90, 0}, {180, 0}, {270, 0}, {0, 89}, {0, -89}};
  const double pxPerMetre = 160.0 / (2 * 4 * std::tan(17.5 * kPi / 180));
  // Projected area of each shape seen along the x axis, the y axis or the z axis (orthographic estimate).
  auto expectedPixels = [&](const Geometry& g, double az, double el) {
    const bool fromTop = std::fabs(el) > 80;
    const bool alongX = !fromTop && (static_cast<int>(az) % 180 == 0);
    double area = 0;
    switch (g.kind) {
      case Geometry::Kind::Box: area = fromTop ? 0.5 * 0.3 : (alongX ? 0.3 * 0.2 : 0.5 * 0.2); break;
      case Geometry::Kind::Cylinder: area = fromTop ? kPi * 0.2 * 0.2 : 0.4 * 0.5; break;
      case Geometry::Kind::Sphere: area = kPi * 0.25 * 0.25; break;
    }
    return area * pxPerMetre * pxPerMetre;
  };
  for (const Geometry& g : shapes) {
    for (const auto& v : views) {
      Camera c = cam;
      c.azimuthDeg = v[0];
      c.elevationDeg = v[1];
      const Image img = renderScene(makeScene({{g, {0, 0, 0}}}, {kGrey}), c, 160, 160);
      const std::uint32_t centre = pixelAt(img, 80, 80);
      if (centre == 0) std::fprintf(stderr, "  centre pixel empty (kind %d, az %g, el %g)\n", static_cast<int>(g.kind), v[0], v[1]);
      CHECK(centre != 0);
      const double ratio = static_cast<double>(covered(img)) / expectedPixels(g, v[0], v[1]);
      if (ratio < 0.9 || ratio > 1.25) {
        std::fprintf(stderr, "  coverage ratio %.2f (kind %d, az %g, el %g)\n", ratio, static_cast<int>(g.kind), v[0], v[1]);
      }
      CHECK(ratio > 0.9 && ratio < 1.25);
    }
  }

  // Background stays empty and shading gives different brightness to different faces.
  {
    Camera c = cam;
    c.azimuthDeg = -50;
    c.elevationDeg = 30;
    const Image img = renderScene(makeScene({{Geometry::box(0.6, 0.6, 0.6), {0, 0, 0}}}, {kGrey}), c, 200, 200);
    CHECK(pixelAt(img, 2, 2) == 0);
    std::set<std::uint32_t> shades;
    for (std::uint32_t p : img.pixels) {
      if (p) shades.insert(p);
    }
    CHECK(shades.size() == 3);  // top, front, side: one flat shade each
    for (std::uint32_t p : shades) CHECK(red(p) == green(p) && green(p) <= blue(p) + 0);
    CHECK((pixelAt(img, 100, 100) >> 24) == 0xFF);  // opaque
  }

  // Smooth shading: a sphere has many different shades.
  {
    const Image img = renderScene(makeScene({{Geometry::sphere(0.5), {0, 0, 0}}}, {kGrey}), cam, 200, 200);
    std::set<std::uint32_t> shades;
    for (std::uint32_t p : img.pixels) {
      if (p) shades.insert(p);
    }
    CHECK(shades.size() > 40);
  }

  // Near-plane clipping: a large floor that extends behind the camera still renders in front of it.
  {
    Camera c = cam;
    c.elevationDeg = 20;
    c.distance = 1.0;
    const Scene floorScene = makeScene({{Geometry::box(10, 10, 0.02), {0, 0, -0.4}}}, {kGrey});
    const Image img = renderScene(floorScene, c, 200, 200);
    CHECK(pixelAt(img, 100, 190) != 0);  // the floor fills the bottom of the picture
    CHECK(pixelAt(img, 100, 5) == 0);    // sky above the horizon
    CHECK(covered(img) > 5000);
  }

  // Degenerate requests do not crash.
  CHECK(covered(renderScene(Scene(), cam, 100, 100)) == 0);
  CHECK(renderScene(makeScene({{cube, {0, 0, 0}}}, {kGrey}), cam, 0, 0).pixels.empty());
  CHECK(covered(renderScene(makeScene({{cube, {0, 0, 0}}}, {kGrey}), cam, 1, 1)) == 0);

  // A real robot fills a sensible part of the picture.
  {
    const Scene sc = buildScene(buildDefault(RobotType::Hexapod));
    const Camera fitted = Camera::fit(sc, 1.5, -50, 25);
    const Image img = renderScene(sc, fitted, 600, 400);
    const std::size_t n = covered(img);
    CHECK(n > 3000 && n < 600u * 400u / 2);
  }
}

// Writes unusual configurations (polar offsets, prismatic joints, tilted links,
// cylindrical chassis, ...) as .xacro/.urdf pairs so that the Python test can
// check, with the real xacro tool, that every expression path expands correctly.
// The 3D scene in a plain text form that an independent program can check against the URDF:
//   joint <name> <value>        (posed files only: the joint values that were applied)
//   frame <link> r00 r01 r02 r10 r11 r12 r20 r21 r22 tx ty tz
//   bounds minx miny minz maxx maxy maxz
std::string sceneText(const Robot& r, const JointPose& pose = JointPose()) {
  const Scene s = buildScene(r, pose);
  std::ostringstream out;
  out << std::setprecision(12);
  for (const auto& jv : pose) out << "joint " << jv.first << ' ' << jv.second << '\n';
  for (const auto& f : s.frames) {
    out << "frame " << f.link;
    for (double v : f.pose.r) out << ' ' << v;
    out << ' ' << f.pose.t.x << ' ' << f.pose.t.y << ' ' << f.pose.t.z << '\n';
  }
  out << "bounds " << s.min.x << ' ' << s.min.y << ' ' << s.min.z << ' ' << s.max.x << ' ' << s.max.y
      << ' ' << s.max.z << '\n';
  return out.str();
}

bool writeVariant(const std::string& dir, const Robot& r) {
  if (hasErrors(validate(r))) {
    std::fprintf(stderr, "variant %s does not validate\n", r.name.c_str());
    return false;
  }
  const std::pair<std::string, std::string> files[] = {{dir + "/" + r.name + ".xacro", emitXacro(r)},
                                                       {dir + "/" + r.name + ".urdf", emitUrdf(r)},
                                                       {dir + "/" + r.name + ".scene", sceneText(r)},
                                                       {dir + "/" + r.name + ".posed.scene", sceneText(r, demoPose(r))}};
  for (const auto& f : files) {
    std::ofstream out(f.first, std::ios::binary);
    if (!out) {
      std::fprintf(stderr, "cannot write %s\n", f.first.c_str());
      return false;
    }
    out << f.second;
  }
  return true;
}

int dumpVariants(const std::string& dir) {
  std::filesystem::create_directories(dir);
  bool ok = true;
  for (RobotType t : allRobotTypes()) ok &= writeVariant(dir, buildDefault(t));

  ArmConfig arm;
  arm.name = "variant_arm_mixed";
  arm.links.clear();
  ArmLinkConfig l1;  // revolute about Z, cartesian
  l1.c = 0.06;
  ArmLinkConfig l2;  // revolute about Y, polar offset
  l2.axis = ArmAxis::Y;
  l2.mode = OffsetMode::Polar;
  l2.a = 0.1; l2.b = 45; l2.c = 0.2;
  ArmLinkConfig l3;  // prismatic along Z
  l3.type = ArmJointType::Prismatic;
  l3.axis = ArmAxis::Z;
  l3.a = 0; l3.b = 0; l3.c = 0.15; l3.lower = 0; l3.upper = 0.25;
  ArmLinkConfig l4;  // revolute about X, off-axis cartesian offset (tilted link)
  l4.axis = ArmAxis::X;
  l4.a = 0.1; l4.b = 0.05; l4.c = 0.2;
  arm.links = {l1, l2, l3, l4};
  arm.toolX = 0.05; arm.toolZ = 0.1;
  ok &= writeVariant(dir, buildArm(arm));

  ArmConfig coincident;
  coincident.name = "variant_arm_coincident";
  coincident.links = {l1, ArmLinkConfig{ArmJointType::Revolute, ArmAxis::Y, OffsetMode::Cartesian, 0, 0, 0, -90, 90, 0.5}};
  ok &= writeVariant(dir, buildArm(coincident));

  ArmConfig empty;
  empty.name = "variant_arm_no_links";
  empty.links.clear();
  ok &= writeVariant(dir, buildArm(empty));

  ChassisConfig round;
  round.shape = ChassisShape::Cylinder;
  round.radius = 0.22;
  round.height = 0.08;

  DiffDriveConfig dd;
  dd.name = "variant_diff_round";
  dd.chassis = round;
  ok &= writeVariant(dir, buildDiffDrive(dd));
  dd.name = "variant_diff_round_no_casters";
  dd.casters = false;
  dd.wheelX = 0.03;
  ok &= writeVariant(dir, buildDiffDrive(dd));

  FourWheelConfig fw;
  fw.name = "variant_four_wheel_round";
  fw.chassis = round;
  ok &= writeVariant(dir, buildFourWheel(fw));

  AckermannConfig ack;
  ack.name = "variant_ackermann_round";
  ack.chassis = round;
  ack.wheelbase = 0.45;
  ack.trackWidth = 0.32;
  ok &= writeVariant(dir, buildAckermann(ack));

  // Mecanum wheels in every kind of wheeled robot, with odd and even roller counts and mixed wheels.
  FourWheelConfig mecFour;
  mecFour.name = "variant_four_wheel_mecanum";
  mecFour.frontLeftWheel = mecFour.frontRightWheel = mecFour.rearLeftWheel = mecFour.rearRightWheel = WheelKind::Mecanum;
  ok &= writeVariant(dir, buildFourWheel(mecFour));
  mecFour.name = "variant_four_wheel_mecanum_mixed";
  mecFour.frontRightWheel = mecFour.rearLeftWheel = WheelKind::Standard;
  mecFour.mecanumRollers = 9;
  mecFour.chassis = round;
  ok &= writeVariant(dir, buildFourWheel(mecFour));

  DiffDriveConfig mecDiff;
  mecDiff.name = "variant_diff_mecanum_left";
  mecDiff.leftWheel = WheelKind::Mecanum;
  mecDiff.mecanumRollers = 5;
  mecDiff.wheelX = -0.04;  // behind the centre: the roller slant flips with the side of the centre line
  ok &= writeVariant(dir, buildDiffDrive(mecDiff));
  mecDiff.name = "variant_diff_mecanum_both";
  mecDiff.rightWheel = WheelKind::Mecanum;
  mecDiff.casters = false;
  mecDiff.mecanumRollers = 12;
  ok &= writeVariant(dir, buildDiffDrive(mecDiff));

  AckermannConfig mecAck;
  mecAck.name = "variant_ackermann_mecanum_front";
  mecAck.frontLeftWheel = mecAck.frontRightWheel = WheelKind::Mecanum;
  ok &= writeVariant(dir, buildAckermann(mecAck));
  mecAck.name = "variant_ackermann_mecanum_all";
  mecAck.rearLeftWheel = mecAck.rearRightWheel = WheelKind::Mecanum;
  mecAck.mecanumRollers = 7;
  ok &= writeVariant(dir, buildAckermann(mecAck));
  mecAck.name = "variant_ackermann_mecanum_rear_left";
  mecAck.frontLeftWheel = mecAck.frontRightWheel = mecAck.rearRightWheel = WheelKind::Standard;
  ok &= writeVariant(dir, buildAckermann(mecAck));

  // Sensors on all kinds of links: the tilted, prismatic, steered and spinning ones too.
  {
    Robot r = buildHexapod(HexapodConfig());
    r.name = "variant_sensors_hexapod";
    SensorConfig shin;
    shin.name = "shin_cam";
    shin.link = "leg1_tibia_link";
    shin.distance = 0.08;
    shin.offsetX = 0.02;
    shin.pitchDeg = -20;
    shin.yawDeg = 15;
    SensorConfig imu = defaultSensor(r, SensorType::Imu, {shin});
    SensorConfig gps = defaultSensor(r, SensorType::Gps, {shin, imu});
    gps.offsetX = 0.04;
    SensorConfig femur;
    femur.type = SensorType::Lidar;
    femur.name = "femur_lidar";
    femur.link = "leg4_femur_link";
    femur.distance = 0.06;
    femur.offsetY = -0.03;
    femur.rollDeg = 90;
    addSensors(r, {shin, imu, gps, femur});
    ok &= writeVariant(dir, r);
  }
  {
    Robot r = buildArm(arm);  // the mixed arm above: polar offsets, a prismatic joint, a tilted link
    r.name = "variant_sensors_arm";
    SensorConfig tool;
    tool.name = "wrist_cam";
    tool.link = "tool0";
    tool.offsetX = 0.03;
    tool.pitchDeg = 90;
    SensorConfig tilted;
    tilted.type = SensorType::Lidar;
    tilted.name = "tilted_lidar";
    tilted.link = "link_4";
    tilted.distance = 0.1;
    tilted.offsetX = 0.02;
    tilted.rollDeg = 30; tilted.pitchDeg = 45; tilted.yawDeg = -60;
    SensorConfig polar;
    polar.type = SensorType::Imu;
    polar.name = "link2_imu";
    polar.link = "link_2";
    polar.distance = 0.05;
    SensorConfig slider;
    slider.type = SensorType::Odometry;
    slider.name = "slider_odom";
    slider.link = "link_3";
    slider.distance = 0.07;
    addSensors(r, {tool, tilted, polar, slider});
    ok &= writeVariant(dir, r);
  }
  {
    Robot r = buildDefault(RobotType::Biped);
    r.name = "variant_sensors_biped";
    SensorConfig head = defaultSensor(r, SensorType::Camera, {});
    head.pitchDeg = 15;
    SensorConfig shank;
    shank.type = SensorType::Imu;
    shank.name = "left_shank_imu";
    shank.link = "left_shank_link";
    shank.distance = 0.1;
    shank.offsetX = 0.02;
    addSensors(r, {head, shank});
    ok &= writeVariant(dir, r);
  }
  {
    FourWheelConfig fw;
    fw.frontLeftWheel = fw.rearRightWheel = WheelKind::Mecanum;
    fw.mecanumRollers = 7;
    Robot r = buildFourWheel(fw);
    r.name = "variant_sensors_mecanum";
    SensorConfig hub;
    hub.type = SensorType::Odometry;
    hub.name = "hub_odom";
    hub.link = "front_left_wheel_link";
    hub.distance = 0.02;
    SensorConfig rear;
    rear.type = SensorType::Gps;
    rear.name = "rear_hub_gps";
    rear.link = "rear_right_wheel_link";
    rear.distance = -0.02;
    rear.offsetZ = 0.01;
    SensorConfig top = defaultSensor(r, SensorType::Lidar, {hub, rear});
    top.offsetX = 0.05;
    addSensors(r, {hub, rear, top});
    ok &= writeVariant(dir, r);
  }
  {
    AckermannConfig ac;
    ac.chassis = round;
    Robot r = buildAckermann(ac);
    r.name = "variant_sensors_ackermann";
    SensorConfig steer;  // on a steered wheel: it turns with the steering and rolls with the wheel
    steer.type = SensorType::Camera;
    steer.name = "steer_cam";
    steer.link = "front_left_steer_link";
    steer.distance = 0.01;
    steer.offsetX = 0.05;
    SensorConfig wheel;
    wheel.type = SensorType::Imu;
    wheel.name = "wheel_imu";
    wheel.link = "front_right_wheel_link";
    wheel.distance = 0.01;
    wheel.rollDeg = 20;
    SensorConfig body = defaultSensor(r, SensorType::Lidar, {steer, wheel});
    addSensors(r, {steer, wheel, body});
    ok &= writeVariant(dir, r);
  }
  {
    Robot r = buildDefault(RobotType::DiffDrive);
    r.name = "variant_sensors_all_types";
    std::vector<SensorConfig> list;
    for (SensorType t : allSensorTypes()) {
      SensorConfig s = defaultSensor(r, t, list);
      s.offsetX = -0.1 + 0.05 * static_cast<double>(list.size());
      s.yawDeg = 20.0 * static_cast<double>(list.size());
      list.push_back(s);
    }
    SensorConfig second = defaultSensor(r, SensorType::Camera, list);
    second.link = "left_wheel_link";
    second.distance = -0.015;
    list.push_back(second);
    addSensors(r, list);
    ok &= writeVariant(dir, r);
  }
  {
    Robot r = buildDefault(RobotType::Quadruped);
    r.name = "variant_sensors_quadruped";
    SensorConfig s = defaultSensor(r, SensorType::Camera, {});
    s.offsetX = 0.15;
    addSensors(r, {s});
    ok &= writeVariant(dir, r);
  }

  HexapodConfig hex;
  hex.name = "variant_hexapod_big";
  hex.coxaLength = 0.1; hex.femurLength = 0.2; hex.tibiaLength = 0.3; hex.bodyRadius = 0.25;
  ok &= writeVariant(dir, buildHexapod(hex));
  return ok ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc == 3 && std::string(argv[1]) == "--dump") return dumpVariants(argv[2]);
  testNum();
  testInertia();
  testDefaults();
  testMaterialsUsed();
  testMacrosInXacro();
  testHexapodGeometry();
  testArm();
  testWheeled();
  testLegged();
  testValidation();
  testUrdfValues();
  testTessellation();
  testTransforms();
  testScene();
  testMovableJoints();
  testJointMotion();
  testPose();
  testMecanum();
  testLinkAxis();
  testSensorNames();
  testDefaultSensor();
  testAttachableLinks();
  testSensorPlacement();
  testSensorEmitters();
  testSensorMaterials();
  testSensorIssues();
  testSensorEverywhere();
  testSensorMany();
  testSensorOnMecanum();
  testCamera();
  testRenderer();
  std::printf("%d checks, %d failed\n", g_checks, g_failed);
  return g_failed == 0 ? 0 : 1;
}
