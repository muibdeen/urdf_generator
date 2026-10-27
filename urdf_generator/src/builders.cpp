// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include "urdf_generator/builders.hpp"

#include <algorithm>
#include <array>
#include <string>

namespace urdfgen {
namespace {

using Axis3 = std::array<double, 3>;
const Axis3 kAxisX{{1, 0, 0}};
const Axis3 kAxisY{{0, 1, 0}};
const Axis3 kAxisZ{{0, 0, 1}};

double toRad(double deg) { return deg * kPi / 180.0; }

Link bodyLink(const std::string& name, const Origin& o, const Geometry& g,
              const std::string& material, const Num& mass) {
  Link l;
  l.name = name;
  l.hasBody = true;
  l.origin = o;
  l.geometry = g;
  l.material = material;
  l.mass = mass;
  return l;
}

Link frameLink(const std::string& name) {
  Link l;
  l.name = name;
  l.hasBody = false;
  return l;
}

Joint fixedJoint(const std::string& name, const std::string& parent, const std::string& child,
                 const Origin& o) {
  Joint j;
  j.name = name;
  j.type = JointType::Fixed;
  j.parent = parent;
  j.child = child;
  j.origin = o;
  return j;
}

Joint continuousJoint(const std::string& name, const std::string& parent, const std::string& child,
                      const Origin& o, const Axis3& axis) {
  Joint j = fixedJoint(name, parent, child, o);
  j.type = JointType::Continuous;
  j.axis = axis;
  return j;
}

Joint limitedJoint(const std::string& name, JointType type, const std::string& parent,
                   const std::string& child, const Origin& o, const Axis3& axis, const Num& lower,
                   const Num& upper, const Num& effort, const Num& velocity) {
  Joint j = fixedJoint(name, parent, child, o);
  j.type = type;
  j.axis = axis;
  j.lower = lower;
  j.upper = upper;
  j.effort = effort;
  j.velocity = velocity;
  return j;
}

// Cylinder placements. A URDF cylinder's axis is local +z.
Origin cylinderAlongX(const Num& length) { return Origin(length / 2, 0, 0, 0, Num::pi() / 2, 0); }
Origin cylinderAlongNegZ(const Num& length) { return Origin(0, 0, -length / 2); }

// ---------------------------------------------------------------- wheeled robots

/// Registers the chassis properties and adds base_link. Returns half the
/// chassis extent along x (used to place casters).
Num addChassis(Robot& r, const ChassisConfig& c) {
  r.section("Chassis");
  Geometry geom;
  Num halfX;
  if (c.shape == ChassisShape::Box) {
    Num len = r.prop("chassis_length", c.length, "x size [m]");
    Num wid = r.prop("chassis_width", c.width, "y size [m]");
    Num hgt = r.prop("chassis_height", c.height, "z size [m]");
    geom = Geometry::box(len, wid, hgt);
    halfX = len / 2;
  } else {
    Num rad = r.prop("chassis_radius", c.radius, "[m]");
    Num hgt = r.prop("chassis_height", c.height, "z size [m]");
    geom = Geometry::cylinder(rad, hgt);
    halfX = rad;
  }
  Num mass = r.prop("chassis_mass", c.mass, "[kg]");
  r.root.links.push_back(bodyLink("base_link", Origin(), geom, "chassis_grey", mass));
  return halfX;
}

void addWheeledMaterials(Robot& r, bool withCasterMaterial = false, bool withRollerMaterial = false) {
  r.addMaterial("chassis_grey", 0.50, 0.55, 0.62);
  r.addMaterial("wheel_black", 0.10, 0.10, 0.10);
  if (withCasterMaterial) r.addMaterial("caster_grey", 0.65, 0.65, 0.65);
  if (withRollerMaterial) r.addMaterial("roller_grey", 0.72, 0.74, 0.78);
}

/// Continuous wheel rolling about its local +y axis.
void addDriveWheelMacro(Robot& r, const Num& wheelR, const Num& wheelW, const Num& wheelM) {
  MacroDef d;
  d.name = "drive_wheel";
  d.strParams = {"prefix", "parent"};
  d.numParams = {"mount_x", "mount_y", "mount_z"};
  d.body = [=](Part& p, const MacroArgs& a) {
    const std::string pre = a.str("prefix");
    p.links.push_back(bodyLink(pre + "_wheel_link", Origin(0, 0, 0, Num::pi() / 2, 0, 0),
                               Geometry::cylinder(wheelR, wheelW), "wheel_black", wheelM));
    p.joints.push_back(continuousJoint(
        pre + "_wheel_joint", a.str("parent"), pre + "_wheel_link",
        Origin(a.num("mount_x"), a.num("mount_y"), a.num("mount_z")), kAxisY));
  };
  r.addMacro(d);
}

void instantiateWheel(Robot& r, const std::string& macro, const std::string& prefix,
                      const Num& x, const Num& y, const Num& z) {
  MacroArgs a;
  a.setStr("prefix", prefix).setStr("parent", "base_link");
  a.setNum("mount_x", x).setNum("mount_y", y).setNum("mount_z", z);
  r.instantiate(macro, a);
}

// ------------------------------------------------------------------ mecanum wheels
//
// A mecanum wheel is a hub (a cylinder on the wheel joint) with N barrel-shaped rollers fixed
// around its rim. Each roller is a cylinder whose axis lies in the tangent plane of the rim and
// is turned 45 degrees away from the wheel axle. The rollers are fixed to the hub (no joints of
// their own), so they describe the shape and mass of the wheel, not its sideways freedom.
//
// In the wheel frame the axle is y. A roller sits at angle phi from the top of the wheel
// (position (R sin phi, 0, R cos phi)); its axis is  a = (-h c/sqrt2, 1/sqrt2, h s/sqrt2)  with
// s = sin phi, c = cos phi and the handedness h = +1 or -1. At the bottom (phi = pi) that is
// (h, 1)/sqrt2 in the (x, y) plane seen from above: the axis of the roller on the ground. For
// h = +1 that is the front-left / rear-right wheel of the usual layout, for h = -1 the other
// two. The cylinder orientation (rpy = 0, acos(a_z), atan2(a_y, a_x)) turns its z axis onto a.

constexpr double kSqrt2 = 1.41421356237309505;
constexpr int kMinRollers = 3, kMaxRollers = 16;

struct Mecanum {
  Num hubR, hubW, hubM, rollerR, rollerL, rollerM, ringR;
  int rollers = 8;
};

/// The handedness of a mecanum wheel mounted at (x, y): +1 for the front-left and rear-right
/// wheels, -1 for the front-right and rear-left ones. A wheel exactly on the middle axis counts
/// as a front wheel.
double handedness(double x, double y) { return ((x >= 0) == (y > 0)) ? 1.0 : -1.0; }

Mecanum addMecanumProps(Robot& r, const Num& wheelR, const Num& wheelW, const Num& wheelM, int rollers) {
  Mecanum m;
  if (!(wheelR.value() - wheelW.value() * 0.45 > 0.0)) {
    r.buildIssues.push_back({Issue::Severity::Error,
                             "A mecanum wheel must be narrower than about 2.2 times its radius, "
                             "otherwise the rollers do not fit round the hub."});
  }
  m.rollers = std::max(kMinRollers, std::min(kMaxRollers, rollers));
  r.section("Mecanum wheels (a hub with " + std::to_string(m.rollers) + " rollers at 45 degrees)");
  m.hubR = r.derived("hub_radius", wheelR - wheelW * 0.45, "[m] the rollers sit on the hub rim");
  m.hubW = r.derived("hub_width", wheelW * 0.6, "[m]");
  m.hubM = r.derived("hub_mass", wheelM * 0.6, "[kg]");
  m.rollerR = r.derived("roller_radius", wheelW * 0.25, "[m]");
  m.rollerL = r.derived("roller_length", wheelW * kSqrt2 - m.rollerR * 2.0,
                        "[m] the slanted roller ends just reach the sides of the wheel");
  m.rollerM = r.derived("roller_mass", wheelM * 0.4 / static_cast<double>(m.rollers), "[kg] each");
  m.ringR = r.derived("roller_ring_radius", wheelR - m.rollerR,
                      "the roller axes lie on this circle around the axle [m]");
  return m;
}

/// The hub, its wheel joint (child of `parent` at `mount`) and the rollers.
void appendMecanumWheel(Part& p, const Mecanum& m, const std::string& pre, const std::string& parent,
                        const Origin& mount, const Num& hand) {
  const std::string hub = pre + "_wheel_link";
  p.links.push_back(bodyLink(hub, Origin(0, 0, 0, Num::pi() / 2, 0, 0),
                             Geometry::cylinder(m.hubR, m.hubW), "wheel_black", m.hubM));
  p.joints.push_back(continuousJoint(pre + "_wheel_joint", parent, hub, mount, kAxisY));
  for (int k = 0; k < m.rollers; ++k) {
    // Start at the bottom so that one roller always touches the ground.
    const double phi = kPi + 2.0 * kPi * k / m.rollers;
    const Num s = nsin(Num(phi)), c = ncos(Num(phi));
    const Num yaw = natan2(Num(1.0), -hand * c);
    const Num pitch = nacos(hand * s / kSqrt2);
    const std::string name = pre + "_roller_" + std::to_string(k + 1);
    Link roller = bodyLink(name + "_link", Origin(), Geometry::cylinder(m.rollerR, m.rollerL),
                           "roller_grey", m.rollerM);
    roller.attachable = false;
    p.links.push_back(roller);
    p.joints.push_back(fixedJoint(name + "_joint", hub, name + "_link",
                                  Origin(m.ringR * s, 0, m.ringR * c, 0, pitch, yaw)));
  }
}

void addMecanumWheelMacro(Robot& r, const Mecanum& m) {
  MacroDef d;
  d.name = "mecanum_wheel";
  d.strParams = {"prefix", "parent"};
  d.numParams = {"mount_x", "mount_y", "mount_z", "hand"};
  d.body = [=](Part& p, const MacroArgs& a) {
    appendMecanumWheel(p, m, a.str("prefix"), a.str("parent"),
                       Origin(a.num("mount_x"), a.num("mount_y"), a.num("mount_z")), a.num("hand"));
  };
  r.addMacro(d);
}

void instantiateMecanumWheel(Robot& r, const std::string& macro, const std::string& prefix,
                             const Num& x, const Num& y, const Num& z, double hand) {
  MacroArgs a;
  a.setStr("prefix", prefix).setStr("parent", "base_link");
  a.setNum("mount_x", x).setNum("mount_y", y).setNum("mount_z", z).setNum("hand", Num(hand));
  r.instantiate(macro, a);
}

bool isMecanum(WheelKind k) { return k == WheelKind::Mecanum; }

}  // namespace

// ============================================================ differential drive

Robot buildDiffDrive(const DiffDriveConfig& c) {
  Robot r(c.name);
  const bool anyMecanum = isMecanum(c.leftWheel) || isMecanum(c.rightWheel);
  const bool anyStandard = !isMecanum(c.leftWheel) || !isMecanum(c.rightWheel);
  addWheeledMaterials(r, c.casters, anyMecanum);
  r.root.links.push_back(frameLink("base_footprint"));
  const Num halfX = addChassis(r, c.chassis);

  r.section("Wheels");
  Num wheelR = r.prop("wheel_radius", c.wheelRadius, "[m]");
  Num wheelW = r.prop("wheel_width", c.wheelWidth, "[m]");
  Num wheelM = r.prop("wheel_mass", c.wheelMass, "[kg]");
  Num wx = r.prop("wheel_x", c.wheelX, "left wheel centre, x from chassis centre [m]");
  Num wy = r.prop("wheel_y", c.wheelY, "left wheel centre, y from chassis centre [m] (right wheel mirrors)");
  Num wz = r.prop("wheel_z", c.wheelZ, "wheel centre, z from chassis centre [m]");

  r.root.joints.push_back(fixedJoint("base_footprint_joint", "base_footprint", "base_link",
                                     Origin(0, 0, wheelR - wz)));

  if (anyMecanum) {
    const Mecanum mec = addMecanumProps(r, wheelR, wheelW, wheelM, c.mecanumRollers);
    addMecanumWheelMacro(r, mec);
  }
  if (anyStandard) addDriveWheelMacro(r, wheelR, wheelW, wheelM);
  if (isMecanum(c.leftWheel)) {
    instantiateMecanumWheel(r, "mecanum_wheel", "left", wx, wy, wz, handedness(c.wheelX, c.wheelY));
  } else {
    instantiateWheel(r, "drive_wheel", "left", wx, wy, wz);
  }
  if (isMecanum(c.rightWheel)) {
    instantiateMecanumWheel(r, "mecanum_wheel", "right", wx, -wy, wz, handedness(c.wheelX, -c.wheelY));
  } else {
    instantiateWheel(r, "drive_wheel", "right", wx, -wy, wz);
  }

  if (c.casters) {
    r.section("Casters (passive balls, front and rear)");
    Num casterR = r.prop("caster_radius", c.casterRadius, "[m]");
    Num casterM = r.prop("caster_mass", c.casterMass, "[kg]");

    MacroDef d;
    d.name = "caster";
    d.strParams = {"prefix", "parent"};
    d.numParams = {"mount_x", "mount_z"};
    d.body = [=](Part& p, const MacroArgs& a) {
      const std::string pre = a.str("prefix");
      p.links.push_back(bodyLink(pre + "_caster_link", Origin(), Geometry::sphere(casterR),
                                 "caster_grey", casterM));
      p.joints.push_back(fixedJoint(pre + "_caster_joint", a.str("parent"), pre + "_caster_link",
                                    Origin(a.num("mount_x"), 0, a.num("mount_z"))));
    };
    r.addMacro(d);

    const Num cx = halfX - casterR;
    const Num cz = wz - wheelR + casterR;  // ball touches the same ground plane as the wheels
    MacroArgs front, rear;
    front.setStr("prefix", "front").setStr("parent", "base_link").setNum("mount_x", cx).setNum("mount_z", cz);
    rear.setStr("prefix", "rear").setStr("parent", "base_link").setNum("mount_x", -cx).setNum("mount_z", cz);
    r.instantiate("caster", front);
    r.instantiate("caster", rear);
  }
  return r;
}

// ================================================================== 4-wheeled

Robot buildFourWheel(const FourWheelConfig& c) {
  Robot r(c.name);
  const WheelKind kinds[4] = {c.frontLeftWheel, c.frontRightWheel, c.rearLeftWheel, c.rearRightWheel};
  const bool anyMecanum = std::any_of(kinds, kinds + 4, isMecanum);
  const bool anyStandard = std::any_of(kinds, kinds + 4, [](WheelKind k) { return !isMecanum(k); });
  addWheeledMaterials(r, false, anyMecanum);
  r.root.links.push_back(frameLink("base_footprint"));
  addChassis(r, c.chassis);

  r.section("Wheels");
  Num wheelR = r.prop("wheel_radius", c.wheelRadius, "[m]");
  Num wheelW = r.prop("wheel_width", c.wheelWidth, "[m]");
  Num wheelM = r.prop("wheel_mass", c.wheelMass, "[kg]");
  Num wx = r.prop("wheel_x", c.wheelX, "front-left wheel centre, x from chassis centre [m] (others mirror)");
  Num wy = r.prop("wheel_y", c.wheelY, "front-left wheel centre, y from chassis centre [m] (others mirror)");
  Num wz = r.prop("wheel_z", c.wheelZ, "wheel centre, z from chassis centre [m]");

  r.root.joints.push_back(fixedJoint("base_footprint_joint", "base_footprint", "base_link",
                                     Origin(0, 0, wheelR - wz)));

  if (anyMecanum) {
    const Mecanum mec = addMecanumProps(r, wheelR, wheelW, wheelM, c.mecanumRollers);
    addMecanumWheelMacro(r, mec);
  }
  if (anyStandard) addDriveWheelMacro(r, wheelR, wheelW, wheelM);
  struct Mount { const char* prefix; WheelKind kind; double sx, sy; Num x, y; };
  const Mount mounts[4] = {{"front_left", c.frontLeftWheel, 1, 1, wx, wy},
                           {"front_right", c.frontRightWheel, 1, -1, wx, -wy},
                           {"rear_left", c.rearLeftWheel, -1, 1, -wx, wy},
                           {"rear_right", c.rearRightWheel, -1, -1, -wx, -wy}};
  for (const Mount& m : mounts) {
    if (isMecanum(m.kind)) {
      instantiateMecanumWheel(r, "mecanum_wheel", m.prefix, m.x, m.y, wz,
                              handedness(m.sx * c.wheelX, m.sy * c.wheelY));
    } else {
      instantiateWheel(r, "drive_wheel", m.prefix, m.x, m.y, wz);
    }
  }
  return r;
}

// ================================================================== Ackermann

Robot buildAckermann(const AckermannConfig& c) {
  Robot r(c.name);
  const bool frontMecanum = isMecanum(c.frontLeftWheel) || isMecanum(c.frontRightWheel);
  const bool frontStandard = !isMecanum(c.frontLeftWheel) || !isMecanum(c.frontRightWheel);
  const bool rearMecanum = isMecanum(c.rearLeftWheel) || isMecanum(c.rearRightWheel);
  const bool rearStandard = !isMecanum(c.rearLeftWheel) || !isMecanum(c.rearRightWheel);
  const bool anyMecanum = frontMecanum || rearMecanum;
  addWheeledMaterials(r, false, anyMecanum);
  r.root.links.push_back(frameLink("base_footprint"));
  addChassis(r, c.chassis);

  r.section("Wheels");
  Num wheelR = r.prop("wheel_radius", c.wheelRadius, "[m]");
  Num wheelW = r.prop("wheel_width", c.wheelWidth, "[m]");
  Num wheelM = r.prop("wheel_mass", c.wheelMass, "[kg]");
  Num wheelZ = r.prop("wheel_z", c.wheelZ, "wheel centre, z from chassis centre [m]");

  r.section("Ackermann geometry");
  Num wheelbase = r.prop("wheelbase", c.wheelbase, "front axle at +x/2, rear axle at -x/2 [m]");
  Num track = r.prop("track_width", c.trackWidth, "left-to-right wheel distance [m]");
  Num steerLim = r.prop("steer_limit", toRad(c.steerLimitDeg),
                        "+/- steering angle [rad] (" + Num::format(c.steerLimitDeg) + " deg)");
  Num steerEffort = r.prop("steer_effort", c.steerEffort, "[N m]");
  Num steerVel = r.prop("steer_velocity", c.steerVelocity, "[rad/s]");
  Num knuckleR = r.prop("knuckle_radius", 0.015, "steering knuckle ball [m]");
  Num knuckleM = r.prop("knuckle_mass", 0.05, "[kg]");

  Mecanum mec;
  if (anyMecanum) mec = addMecanumProps(r, wheelR, wheelW, wheelM, c.mecanumRollers);

  r.root.joints.push_back(fixedJoint("base_footprint_joint", "base_footprint", "base_link",
                                     Origin(0, 0, wheelR - wheelZ)));

  if (rearStandard) addDriveWheelMacro(r, wheelR, wheelW, wheelM);
  if (rearMecanum) addMecanumWheelMacro(r, mec);

  // Steered front wheel: knuckle link on a vertical steering joint, wheel on the knuckle.
  auto steerJoint = [=](Part& p, const MacroArgs& a, const std::string& pre) {
    p.links.push_back(bodyLink(pre + "_steer_link", Origin(), Geometry::sphere(knuckleR),
                               "chassis_grey", knuckleM));
    p.joints.push_back(limitedJoint(
        pre + "_steer_joint", JointType::Revolute, a.str("parent"), pre + "_steer_link",
        Origin(a.num("mount_x"), a.num("mount_y"), a.num("mount_z")), kAxisZ, -steerLim, steerLim,
        steerEffort, steerVel));
  };
  if (frontStandard) {
    MacroDef d;
    d.name = "steered_wheel";
    d.strParams = {"prefix", "parent"};
    d.numParams = {"mount_x", "mount_y", "mount_z"};
    d.body = [=](Part& p, const MacroArgs& a) {
      const std::string pre = a.str("prefix");
      steerJoint(p, a, pre);
      p.links.push_back(bodyLink(pre + "_wheel_link", Origin(0, 0, 0, Num::pi() / 2, 0, 0),
                                 Geometry::cylinder(wheelR, wheelW), "wheel_black", wheelM));
      p.joints.push_back(continuousJoint(pre + "_wheel_joint", pre + "_steer_link",
                                         pre + "_wheel_link", Origin(), kAxisY));
    };
    r.addMacro(d);
  }
  if (frontMecanum) {
    MacroDef d;
    d.name = "steered_mecanum_wheel";
    d.strParams = {"prefix", "parent"};
    d.numParams = {"mount_x", "mount_y", "mount_z", "hand"};
    d.body = [=](Part& p, const MacroArgs& a) {
      const std::string pre = a.str("prefix");
      steerJoint(p, a, pre);
      appendMecanumWheel(p, mec, pre, pre + "_steer_link", Origin(), a.num("hand"));
    };
    r.addMacro(d);
  }

  const Num halfBase = wheelbase / 2;
  const Num halfTrack = track / 2;
  const double hx = c.wheelbase / 2, hy = c.trackWidth / 2;
  if (isMecanum(c.frontLeftWheel)) {
    instantiateMecanumWheel(r, "steered_mecanum_wheel", "front_left", halfBase, halfTrack, wheelZ, handedness(hx, hy));
  } else {
    instantiateWheel(r, "steered_wheel", "front_left", halfBase, halfTrack, wheelZ);
  }
  if (isMecanum(c.frontRightWheel)) {
    instantiateMecanumWheel(r, "steered_mecanum_wheel", "front_right", halfBase, -halfTrack, wheelZ, handedness(hx, -hy));
  } else {
    instantiateWheel(r, "steered_wheel", "front_right", halfBase, -halfTrack, wheelZ);
  }
  if (isMecanum(c.rearLeftWheel)) {
    instantiateMecanumWheel(r, "mecanum_wheel", "rear_left", -halfBase, halfTrack, wheelZ, handedness(-hx, hy));
  } else {
    instantiateWheel(r, "drive_wheel", "rear_left", -halfBase, halfTrack, wheelZ);
  }
  if (isMecanum(c.rearRightWheel)) {
    instantiateMecanumWheel(r, "mecanum_wheel", "rear_right", -halfBase, -halfTrack, wheelZ, handedness(-hx, -hy));
  } else {
    instantiateWheel(r, "drive_wheel", "rear_right", -halfBase, -halfTrack, wheelZ);
  }
  return r;
}

// ==================================================================== hexapod

Robot buildHexapod(const HexapodConfig& c) {
  Robot r(c.name);
  r.addMaterial("body_grey", 0.55, 0.57, 0.62);
  r.addMaterial("leg_orange", 0.95, 0.55, 0.15);
  r.addMaterial("foot_dark", 0.20, 0.20, 0.22);

  r.section("Body");
  Num bodyR = r.prop("body_radius", c.bodyRadius, "distance of the six leg mounts from the centre [m]");
  Num bodyH = r.prop("body_height", c.bodyHeight, "[m]");
  Num bodyM = r.prop("body_mass", c.bodyMass, "[kg]");

  r.section("Legs (3 DOF each: coxa yaw, femur pitch, tibia pitch)");
  Num coxaL = r.prop("coxa_length", c.coxaLength, "[m]");
  Num femurL = r.prop("femur_length", c.femurLength, "[m]");
  Num tibiaL = r.prop("tibia_length", c.tibiaLength, "[m]");
  Num legR = r.prop("leg_radius", c.legRadius, "[m]");
  Num coxaM = r.prop("coxa_mass", c.coxaMass, "[kg]");
  Num femurM = r.prop("femur_mass", c.femurMass, "[kg]");
  Num tibiaM = r.prop("tibia_mass", c.tibiaMass, "[kg]");
  Num footM = r.prop("foot_mass", c.footMass, "[kg]");

  r.section("Joints");
  Num coxaLim = r.prop("coxa_limit", toRad(c.coxaLimitDeg), "+/- [rad] (" + Num::format(c.coxaLimitDeg) + " deg)");
  Num femurLim = r.prop("femur_limit", toRad(c.femurLimitDeg), "+/- [rad] (" + Num::format(c.femurLimitDeg) + " deg)");
  Num tibiaLim = r.prop("tibia_limit", toRad(c.tibiaLimitDeg), "+/- [rad] (" + Num::format(c.tibiaLimitDeg) + " deg)");
  Num effort = r.prop("joint_effort", c.jointEffort, "[N m]");
  Num vel = r.prop("joint_velocity", c.jointVelocity, "[rad/s]");

  r.root.links.push_back(bodyLink("base_link", Origin(), Geometry::cylinder(bodyR, bodyH), "body_grey", bodyM));

  MacroDef d;
  d.name = "hexapod_leg";
  d.strParams = {"prefix", "parent"};
  d.numParams = {"mount_x", "mount_y", "mount_yaw"};
  d.body = [=](Part& p, const MacroArgs& a) {
    const std::string pre = a.str("prefix");
    const std::string coxa = pre + "_coxa_link", femur = pre + "_femur_link",
                      tibia = pre + "_tibia_link", foot = pre + "_foot_link";
    // coxa and femur run outward along +x; the tibia hangs down along -z.
    p.links.push_back(bodyLink(coxa, cylinderAlongX(coxaL), Geometry::cylinder(legR, coxaL), "leg_orange", coxaM));
    p.links.push_back(bodyLink(femur, cylinderAlongX(femurL), Geometry::cylinder(legR, femurL), "leg_orange", femurM));
    p.links.push_back(bodyLink(tibia, cylinderAlongNegZ(tibiaL), Geometry::cylinder(legR, tibiaL), "leg_orange", tibiaM));
    p.links.push_back(bodyLink(foot, Origin(), Geometry::sphere(legR * 1.5), "foot_dark", footM));

    p.joints.push_back(limitedJoint(
        pre + "_coxa_joint", JointType::Revolute, a.str("parent"), coxa,
        Origin(a.num("mount_x"), a.num("mount_y"), 0, 0, 0, a.num("mount_yaw")), kAxisZ,
        -coxaLim, coxaLim, effort, vel));
    p.joints.push_back(limitedJoint(pre + "_femur_joint", JointType::Revolute, coxa, femur,
                                    Origin(coxaL, 0, 0), kAxisY, -femurLim, femurLim, effort, vel));
    p.joints.push_back(limitedJoint(pre + "_tibia_joint", JointType::Revolute, femur, tibia,
                                    Origin(femurL, 0, 0), kAxisY, -tibiaLim, tibiaLim, effort, vel));
    p.joints.push_back(fixedJoint(pre + "_foot_joint", tibia, foot, Origin(0, 0, -tibiaL)));
  };
  r.addMacro(d);

  // Six legs spaced 60 degrees apart, leg1 straight ahead (+x), counter-clockwise.
  for (int k = 0; k < 6; ++k) {
    const Num ang = Num::deg(60.0 * k);
    MacroArgs a;
    a.setStr("prefix", "leg" + std::to_string(k + 1)).setStr("parent", "base_link");
    a.setNum("mount_x", bodyR * ncos(ang)).setNum("mount_y", bodyR * nsin(ang)).setNum("mount_yaw", ang);
    r.instantiate("hexapod_leg", a);
  }
  return r;
}

// =================================================================== quadruped

Robot buildQuadruped(const QuadrupedConfig& c) {
  Robot r(c.name);
  r.addMaterial("body_grey", 0.55, 0.57, 0.62);
  r.addMaterial("leg_orange", 0.95, 0.55, 0.15);
  r.addMaterial("foot_dark", 0.20, 0.20, 0.22);

  r.section("Body");
  Num bodyL = r.prop("body_length", c.bodyLength, "x size [m]");
  Num bodyW = r.prop("body_width", c.bodyWidth, "y size [m]");
  Num bodyH = r.prop("body_height", c.bodyHeight, "z size [m]");
  Num bodyM = r.prop("body_mass", c.bodyMass, "[kg]");
  Num mountX = r.prop("leg_mount_x", c.legMountX, "hip distance from centre along x [m]");
  Num mountY = r.prop("leg_mount_y", c.legMountY, "hip distance from centre along y [m]");

  r.section("Legs (3 DOF each: coxa roll, femur pitch, tibia pitch)");
  Num coxaL = r.prop("coxa_length", c.coxaLength, "sideways hip link [m]");
  Num femurL = r.prop("femur_length", c.femurLength, "thigh [m]");
  Num tibiaL = r.prop("tibia_length", c.tibiaLength, "shin [m]");
  Num legR = r.prop("leg_radius", c.legRadius, "[m]");
  Num coxaM = r.prop("coxa_mass", c.coxaMass, "[kg]");
  Num femurM = r.prop("femur_mass", c.femurMass, "[kg]");
  Num tibiaM = r.prop("tibia_mass", c.tibiaMass, "[kg]");
  Num footM = r.prop("foot_mass", c.footMass, "[kg]");

  r.section("Joints");
  Num coxaLim = r.prop("coxa_limit", toRad(c.coxaLimitDeg), "+/- [rad] (" + Num::format(c.coxaLimitDeg) + " deg)");
  Num femurLim = r.prop("femur_limit", toRad(c.femurLimitDeg), "+/- [rad] (" + Num::format(c.femurLimitDeg) + " deg)");
  Num tibiaLim = r.prop("tibia_limit", toRad(c.tibiaLimitDeg), "+/- [rad] (" + Num::format(c.tibiaLimitDeg) + " deg)");
  Num effort = r.prop("joint_effort", c.jointEffort, "[N m]");
  Num vel = r.prop("joint_velocity", c.jointVelocity, "[rad/s]");

  r.root.links.push_back(bodyLink("base_link", Origin(), Geometry::box(bodyL, bodyW, bodyH), "body_grey", bodyM));

  MacroDef d;
  d.name = "quadruped_leg";
  d.strParams = {"prefix", "parent"};
  d.numParams = {"mount_x", "mount_y", "side"};  // side = +1 (left) or -1 (right)
  d.body = [=](Part& p, const MacroArgs& a) {
    const std::string pre = a.str("prefix");
    const std::string coxa = pre + "_coxa_link", femur = pre + "_femur_link",
                      tibia = pre + "_tibia_link", foot = pre + "_foot_link";
    const Num side = a.num("side");
    // coxa runs sideways along y, femur and tibia hang down along -z.
    p.links.push_back(bodyLink(coxa, Origin(0, side * coxaL / 2, 0, Num::pi() / 2, 0, 0),
                               Geometry::cylinder(legR, coxaL), "leg_orange", coxaM));
    p.links.push_back(bodyLink(femur, cylinderAlongNegZ(femurL), Geometry::cylinder(legR, femurL), "leg_orange", femurM));
    p.links.push_back(bodyLink(tibia, cylinderAlongNegZ(tibiaL), Geometry::cylinder(legR, tibiaL), "leg_orange", tibiaM));
    p.links.push_back(bodyLink(foot, Origin(), Geometry::sphere(legR * 1.5), "foot_dark", footM));

    p.joints.push_back(limitedJoint(pre + "_coxa_joint", JointType::Revolute, a.str("parent"), coxa,
                                    Origin(a.num("mount_x"), a.num("mount_y"), 0), kAxisX,
                                    -coxaLim, coxaLim, effort, vel));
    p.joints.push_back(limitedJoint(pre + "_femur_joint", JointType::Revolute, coxa, femur,
                                    Origin(0, side * coxaL, 0), kAxisY, -femurLim, femurLim, effort, vel));
    p.joints.push_back(limitedJoint(pre + "_tibia_joint", JointType::Revolute, femur, tibia,
                                    Origin(0, 0, -femurL), kAxisY, -tibiaLim, tibiaLim, effort, vel));
    p.joints.push_back(fixedJoint(pre + "_foot_joint", tibia, foot, Origin(0, 0, -tibiaL)));
  };
  r.addMacro(d);

  auto leg = [&](const std::string& prefix, const Num& x, const Num& y, double side) {
    MacroArgs a;
    a.setStr("prefix", prefix).setStr("parent", "base_link");
    a.setNum("mount_x", x).setNum("mount_y", y).setNum("side", Num(side));
    r.instantiate("quadruped_leg", a);
  };
  leg("front_left", mountX, mountY, 1.0);
  leg("front_right", mountX, -mountY, -1.0);
  leg("rear_left", -mountX, mountY, 1.0);
  leg("rear_right", -mountX, -mountY, -1.0);
  return r;
}

// ====================================================================== biped

Robot buildBiped(const BipedConfig& c) {
  Robot r(c.name);
  r.addMaterial("torso_grey", 0.55, 0.57, 0.62);
  r.addMaterial("leg_blue", 0.25, 0.45, 0.85);
  r.addMaterial("foot_dark", 0.20, 0.20, 0.22);

  r.section("Torso");
  Num torsoD = r.prop("torso_depth", c.torsoDepth, "x size [m]");
  Num torsoW = r.prop("torso_width", c.torsoWidth, "y size [m]");
  Num torsoH = r.prop("torso_height", c.torsoHeight, "z size [m]");
  Num torsoM = r.prop("torso_mass", c.torsoMass, "[kg]");
  Num hipOff = r.prop("hip_offset", c.hipOffset, "distance of each hip from the centre along y [m]");

  r.section("Legs (3 DOF each: hip, knee, ankle - all pitch)");
  Num thighL = r.prop("thigh_length", c.thighLength, "[m]");
  Num shankL = r.prop("shank_length", c.shankLength, "[m]");
  Num footL = r.prop("foot_length", c.footLength, "[m]");
  Num footW = r.prop("foot_width", c.footWidth, "[m]");
  Num footH = r.prop("foot_height", c.footHeight, "[m]");
  Num legR = r.prop("leg_radius", c.legRadius, "[m]");
  Num thighM = r.prop("thigh_mass", c.thighMass, "[kg]");
  Num shankM = r.prop("shank_mass", c.shankMass, "[kg]");
  Num footM = r.prop("foot_mass", c.footMass, "[kg]");

  r.section("Joints");
  Num hipLim = r.prop("hip_limit", toRad(c.hipLimitDeg), "+/- [rad] (" + Num::format(c.hipLimitDeg) + " deg)");
  Num kneeLim = r.prop("knee_limit", toRad(c.kneeLimitDeg), "0 .. +limit [rad] (" + Num::format(c.kneeLimitDeg) + " deg), knee bends backwards");
  Num ankleLim = r.prop("ankle_limit", toRad(c.ankleLimitDeg), "+/- [rad] (" + Num::format(c.ankleLimitDeg) + " deg)");
  Num effort = r.prop("joint_effort", c.jointEffort, "[N m]");
  Num vel = r.prop("joint_velocity", c.jointVelocity, "[rad/s]");

  r.root.links.push_back(bodyLink("base_link", Origin(), Geometry::box(torsoD, torsoW, torsoH), "torso_grey", torsoM));

  MacroDef d;
  d.name = "biped_leg";
  d.strParams = {"prefix", "parent"};
  d.numParams = {"mount_y", "mount_z"};
  d.body = [=](Part& p, const MacroArgs& a) {
    const std::string pre = a.str("prefix");
    const std::string thigh = pre + "_thigh_link", shank = pre + "_shank_link", foot = pre + "_foot_link";
    p.links.push_back(bodyLink(thigh, cylinderAlongNegZ(thighL), Geometry::cylinder(legR, thighL), "leg_blue", thighM));
    p.links.push_back(bodyLink(shank, cylinderAlongNegZ(shankL), Geometry::cylinder(legR, shankL), "leg_blue", shankM));
    // The foot box sits under the ankle, shifted forward so the ankle is nearer the heel.
    p.links.push_back(bodyLink(foot, Origin(footL / 4, 0, -footH / 2), Geometry::box(footL, footW, footH), "foot_dark", footM));

    p.joints.push_back(limitedJoint(pre + "_hip_joint", JointType::Revolute, a.str("parent"), thigh,
                                    Origin(0, a.num("mount_y"), a.num("mount_z")), kAxisY,
                                    -hipLim, hipLim, effort, vel));
    p.joints.push_back(limitedJoint(pre + "_knee_joint", JointType::Revolute, thigh, shank,
                                    Origin(0, 0, -thighL), kAxisY, Num(0.0), kneeLim, effort, vel));
    p.joints.push_back(limitedJoint(pre + "_ankle_joint", JointType::Revolute, shank, foot,
                                    Origin(0, 0, -shankL), kAxisY, -ankleLim, ankleLim, effort, vel));
  };
  r.addMacro(d);

  const Num hipZ = -torsoH / 2;
  MacroArgs left, right;
  left.setStr("prefix", "left").setStr("parent", "base_link").setNum("mount_y", hipOff).setNum("mount_z", hipZ);
  right.setStr("prefix", "right").setStr("parent", "base_link").setNum("mount_y", -hipOff).setNum("mount_z", hipZ);
  r.instantiate("biped_leg", left);
  r.instantiate("biped_leg", right);
  return r;
}

// ======================================================================== arm

namespace {

struct Offset3 {
  Num x, y, z;
};

/// Origin of a cylinder that spans from the frame origin to the point v.
Origin cylinderAlongVector(const Offset3& v, Num* length) {
  *length = nsqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  const Num pitch = natan2(nsqrt(v.x * v.x + v.y * v.y), v.z);
  const Num yaw = natan2(v.y, v.x);
  return Origin(v.x / 2, v.y / 2, v.z / 2, 0, pitch, yaw);
}

const Axis3& axisOf(ArmAxis a) {
  switch (a) {
    case ArmAxis::X: return kAxisX;
    case ArmAxis::Y: return kAxisY;
    case ArmAxis::Z: return kAxisZ;
  }
  return kAxisZ;
}

const char* axisName(ArmAxis a) {
  switch (a) {
    case ArmAxis::X: return "X";
    case ArmAxis::Y: return "Y";
    case ArmAxis::Z: return "Z";
  }
  return "Z";
}

}  // namespace

Robot buildArm(const ArmConfig& c) {
  Robot r(c.name);
  r.addMaterial("base_dark", 0.25, 0.27, 0.30);
  r.addMaterial("arm_orange", 0.95, 0.50, 0.10);

  r.section("Base");
  Num baseR = r.prop("base_radius", c.baseRadius, "[m]");
  Num baseH = r.prop("base_height", c.baseHeight, "[m]");
  Num baseM = r.prop("base_mass", c.baseMass, "[kg]");

  r.section("General");
  Num linkR = r.prop("link_radius", c.linkRadius, "thickness of the link cylinders [m]");
  Num effort = r.prop("joint_effort", c.jointEffort, "[N for prismatic, N m for revolute]");
  Num vel = r.prop("joint_velocity", c.jointVelocity, "[m/s or rad/s]");

  const std::size_t n = c.links.size();
  std::vector<Offset3> offset;     // offset[i]: joint i+1 relative to the previous frame; offset[n]: tool
  std::vector<Num> lower, upper, mass;
  for (std::size_t i = 0; i < n; ++i) {
    const ArmLinkConfig& L = c.links[i];
    const std::string idx = std::to_string(i + 1);
    const std::string k = "joint" + idx;
    const bool rev = L.type == ArmJointType::Revolute;
    r.section("Joint " + idx + ": " + (rev ? "revolute" : "prismatic") + " about " + axisName(L.axis));
    if (L.mode == OffsetMode::Cartesian) {
      offset.push_back({r.prop(k + "_x", L.a, "offset x from previous frame [m]"),
                        r.prop(k + "_y", L.b, "offset y [m]"),
                        r.prop(k + "_z", L.c, "offset z [m]")});
    } else {
      Num rr = r.prop(k + "_r", L.a, "polar offset radius [m]");
      Num th = r.prop(k + "_theta", toRad(L.b), "polar offset angle about +z [rad] (" + Num::format(L.b) + " deg)");
      Num zz = r.prop(k + "_z", L.c, "offset z [m]");
      offset.push_back({rr * ncos(th), rr * nsin(th), zz});
    }
    if (rev) {
      lower.push_back(r.prop(k + "_lower", toRad(L.lower), "lower limit [rad] (" + Num::format(L.lower) + " deg)"));
      upper.push_back(r.prop(k + "_upper", toRad(L.upper), "upper limit [rad] (" + Num::format(L.upper) + " deg)"));
    } else {
      lower.push_back(r.prop(k + "_lower", L.lower, "lower limit [m]"));
      upper.push_back(r.prop(k + "_upper", L.upper, "upper limit [m]"));
    }
    mass.push_back(r.prop("link" + idx + "_mass", L.mass, "[kg]"));
  }
  r.section("Tool frame (end effector), relative to the last link");
  offset.push_back({r.prop("tool_x", c.toolX, "[m]"), r.prop("tool_y", c.toolY, "[m]"),
                    r.prop("tool_z", c.toolZ, "[m]")});

  r.root.links.push_back(bodyLink("base_link", Origin(0, 0, baseH / 2),
                                  Geometry::cylinder(baseR, baseH), "base_dark", baseM));

  for (std::size_t i = 0; i < n; ++i) {
    const ArmLinkConfig& L = c.links[i];
    const std::string parent = i == 0 ? "base_link" : "link_" + std::to_string(i);
    const std::string child = "link_" + std::to_string(i + 1);
    const std::string jointName = "joint_" + std::to_string(i + 1);

    r.root.joints.push_back(limitedJoint(
        jointName, L.type == ArmJointType::Revolute ? JointType::Revolute : JointType::Prismatic,
        parent, child, Origin(offset[i].x, offset[i].y, offset[i].z), axisOf(L.axis), lower[i],
        upper[i], effort, vel));

    // The link's body spans from its own joint to the next joint (or the tool frame).
    const Offset3& next = offset[i + 1];
    const double reach = std::sqrt(next.x.value() * next.x.value() + next.y.value() * next.y.value() +
                                   next.z.value() * next.z.value());
    if (reach < 1e-6) {
      r.root.links.push_back(bodyLink(child, Origin(), Geometry::sphere(linkR * 1.5), "arm_orange", mass[i]));
    } else {
      Num length;
      const Origin o = cylinderAlongVector(next, &length);
      r.root.links.push_back(bodyLink(child, o, Geometry::cylinder(linkR, length), "arm_orange", mass[i]));
    }
  }

  r.root.links.push_back(frameLink("tool0"));
  r.root.joints.push_back(fixedJoint("tool_joint", n == 0 ? "base_link" : "link_" + std::to_string(n),
                                     "tool0", Origin(offset[n].x, offset[n].y, offset[n].z)));
  return r;
}

// ================================================================== registry

const std::vector<RobotType>& allRobotTypes() {
  static const std::vector<RobotType> kAll = {
      RobotType::Hexapod,  RobotType::Arm,       RobotType::DiffDrive, RobotType::FourWheel,
      RobotType::Ackermann, RobotType::Quadruped, RobotType::Biped};
  return kAll;
}

std::string robotTypeId(RobotType t) {
  switch (t) {
    case RobotType::Hexapod: return "hexapod";
    case RobotType::Arm: return "arm";
    case RobotType::DiffDrive: return "diff_drive";
    case RobotType::FourWheel: return "four_wheel";
    case RobotType::Ackermann: return "ackermann";
    case RobotType::Quadruped: return "quadruped";
    case RobotType::Biped: return "biped";
  }
  return "";
}

std::string robotTypeLabel(RobotType t) {
  switch (t) {
    case RobotType::Hexapod: return "Hexapod";
    case RobotType::Arm: return "Arm linkage system";
    case RobotType::DiffDrive: return "Differential drive";
    case RobotType::FourWheel: return "4-wheeled";
    case RobotType::Ackermann: return "Ackermann drive";
    case RobotType::Quadruped: return "Quadruped";
    case RobotType::Biped: return "Biped";
  }
  return "";
}

Robot buildDefault(RobotType t) {
  switch (t) {
    case RobotType::Hexapod: return buildHexapod({});
    case RobotType::Arm: return buildArm({});
    case RobotType::DiffDrive: return buildDiffDrive({});
    case RobotType::FourWheel: return buildFourWheel({});
    case RobotType::Ackermann: return buildAckermann({});
    case RobotType::Quadruped: return buildQuadruped({});
    case RobotType::Biped: return buildBiped({});
  }
  return Robot("empty");
}

}  // namespace urdfgen
