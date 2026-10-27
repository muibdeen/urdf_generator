// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include "config_pages.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRegularExpression>
#include <QRegularExpressionValidator>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QStringList>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <cmath>
#include <vector>

#include "sensor_panel.hpp"
#include "urdf_generator/sensors.hpp"

namespace urdfgui {
namespace {

using namespace urdfgen;

const QString kDeg = QStringLiteral(" °");
const QString kMetre = QStringLiteral(" m");
constexpr double kPiD = 3.14159265358979323846;

// Spin boxes and combo boxes ignore the mouse wheel unless they have focus,
// so scrolling a long form does not change values by accident.
template <class T>
class NoWheel : public T {
 public:
  NoWheel() { this->setFocusPolicy(Qt::StrongFocus); }

 protected:
  void wheelEvent(QWheelEvent* e) override {
    if (this->hasFocus()) {
      T::wheelEvent(e);
    } else {
      e->ignore();
    }
  }
};
using Spin = NoWheel<QDoubleSpinBox>;
using IntSpin = NoWheel<QSpinBox>;
using Combo = NoWheel<QComboBox>;

QDoubleSpinBox* makeSpin(const QString& key, double value, double lo, double hi, int decimals,
                         double step, const QString& suffix) {
  auto* s = new Spin;
  s->setObjectName(key);
  s->setDecimals(decimals);
  s->setRange(lo, hi);
  s->setSingleStep(step);
  s->setSuffix(suffix);
  s->setValue(value);
  s->setAlignment(Qt::AlignRight);
  return s;
}

void setRowVisible(QFormLayout* form, QWidget* field, bool visible) {
  field->setVisible(visible);
  if (QWidget* label = form->labelForField(field)) label->setVisible(visible);
}

struct ChassisFields {
  QComboBox* shape = nullptr;
  QDoubleSpinBox* length = nullptr;
  QDoubleSpinBox* width = nullptr;
  QDoubleSpinBox* height = nullptr;
  QDoubleSpinBox* radius = nullptr;
  QDoubleSpinBox* mass = nullptr;

  ChassisConfig read() const {
    ChassisConfig c;
    c.shape = shape->currentIndex() == 0 ? ChassisShape::Box : ChassisShape::Cylinder;
    c.length = length->value();
    c.width = width->value();
    c.height = height->value();
    c.radius = radius->value();
    c.mass = mass->value();
    return c;
  }
};

/// The "Standard / Mecanum" choice of every wheel of a wheeled robot, and the roller count.
struct WheelChoices {
  std::vector<QComboBox*> kinds;
  QSpinBox* rollers = nullptr;

  WheelKind kind(std::size_t i) const {
    return kinds.at(i)->currentIndex() == 1 ? WheelKind::Mecanum : WheelKind::Standard;
  }
};

/// Shared plumbing: the robot-name field, grouped form sections and typed spin boxes.
class PageBase : public ConfigPage {
 public:
  explicit PageBase(const std::string& defaultName) {
    layout_ = new QVBoxLayout(this);
    layout_->setContentsMargins(0, 0, 0, 0);
    QFormLayout* f = addGroup("Robot");
    name_ = new QLineEdit(QString::fromStdString(defaultName));
    name_->setObjectName("robot_name");
    name_->setValidator(
        new QRegularExpressionValidator(QRegularExpression("[A-Za-z0-9_.\\-]*"), name_));
    name_->setToolTip(
        "Used as the robot name and as the default file name.\n"
        "Letters, digits, '_', '-' and '.' only.");
    f->addRow("Name", name_);
    connect(name_, &QLineEdit::textChanged, this, &ConfigPage::changed);

    sensors_ = new SensorPanel;
    connect(sensors_, &SensorPanel::changed, this, &ConfigPage::changed);
    connect(sensors_, &SensorPanel::addRequested, this, [this](SensorType type) {
      const Robot base = buildRobot();  // a new sensor starts on top of the main body
      sensors_->setRobot(base);
      sensors_->append(defaultSensor(base, type, sensors_->sensors()));
    });
  }

  /// The robot of the page's own values, with the sensors of the sensor panel mounted on it.
  /// Also keeps the panel's link lists and slider ranges in step with the robot.
  Robot build() const final {
    Robot r = buildRobot();
    sensors_->setRobot(r);
    addSensors(r, sensors_->sensors());
    return r;
  }

 protected:
  /// The robot without sensors.
  virtual Robot buildRobot() const = 0;

  std::string robotName() const { return name_->text().trimmed().toStdString(); }

  /// Call at the end of a page's constructor.
  void finish() {
    layout_->addWidget(sensors_);
    layout_->addStretch(1);
  }

  QFormLayout* addGroup(const QString& title) {
    auto* box = new QGroupBox(title);
    auto* form = new QFormLayout(box);
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    layout_->addWidget(box);
    return form;
  }

  QDoubleSpinBox* add(QFormLayout* f, const QString& label, QDoubleSpinBox* s, const QString& tip) {
    if (!tip.isEmpty()) s->setToolTip(tip);
    f->addRow(label, s);
    connect(s, &QDoubleSpinBox::valueChanged, this, &ConfigPage::changed);
    return s;
  }

  /// A size in metres; must be greater than zero.
  QDoubleSpinBox* length(QFormLayout* f, const QString& key, const QString& label, double v,
                         const QString& tip = QString()) {
    return add(f, label, makeSpin(key, v, 0.001, 10.0, 4, 0.01, kMetre), tip);
  }
  /// A position in metres; may be negative.
  QDoubleSpinBox* position(QFormLayout* f, const QString& key, const QString& label, double v,
                           const QString& tip = QString()) {
    return add(f, label, makeSpin(key, v, -10.0, 10.0, 4, 0.01, kMetre), tip);
  }
  QDoubleSpinBox* mass(QFormLayout* f, const QString& key, const QString& label, double v,
                       const QString& tip = QString()) {
    return add(f, label, makeSpin(key, v, 0.001, 1000.0, 3, 0.1, QStringLiteral(" kg")), tip);
  }
  QDoubleSpinBox* angle(QFormLayout* f, const QString& key, const QString& label, double v,
                        double lo, double hi, const QString& tip = QString()) {
    return add(f, label, makeSpin(key, v, lo, hi, 1, 5.0, kDeg), tip);
  }
  QDoubleSpinBox* effort(QFormLayout* f, double v) {
    return add(f, "Max effort", makeSpin("joint_effort", v, 0.001, 10000.0, 2, 1.0, QStringLiteral(" N·m")),
               "Force or torque limit written to every limited joint.");
  }
  QDoubleSpinBox* velocity(QFormLayout* f, double v) {
    return add(f, "Max velocity", makeSpin("joint_velocity", v, 0.001, 1000.0, 2, 0.5, QStringLiteral(" rad/s")),
               "Speed limit written to every limited joint.");
  }

  QComboBox* combo(QFormLayout* f, const QString& key, const QString& label,
                   const QStringList& items, int index) {
    auto* c = new Combo;
    c->setObjectName(key);
    c->addItems(items);
    c->setCurrentIndex(index);
    f->addRow(label, c);
    connect(c, &QComboBox::currentIndexChanged, this, &ConfigPage::changed);
    return c;
  }

  ChassisFields addChassis(const ChassisConfig& d) {
    ChassisFields c;
    QFormLayout* f = addGroup("Chassis");
    c.shape = combo(f, "chassis_shape", "Shape",
                    {"Square / rectangular box", "Circular cylinder"},
                    d.shape == ChassisShape::Box ? 0 : 1);
    c.length = length(f, "chassis_length", "Length (x)", d.length);
    c.width = length(f, "chassis_width", "Width (y)", d.width);
    c.radius = length(f, "chassis_radius", "Radius", d.radius);
    c.height = length(f, "chassis_height", "Height (z)", d.height);
    c.mass = mass(f, "chassis_mass", "Mass", d.mass);
    auto sync = [c, f]() {
      const bool box = c.shape->currentIndex() == 0;
      setRowVisible(f, c.length, box);
      setRowVisible(f, c.width, box);
      setRowVisible(f, c.radius, !box);
    };
    connect(c.shape, &QComboBox::currentIndexChanged, this, sync);
    sync();
    return c;
  }

  /// One Standard / Mecanum combo per wheel (key = object name, label = row text, in the order
  /// given), the number of rollers per mecanum wheel, and two buttons that set every wheel at once.
  WheelChoices addWheelChoices(const std::vector<std::pair<QString, QString>>& wheels, int rollers) {
    WheelChoices w;
    QFormLayout* f = addGroup("Wheel type");
    for (const auto& kv : wheels) {
      QComboBox* c = combo(f, kv.first, kv.second, {"Standard", "Mecanum"}, 0);
      c->setToolTip(
          "Standard: a plain cylinder wheel.\n"
          "Mecanum: a hub with rollers set at 45 degrees around its rim. The slant of the rollers "
          "follows the wheel's position (front-left and rear-right one way, front-right and "
          "rear-left the other), so four mecanum wheels form the usual layout.");
      w.kinds.push_back(c);
    }
    w.rollers = new IntSpin;
    w.rollers->setObjectName("mecanum_rollers");
    w.rollers->setRange(3, 16);
    w.rollers->setValue(rollers);
    w.rollers->setAlignment(Qt::AlignRight);
    w.rollers->setToolTip(
        "Rollers around the rim of every mecanum wheel. About 2\u03C0 \u00D7 (radius \u2212 width/4) / width "
        "rollers fill the rim (8 for the default wheels).");
    f->addRow("Rollers per mecanum wheel", w.rollers);
    connect(w.rollers, &QSpinBox::valueChanged, this, &ConfigPage::changed);

    auto* row = new QHBoxLayout;
    auto* allStandard = new QPushButton("All standard");
    auto* allMecanum = new QPushButton("All mecanum");
    allStandard->setObjectName("all_standard");
    allMecanum->setObjectName("all_mecanum");
    allStandard->setAutoDefault(false);
    allMecanum->setAutoDefault(false);
    row->addStretch(1);
    row->addWidget(allStandard);
    row->addWidget(allMecanum);
    f->addRow(row);

    const std::vector<QComboBox*> kinds = w.kinds;
    QSpinBox* rollersBox = w.rollers;
    auto sync = [kinds, rollersBox]() {
      bool any = false;
      for (QComboBox* c : kinds) any = any || c->currentIndex() == 1;
      rollersBox->setEnabled(any);
    };
    for (QComboBox* c : kinds) connect(c, &QComboBox::currentIndexChanged, this, sync);
    connect(allStandard, &QPushButton::clicked, this, [kinds]() {
      for (QComboBox* c : kinds) c->setCurrentIndex(0);
    });
    connect(allMecanum, &QPushButton::clicked, this, [kinds]() {
      for (QComboBox* c : kinds) c->setCurrentIndex(1);
    });
    sync();
    return w;
  }

  QVBoxLayout* layout_ = nullptr;

 private:
  QLineEdit* name_ = nullptr;
};

// ================================================================= hexapod

class HexapodPage : public PageBase {
 public:
  HexapodPage() : PageBase(HexapodConfig{}.name) {
    const HexapodConfig d{};
    QFormLayout* body = addGroup("Body");
    bodyRadius_ = length(body, "body_radius", "Leg mounts: distance from centre", d.bodyRadius,
                         "Radius of the circle the six leg hips sit on.");
    bodyHeight_ = length(body, "body_height", "Body height", d.bodyHeight);

    QFormLayout* legs = addGroup("Leg link lengths");
    coxa_ = length(legs, "coxa_length", "Coxa (hip)", d.coxaLength,
                   "Short link from the body to the femur; rotates about the vertical axis.");
    femur_ = length(legs, "femur_length", "Femur (thigh)", d.femurLength);
    tibia_ = length(legs, "tibia_length", "Tibia (shin)", d.tibiaLength);
    legRadius_ = length(legs, "leg_radius", "Link thickness (radius)", d.legRadius);

    QFormLayout* masses = addGroup("Masses");
    bodyMass_ = mass(masses, "body_mass", "Body", d.bodyMass);
    coxaMass_ = mass(masses, "coxa_mass", "Coxa", d.coxaMass);
    femurMass_ = mass(masses, "femur_mass", "Femur", d.femurMass);
    tibiaMass_ = mass(masses, "tibia_mass", "Tibia", d.tibiaMass);
    footMass_ = mass(masses, "foot_mass", "Foot", d.footMass);

    QFormLayout* joints = addGroup("Joints");
    coxaLim_ = angle(joints, "coxa_limit", "Coxa limit ±", d.coxaLimitDeg, 1, 180);
    femurLim_ = angle(joints, "femur_limit", "Femur limit ±", d.femurLimitDeg, 1, 180);
    tibiaLim_ = angle(joints, "tibia_limit", "Tibia limit ±", d.tibiaLimitDeg, 1, 180);
    effort_ = effort(joints, d.jointEffort);
    velocity_ = velocity(joints, d.jointVelocity);
    finish();
  }

  RobotType type() const override { return RobotType::Hexapod; }
  QString hint() const override {
    return "Six legs with three joints each (coxa yaw, femur pitch, tibia pitch), "
           "spaced 60° around a round body.";
  }
  Robot buildRobot() const override {
    HexapodConfig c;
    c.name = robotName();
    c.bodyRadius = bodyRadius_->value();
    c.bodyHeight = bodyHeight_->value();
    c.coxaLength = coxa_->value();
    c.femurLength = femur_->value();
    c.tibiaLength = tibia_->value();
    c.legRadius = legRadius_->value();
    c.bodyMass = bodyMass_->value();
    c.coxaMass = coxaMass_->value();
    c.femurMass = femurMass_->value();
    c.tibiaMass = tibiaMass_->value();
    c.footMass = footMass_->value();
    c.coxaLimitDeg = coxaLim_->value();
    c.femurLimitDeg = femurLim_->value();
    c.tibiaLimitDeg = tibiaLim_->value();
    c.jointEffort = effort_->value();
    c.jointVelocity = velocity_->value();
    return buildHexapod(c);
  }

 private:
  QDoubleSpinBox *bodyRadius_, *bodyHeight_, *coxa_, *femur_, *tibia_, *legRadius_;
  QDoubleSpinBox *bodyMass_, *coxaMass_, *femurMass_, *tibiaMass_, *footMass_;
  QDoubleSpinBox *coxaLim_, *femurLim_, *tibiaLim_, *effort_, *velocity_;
};

// ================================================================ quadruped

class QuadrupedPage : public PageBase {
 public:
  QuadrupedPage() : PageBase(QuadrupedConfig{}.name) {
    const QuadrupedConfig d{};
    QFormLayout* body = addGroup("Body");
    bodyLength_ = length(body, "body_length", "Length (x)", d.bodyLength);
    bodyWidth_ = length(body, "body_width", "Width (y)", d.bodyWidth);
    bodyHeight_ = length(body, "body_height", "Height (z)", d.bodyHeight);

    QFormLayout* mount = addGroup("Leg mounts: distance from centre");
    mountX_ = length(mount, "leg_mount_x", "Hips along x (front/rear)", d.legMountX,
                     "Front hips at +x, rear hips at -x.");
    mountY_ = length(mount, "leg_mount_y", "Hips along y (left/right)", d.legMountY,
                     "Left hips at +y, right hips at -y.");

    QFormLayout* legs = addGroup("Leg link lengths");
    coxa_ = length(legs, "coxa_length", "Coxa (sideways hip link)", d.coxaLength);
    femur_ = length(legs, "femur_length", "Femur (thigh)", d.femurLength);
    tibia_ = length(legs, "tibia_length", "Tibia (shin)", d.tibiaLength);
    legRadius_ = length(legs, "leg_radius", "Link thickness (radius)", d.legRadius);

    QFormLayout* masses = addGroup("Masses");
    bodyMass_ = mass(masses, "body_mass", "Body", d.bodyMass);
    coxaMass_ = mass(masses, "coxa_mass", "Coxa", d.coxaMass);
    femurMass_ = mass(masses, "femur_mass", "Femur", d.femurMass);
    tibiaMass_ = mass(masses, "tibia_mass", "Tibia", d.tibiaMass);
    footMass_ = mass(masses, "foot_mass", "Foot", d.footMass);

    QFormLayout* joints = addGroup("Joints");
    coxaLim_ = angle(joints, "coxa_limit", "Coxa limit ±", d.coxaLimitDeg, 1, 180);
    femurLim_ = angle(joints, "femur_limit", "Femur limit ±", d.femurLimitDeg, 1, 180);
    tibiaLim_ = angle(joints, "tibia_limit", "Tibia limit ±", d.tibiaLimitDeg, 1, 180);
    effort_ = effort(joints, d.jointEffort);
    velocity_ = velocity(joints, d.jointVelocity);
    finish();
  }

  RobotType type() const override { return RobotType::Quadruped; }
  QString hint() const override {
    return "Four legs with three joints each (coxa roll, femur pitch, tibia pitch) "
           "on a box body.";
  }
  Robot buildRobot() const override {
    QuadrupedConfig c;
    c.name = robotName();
    c.bodyLength = bodyLength_->value();
    c.bodyWidth = bodyWidth_->value();
    c.bodyHeight = bodyHeight_->value();
    c.legMountX = mountX_->value();
    c.legMountY = mountY_->value();
    c.coxaLength = coxa_->value();
    c.femurLength = femur_->value();
    c.tibiaLength = tibia_->value();
    c.legRadius = legRadius_->value();
    c.bodyMass = bodyMass_->value();
    c.coxaMass = coxaMass_->value();
    c.femurMass = femurMass_->value();
    c.tibiaMass = tibiaMass_->value();
    c.footMass = footMass_->value();
    c.coxaLimitDeg = coxaLim_->value();
    c.femurLimitDeg = femurLim_->value();
    c.tibiaLimitDeg = tibiaLim_->value();
    c.jointEffort = effort_->value();
    c.jointVelocity = velocity_->value();
    return buildQuadruped(c);
  }

 private:
  QDoubleSpinBox *bodyLength_, *bodyWidth_, *bodyHeight_, *mountX_, *mountY_;
  QDoubleSpinBox *coxa_, *femur_, *tibia_, *legRadius_;
  QDoubleSpinBox *bodyMass_, *coxaMass_, *femurMass_, *tibiaMass_, *footMass_;
  QDoubleSpinBox *coxaLim_, *femurLim_, *tibiaLim_, *effort_, *velocity_;
};

// ===================================================================== biped

class BipedPage : public PageBase {
 public:
  BipedPage() : PageBase(BipedConfig{}.name) {
    const BipedConfig d{};
    QFormLayout* torso = addGroup("Torso");
    torsoDepth_ = length(torso, "torso_depth", "Depth (x)", d.torsoDepth);
    torsoWidth_ = length(torso, "torso_width", "Width (y)", d.torsoWidth);
    torsoHeight_ = length(torso, "torso_height", "Height (z)", d.torsoHeight);
    torsoMass_ = mass(torso, "torso_mass", "Mass", d.torsoMass);

    QFormLayout* hips = addGroup("Leg mounts: distance from centre");
    hipOffset_ = length(hips, "hip_offset", "Hip offset from centre (y)", d.hipOffset,
                        "Each hip sits this far to the left (+y) or right (-y) of the centre line, "
                        "at the bottom of the torso.");

    QFormLayout* legs = addGroup("Leg link lengths");
    thigh_ = length(legs, "thigh_length", "Thigh", d.thighLength);
    shank_ = length(legs, "shank_length", "Shank", d.shankLength);
    legRadius_ = length(legs, "leg_radius", "Link thickness (radius)", d.legRadius);

    QFormLayout* foot = addGroup("Foot");
    footLength_ = length(foot, "foot_length", "Length (x)", d.footLength);
    footWidth_ = length(foot, "foot_width", "Width (y)", d.footWidth);
    footHeight_ = length(foot, "foot_height", "Height (z)", d.footHeight);

    QFormLayout* masses = addGroup("Masses");
    thighMass_ = mass(masses, "thigh_mass", "Thigh", d.thighMass);
    shankMass_ = mass(masses, "shank_mass", "Shank", d.shankMass);
    footMass_ = mass(masses, "foot_mass", "Foot", d.footMass);

    QFormLayout* joints = addGroup("Joints (all pitch)");
    hipLim_ = angle(joints, "hip_limit", "Hip limit ±", d.hipLimitDeg, 1, 180);
    kneeLim_ = angle(joints, "knee_limit", "Knee bend (0 to)", d.kneeLimitDeg, 1, 180,
                     "The knee bends backwards from 0 up to this angle.");
    ankleLim_ = angle(joints, "ankle_limit", "Ankle limit ±", d.ankleLimitDeg, 1, 180);
    effort_ = effort(joints, d.jointEffort);
    velocity_ = velocity(joints, d.jointVelocity);
    finish();
  }

  RobotType type() const override { return RobotType::Biped; }
  QString hint() const override {
    return "Two legs with three joints each (hip, knee, ankle, all pitch) and box feet, "
           "hanging from a box torso.";
  }
  Robot buildRobot() const override {
    BipedConfig c;
    c.name = robotName();
    c.torsoDepth = torsoDepth_->value();
    c.torsoWidth = torsoWidth_->value();
    c.torsoHeight = torsoHeight_->value();
    c.torsoMass = torsoMass_->value();
    c.hipOffset = hipOffset_->value();
    c.thighLength = thigh_->value();
    c.shankLength = shank_->value();
    c.legRadius = legRadius_->value();
    c.footLength = footLength_->value();
    c.footWidth = footWidth_->value();
    c.footHeight = footHeight_->value();
    c.thighMass = thighMass_->value();
    c.shankMass = shankMass_->value();
    c.footMass = footMass_->value();
    c.hipLimitDeg = hipLim_->value();
    c.kneeLimitDeg = kneeLim_->value();
    c.ankleLimitDeg = ankleLim_->value();
    c.jointEffort = effort_->value();
    c.jointVelocity = velocity_->value();
    return buildBiped(c);
  }

 private:
  QDoubleSpinBox *torsoDepth_, *torsoWidth_, *torsoHeight_, *torsoMass_, *hipOffset_;
  QDoubleSpinBox *thigh_, *shank_, *legRadius_, *footLength_, *footWidth_, *footHeight_;
  QDoubleSpinBox *thighMass_, *shankMass_, *footMass_;
  QDoubleSpinBox *hipLim_, *kneeLim_, *ankleLim_, *effort_, *velocity_;
};

// ============================================================ wheeled robots

class DiffDrivePage : public PageBase {
 public:
  DiffDrivePage() : PageBase(DiffDriveConfig{}.name) {
    const DiffDriveConfig d{};
    chassis_ = addChassis(d.chassis);

    QFormLayout* w = addGroup("Drive wheels");
    radius_ = length(w, "wheel_radius", "Wheel radius", d.wheelRadius);
    width_ = length(w, "wheel_width", "Wheel width", d.wheelWidth);
    mass_ = mass(w, "wheel_mass", "Wheel mass", d.wheelMass);
    x_ = position(w, "wheel_x", "Position x from centre", d.wheelX,
                  "Along the forward axis. 0 puts the axle under the chassis centre.");
    y_ = position(w, "wheel_y", "Position y from centre", d.wheelY,
                  "Left wheel at +y; the right wheel is placed at -y.");
    z_ = position(w, "wheel_z", "Position z from centre", d.wheelZ,
                  "Height of the wheel centre relative to the chassis centre. "
                  "The robot is placed so the wheels touch the ground plane.");

    wheels_ = addWheelChoices({{"left_wheel_type", "Left wheel"}, {"right_wheel_type", "Right wheel"}},
                              d.mecanumRollers);

    QFormLayout* c = addGroup("Casters");
    casters_ = new QCheckBox("Add front and rear ball casters");
    casters_->setObjectName("casters");
    casters_->setChecked(d.casters);
    c->addRow(casters_);
    casterRadius_ = length(c, "caster_radius", "Caster radius", d.casterRadius);
    casterMass_ = mass(c, "caster_mass", "Caster mass", d.casterMass);
    connect(casters_, &QCheckBox::toggled, this, [this](bool on) {
      casterRadius_->setEnabled(on);
      casterMass_->setEnabled(on);
      emit changed();
    });
    casterRadius_->setEnabled(d.casters);
    casterMass_->setEnabled(d.casters);
    finish();
  }

  RobotType type() const override { return RobotType::DiffDrive; }
  QString hint() const override {
    return "A chassis with a left and a right drive wheel, plus optional ball casters at the "
           "front and rear. Each wheel can be a plain wheel or a mecanum wheel.";
  }
  Robot buildRobot() const override {
    DiffDriveConfig c;
    c.name = robotName();
    c.chassis = chassis_.read();
    c.wheelRadius = radius_->value();
    c.wheelWidth = width_->value();
    c.wheelMass = mass_->value();
    c.wheelX = x_->value();
    c.wheelY = y_->value();
    c.wheelZ = z_->value();
    c.leftWheel = wheels_.kind(0);
    c.rightWheel = wheels_.kind(1);
    c.mecanumRollers = wheels_.rollers->value();
    c.casters = casters_->isChecked();
    c.casterRadius = casterRadius_->value();
    c.casterMass = casterMass_->value();
    return buildDiffDrive(c);
  }

 private:
  ChassisFields chassis_;
  WheelChoices wheels_;
  QDoubleSpinBox *radius_, *width_, *mass_, *x_, *y_, *z_, *casterRadius_, *casterMass_;
  QCheckBox* casters_;
};

class FourWheelPage : public PageBase {
 public:
  FourWheelPage() : PageBase(FourWheelConfig{}.name) {
    const FourWheelConfig d{};
    chassis_ = addChassis(d.chassis);

    QFormLayout* w = addGroup("Wheels (all four driven)");
    radius_ = length(w, "wheel_radius", "Wheel radius", d.wheelRadius);
    width_ = length(w, "wheel_width", "Wheel width", d.wheelWidth);
    mass_ = mass(w, "wheel_mass", "Wheel mass", d.wheelMass);
    x_ = position(w, "wheel_x", "Position x from centre", d.wheelX,
                  "Front wheels at +x, rear wheels at -x.");
    y_ = position(w, "wheel_y", "Position y from centre", d.wheelY,
                  "Left wheels at +y, right wheels at -y.");
    z_ = position(w, "wheel_z", "Position z from centre", d.wheelZ,
                  "Height of the wheel centres relative to the chassis centre.");
    wheels_ = addWheelChoices({{"front_left_wheel_type", "Front left"},
                               {"front_right_wheel_type", "Front right"},
                               {"rear_left_wheel_type", "Rear left"},
                               {"rear_right_wheel_type", "Rear right"}},
                              d.mecanumRollers);
    finish();
  }

  RobotType type() const override { return RobotType::FourWheel; }
  QString hint() const override {
    return "A chassis with four driven wheels (skid-steer / 4WD). Enter the front-left wheel; "
           "the other three are mirrored. Each wheel can be a plain wheel or a mecanum wheel.";
  }
  Robot buildRobot() const override {
    FourWheelConfig c;
    c.name = robotName();
    c.chassis = chassis_.read();
    c.wheelRadius = radius_->value();
    c.wheelWidth = width_->value();
    c.wheelMass = mass_->value();
    c.wheelX = x_->value();
    c.wheelY = y_->value();
    c.wheelZ = z_->value();
    c.frontLeftWheel = wheels_.kind(0);
    c.frontRightWheel = wheels_.kind(1);
    c.rearLeftWheel = wheels_.kind(2);
    c.rearRightWheel = wheels_.kind(3);
    c.mecanumRollers = wheels_.rollers->value();
    return buildFourWheel(c);
  }

 private:
  ChassisFields chassis_;
  WheelChoices wheels_;
  QDoubleSpinBox *radius_, *width_, *mass_, *x_, *y_, *z_;
};

class AckermannPage : public PageBase {
 public:
  AckermannPage() : PageBase(AckermannConfig{}.name) {
    const AckermannConfig d{};
    chassis_ = addChassis(d.chassis);

    QFormLayout* w = addGroup("Wheels");
    radius_ = length(w, "wheel_radius", "Wheel radius", d.wheelRadius);
    width_ = length(w, "wheel_width", "Wheel width", d.wheelWidth);
    mass_ = mass(w, "wheel_mass", "Wheel mass", d.wheelMass);
    z_ = position(w, "wheel_z", "Position z from centre", d.wheelZ,
                  "Height of the wheel centres relative to the chassis centre.");
    wheels_ = addWheelChoices({{"front_left_wheel_type", "Front left (steered)"},
                               {"front_right_wheel_type", "Front right (steered)"},
                               {"rear_left_wheel_type", "Rear left (driven)"},
                               {"rear_right_wheel_type", "Rear right (driven)"}},
                              d.mecanumRollers);

    QFormLayout* g = addGroup("Ackermann geometry");
    wheelbase_ = length(g, "wheelbase", "Wheelbase", d.wheelbase,
                        "Distance between the front and rear axles.");
    track_ = length(g, "track_width", "Track width", d.trackWidth,
                    "Distance between the left and right wheels.");
    steerLimit_ = angle(g, "steer_limit", "Steering limit ±", d.steerLimitDeg, 1, 85);
    steerEffort_ = add(g, "Steering max effort",
                       makeSpin("steer_effort", d.steerEffort, 0.001, 10000.0, 2, 1.0, QStringLiteral(" N·m")),
                       QString());
    steerVelocity_ = add(g, "Steering max velocity",
                         makeSpin("steer_velocity", d.steerVelocity, 0.001, 1000.0, 2, 0.5, QStringLiteral(" rad/s")),
                         QString());
    finish();
  }

  RobotType type() const override { return RobotType::Ackermann; }
  QString hint() const override {
    return "Rear wheels driven, front wheels steered about vertical joints (Ackermann layout). "
           "Wheels are placed from the wheelbase and track width. Each wheel can be a plain wheel "
           "or a mecanum wheel.";
  }
  Robot buildRobot() const override {
    AckermannConfig c;
    c.name = robotName();
    c.chassis = chassis_.read();
    c.wheelRadius = radius_->value();
    c.wheelWidth = width_->value();
    c.wheelMass = mass_->value();
    c.wheelZ = z_->value();
    c.frontLeftWheel = wheels_.kind(0);
    c.frontRightWheel = wheels_.kind(1);
    c.rearLeftWheel = wheels_.kind(2);
    c.rearRightWheel = wheels_.kind(3);
    c.mecanumRollers = wheels_.rollers->value();
    c.wheelbase = wheelbase_->value();
    c.trackWidth = track_->value();
    c.steerLimitDeg = steerLimit_->value();
    c.steerEffort = steerEffort_->value();
    c.steerVelocity = steerVelocity_->value();
    return buildAckermann(c);
  }

 private:
  ChassisFields chassis_;
  WheelChoices wheels_;
  QDoubleSpinBox *radius_, *width_, *mass_, *z_, *wheelbase_, *track_;
  QDoubleSpinBox *steerLimit_, *steerEffort_, *steerVelocity_;
};

// ====================================================================== arm

struct ArmRow {
  QGroupBox* box = nullptr;
  QComboBox *type = nullptr, *axis = nullptr, *mode = nullptr;
  QDoubleSpinBox *a = nullptr, *b = nullptr, *c = nullptr;
  QDoubleSpinBox *lower = nullptr, *upper = nullptr, *mass = nullptr;
};

class ArmPage : public PageBase {
 public:
  ArmPage() : PageBase(ArmConfig{}.name) {
    const ArmConfig d{};
    QFormLayout* base = addGroup("Base");
    baseRadius_ = length(base, "base_radius", "Base radius", d.baseRadius);
    baseHeight_ = length(base, "base_height", "Base height", d.baseHeight);
    baseMass_ = mass(base, "base_mass", "Base mass", d.baseMass);

    QFormLayout* gen = addGroup("General");
    linkRadius_ = length(gen, "link_radius", "Link thickness (radius)", d.linkRadius);
    effort_ = add(gen, "Max effort",
                  makeSpin("joint_effort", d.jointEffort, 0.001, 10000.0, 2, 1.0, QStringLiteral(" N or N·m")),
                  "Force (sliding joints) or torque (revolute joints) limit.");
    velocity_ = add(gen, "Max velocity",
                    makeSpin("joint_velocity", d.jointVelocity, 0.001, 1000.0, 2, 0.5, QStringLiteral(" m/s or rad/s")),
                    QString());

    QFormLayout* tool = addGroup("Tool frame (end effector)");
    toolX_ = position(tool, "tool_x", "Offset x", d.toolX,
                      "Position of the tool0 frame relative to the last link's frame. "
                      "The last link is drawn as a rod up to this point.");
    toolY_ = position(tool, "tool_y", "Offset y", d.toolY);
    toolZ_ = position(tool, "tool_z", "Offset z", d.toolZ);

    auto* box = new QGroupBox("Links");
    auto* v = new QVBoxLayout(box);
    auto* top = new QHBoxLayout;
    top->addWidget(new QLabel("Number of links"));
    count_ = new IntSpin;
    count_->setObjectName("link_count");
    count_->setRange(1, 16);
    top->addWidget(count_);
    top->addStretch(1);
    v->addLayout(top);
    auto* help = new QLabel(
        "Each link adds one joint. Its offset places that joint relative to the previous "
        "link's frame, either Cartesian (x, y, z) or Polar (r, θ, z) with θ measured "
        "from +x about +z.");
    help->setWordWrap(true);
    v->addWidget(help);
    rowsLayout_ = new QVBoxLayout;
    v->addLayout(rowsLayout_);
    layout_->addWidget(box);

    for (const ArmLinkConfig& l : d.links) addRow(l);
    count_->setValue(static_cast<int>(rows_.size()));
    connect(count_, &QSpinBox::valueChanged, this, &ArmPage::resizeRows);
    finish();
  }

  RobotType type() const override { return RobotType::Arm; }
  QString hint() const override {
    return "A serial chain of links. Pick the number of links, then for each joint choose "
           "revolute or sliding, its axis, and a Cartesian or Polar offset.";
  }
  Robot buildRobot() const override {
    ArmConfig c;
    c.name = robotName();
    c.baseRadius = baseRadius_->value();
    c.baseHeight = baseHeight_->value();
    c.baseMass = baseMass_->value();
    c.linkRadius = linkRadius_->value();
    c.jointEffort = effort_->value();
    c.jointVelocity = velocity_->value();
    c.toolX = toolX_->value();
    c.toolY = toolY_->value();
    c.toolZ = toolZ_->value();
    c.links.clear();
    for (const ArmRow& r : rows_) {
      ArmLinkConfig l;
      l.type = r.type->currentIndex() == 0 ? ArmJointType::Revolute : ArmJointType::Prismatic;
      l.axis = static_cast<ArmAxis>(r.axis->currentIndex());
      l.mode = r.mode->currentIndex() == 0 ? OffsetMode::Cartesian : OffsetMode::Polar;
      l.a = r.a->value();
      l.b = r.b->value();
      l.c = r.c->value();
      l.lower = r.lower->value();
      l.upper = r.upper->value();
      l.mass = r.mass->value();
      c.links.push_back(l);
    }
    return buildArm(c);
  }

 private:
  static void applyMode(const ArmRow& r, bool polar, double a, double b, double c) {
    const QSignalBlocker ba(r.a), bb(r.b), bc(r.c);
    if (polar) {
      r.a->setPrefix("r ");
      r.a->setRange(0.0, 5.0);
      r.b->setPrefix(QStringLiteral("θ "));
      r.b->setSuffix(kDeg);
      r.b->setDecimals(2);
      r.b->setRange(-360.0, 360.0);
      r.b->setSingleStep(5.0);
      r.a->setToolTip("Distance from the previous frame's z axis.");
      r.b->setToolTip("Angle from the previous frame's +x axis, about +z.");
    } else {
      r.a->setPrefix("x ");
      r.a->setRange(-5.0, 5.0);
      r.b->setPrefix("y ");
      r.b->setSuffix(kMetre);
      r.b->setDecimals(4);
      r.b->setRange(-5.0, 5.0);
      r.b->setSingleStep(0.01);
      r.a->setToolTip("Offset along the previous frame's x axis.");
      r.b->setToolTip("Offset along the previous frame's y axis.");
    }
    r.a->setValue(a);
    r.b->setValue(b);
    r.c->setValue(c);
  }

  static void applyLimits(const ArmRow& r, bool revolute, double lo, double hi) {
    const QSignalBlocker bl(r.lower), bu(r.upper);
    for (QDoubleSpinBox* s : {r.lower, r.upper}) {
      if (revolute) {
        s->setSuffix(kDeg);
        s->setDecimals(1);
        s->setRange(-360.0, 360.0);
        s->setSingleStep(5.0);
      } else {
        s->setSuffix(kMetre);
        s->setDecimals(4);
        s->setRange(-5.0, 5.0);
        s->setSingleStep(0.01);
      }
    }
    r.lower->setValue(lo);
    r.upper->setValue(hi);
  }

  void addRow(const ArmLinkConfig& l) {
    const int index = static_cast<int>(rows_.size()) + 1;
    const QString k = QStringLiteral("link%1_").arg(index);
    ArmRow r;
    r.box = new QGroupBox(QStringLiteral("Link %1").arg(index));
    auto* form = new QFormLayout(r.box);
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);

    auto makeCombo = [&](const QString& key, const QStringList& items, int idx) {
      auto* c = new Combo;
      c->setObjectName(k + key);
      c->addItems(items);
      c->setCurrentIndex(idx);
      return c;
    };
    r.type = makeCombo("type", {"Revolute (rotating)", "Prismatic (sliding)"},
                       l.type == ArmJointType::Revolute ? 0 : 1);
    r.axis = makeCombo("axis", {"X", "Y", "Z"}, static_cast<int>(l.axis));
    auto* joint = new QHBoxLayout;
    joint->addWidget(r.type, 1);
    joint->addWidget(new QLabel("axis"));
    joint->addWidget(r.axis);
    form->addRow("Joint", joint);

    r.mode = makeCombo("mode", {"Cartesian (x, y, z)", "Polar (r, θ, z)"},
                       l.mode == OffsetMode::Cartesian ? 0 : 1);
    form->addRow("Offset", r.mode);

    r.a = makeSpin(k + "a", 0.0, -5.0, 5.0, 4, 0.01, kMetre);
    r.b = makeSpin(k + "b", 0.0, -5.0, 5.0, 4, 0.01, kMetre);
    r.c = makeSpin(k + "c", 0.0, -5.0, 5.0, 4, 0.01, kMetre);
    r.c->setPrefix("z ");
    r.c->setToolTip("Offset along the previous frame's z axis.");
    auto* offsets = new QHBoxLayout;
    offsets->addWidget(r.a);
    offsets->addWidget(r.b);
    offsets->addWidget(r.c);
    form->addRow("", offsets);

    r.lower = makeSpin(k + "lower", 0.0, -360.0, 360.0, 1, 5.0, kDeg);
    r.upper = makeSpin(k + "upper", 0.0, -360.0, 360.0, 1, 5.0, kDeg);
    r.lower->setPrefix("min ");
    r.upper->setPrefix("max ");
    auto* limits = new QHBoxLayout;
    limits->addWidget(r.lower);
    limits->addWidget(r.upper);
    form->addRow("Limits", limits);

    r.mass = makeSpin(k + "mass", l.mass, 0.001, 1000.0, 3, 0.1, QStringLiteral(" kg"));
    form->addRow("Mass", r.mass);

    applyMode(r, l.mode == OffsetMode::Polar, l.a, l.b, l.c);
    applyLimits(r, l.type == ArmJointType::Revolute, l.lower, l.upper);

    // Switching the offset mode converts the numbers so the joint stays where it is.
    connect(r.mode, &QComboBox::currentIndexChanged, this, [this, r](int) {
      const bool polar = r.mode->currentIndex() == 1;
      double a = r.a->value(), b = r.b->value();
      const double c = r.c->value();
      if (polar) {  // (x, y) -> (r, theta)
        const double x = a, y = b;
        a = std::hypot(x, y);
        b = a > 1e-12 ? std::atan2(y, x) * 180.0 / kPiD : 0.0;
      } else {      // (r, theta) -> (x, y)
        const double radius = a, theta = b * kPiD / 180.0;
        a = radius * std::cos(theta);
        b = radius * std::sin(theta);
      }
      applyMode(r, polar, a, b, c);
      emit changed();
    });
    // Switching between revolute and prismatic resets the limits to sensible defaults.
    connect(r.type, &QComboBox::currentIndexChanged, this, [this, r](int) {
      const bool revolute = r.type->currentIndex() == 0;
      applyLimits(r, revolute, revolute ? -170.0 : 0.0, revolute ? 170.0 : 0.3);
      emit changed();
    });
    connect(r.axis, &QComboBox::currentIndexChanged, this, &ConfigPage::changed);
    for (QDoubleSpinBox* s : {r.a, r.b, r.c, r.lower, r.upper, r.mass}) {
      connect(s, &QDoubleSpinBox::valueChanged, this, &ConfigPage::changed);
    }

    rowsLayout_->addWidget(r.box);
    rows_.push_back(r);
  }

  void resizeRows(int n) {
    while (static_cast<int>(rows_.size()) < n) {
      ArmLinkConfig next;  // a typical follow-on joint: revolute about Y, 0.2 m further up
      next.type = ArmJointType::Revolute;
      next.axis = ArmAxis::Y;
      next.mode = OffsetMode::Cartesian;
      next.a = 0.0;
      next.b = 0.0;
      next.c = 0.2;
      next.lower = -120.0;
      next.upper = 120.0;
      next.mass = 0.5;
      addRow(next);
    }
    while (static_cast<int>(rows_.size()) > n) {
      delete rows_.back().box;
      rows_.pop_back();
    }
    emit changed();
  }

  QDoubleSpinBox *baseRadius_, *baseHeight_, *baseMass_, *linkRadius_, *effort_, *velocity_;
  QDoubleSpinBox *toolX_, *toolY_, *toolZ_;
  QSpinBox* count_;
  QVBoxLayout* rowsLayout_;
  std::vector<ArmRow> rows_;
};

}  // namespace

ConfigPage* createPage(RobotType type, QWidget* parent) {
  ConfigPage* page = nullptr;
  switch (type) {
    case RobotType::Hexapod: page = new HexapodPage; break;
    case RobotType::Arm: page = new ArmPage; break;
    case RobotType::DiffDrive: page = new DiffDrivePage; break;
    case RobotType::FourWheel: page = new FourWheelPage; break;
    case RobotType::Ackermann: page = new AckermannPage; break;
    case RobotType::Quadruped: page = new QuadrupedPage; break;
    case RobotType::Biped: page = new BipedPage; break;
  }
  if (page && parent) page->setParent(parent);
  return page;
}

}  // namespace urdfgui
