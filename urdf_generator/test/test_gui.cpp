// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// Drives the real main window without a display (QT_QPA_PLATFORM=offscreen).
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QRegularExpression>
#include <QSlider>
#include <QMouseEvent>
#include <QSpinBox>
#include <QTemporaryDir>
#include <QWheelEvent>
#include <QXmlStreamReader>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "main_window.hpp"
#include "sensor_panel.hpp"
#include "urdf_generator/sensors.hpp"
#include "urdf_generator/builders.hpp"
#include "urdf_generator/emitter.hpp"
#include "urdf_generator/scene.hpp"

using namespace urdfgui;
using urdfgen::RobotType;

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

#define CHECK_NEAR(a, b, tol)                                                                   \
  do {                                                                                          \
    ++g_checks;                                                                                 \
    const double va = (a), vb = (b);                                                            \
    if (!(std::fabs(va - vb) <= (tol))) {                                                       \
      ++g_failed;                                                                               \
      std::fprintf(stderr, "FAIL %s:%d: %s = %.9g, expected %.9g\n", __FILE__, __LINE__, #a, va, \
                   vb);                                                                         \
    }                                                                                           \
  } while (0)

namespace {

template <class T>
T* find(QWidget* root, const char* name) {
  T* w = root->findChild<T*>(name);
  if (!w) std::fprintf(stderr, "  (widget '%s' not found)\n", name);
  return w;
}

bool wellFormed(const QString& xml) {
  QXmlStreamReader r(xml);
  while (!r.atEnd()) r.readNext();
  return !r.hasError();
}

void setValue(QWidget* page, const char* name, double v) {
  if (auto* s = find<QDoubleSpinBox>(page, name)) s->setValue(v);
}

// ------------------------------------------------------------------- tests

void testDefaultsMatchCore(MainWindow& win) {
  for (RobotType t : urdfgen::allRobotTypes()) {
    win.selectType(t);
    win.refreshNow();
    const std::string id = urdfgen::robotTypeId(t);
    if (win.hasErrors()) std::fprintf(stderr, "  [%s] has errors\n", id.c_str());
    CHECK(!win.hasErrors());
    CHECK(win.canSave());
    CHECK(win.currentPage()->type() == t);
    CHECK(!win.currentPage()->hint().isEmpty());

    // The page's default values must reproduce the library defaults exactly.
    const urdfgen::Robot ref = urdfgen::buildDefault(t);
    const bool urdfSame = win.urdfText() == QString::fromStdString(urdfgen::emitUrdf(ref));
    const bool xacroSame = win.xacroText() == QString::fromStdString(urdfgen::emitXacro(ref));
    if (!urdfSame || !xacroSame) std::fprintf(stderr, "  [%s] GUI default differs from core default\n", id.c_str());
    CHECK(urdfSame);
    CHECK(xacroSame);
    CHECK(wellFormed(win.urdfText()));
    CHECK(wellFormed(win.xacroText()));
  }
}

void testHexapodEdit(MainWindow& win) {
  win.selectType(RobotType::Hexapod);
  QWidget* page = win.currentPage();
  setValue(page, "femur_length", 0.13);
  setValue(page, "body_radius", 0.2);
  win.refreshNow();
  CHECK(!win.hasErrors());
  CHECK(win.xacroText().contains("<xacro:property name=\"femur_length\" value=\"0.13\"/>"));
  CHECK(win.xacroText().contains("<xacro:property name=\"body_radius\" value=\"0.2\"/>"));
  CHECK(win.urdfText().contains("length=\"0.13\""));
  CHECK(win.urdfText().contains("<joint name=\"leg1_femur_joint\""));
  // Restore for later tests.
  setValue(page, "femur_length", 0.10);
  setValue(page, "body_radius", 0.12);
}

void testArm(MainWindow& win) {
  win.selectType(RobotType::Arm);
  QWidget* page = win.currentPage();
  auto* count = find<QSpinBox>(page, "link_count");
  CHECK(count && count->value() == 3);
  win.refreshNow();
  CHECK(win.urdfText().contains("<link name=\"tool0\"/>"));
  CHECK(!win.urdfText().contains("link_4"));

  // More links.
  count->setValue(5);
  win.refreshNow();
  CHECK(!win.hasErrors());
  CHECK(win.urdfText().contains("<joint name=\"joint_5\""));
  CHECK(win.urdfText().contains("<link name=\"link_5\">"));
  CHECK(find<QComboBox>(page, "link5_type") != nullptr);

  // Fewer links, and the removed rows really go away.
  count->setValue(2);
  win.refreshNow();
  CHECK(!win.urdfText().contains("joint_3"));
  CHECK(page->findChild<QComboBox*>("link3_type") == nullptr);
  count->setValue(3);
  win.refreshNow();

  // Revolute <-> sliding: limits are reset to sensible values for the joint type.
  auto* type1 = find<QComboBox>(page, "link1_type");
  auto* lower1 = find<QDoubleSpinBox>(page, "link1_lower");
  auto* upper1 = find<QDoubleSpinBox>(page, "link1_upper");
  CHECK(type1 && lower1 && upper1);
  if (type1 && lower1 && upper1) {
    type1->setCurrentIndex(1);  // prismatic
    CHECK_NEAR(lower1->value(), 0.0, 1e-9);
    CHECK_NEAR(upper1->value(), 0.3, 1e-9);
    win.refreshNow();
    CHECK(win.urdfText().contains("<joint name=\"joint_1\" type=\"prismatic\">"));
    CHECK(win.urdfText().contains("lower=\"0\" upper=\"0.3\""));
    type1->setCurrentIndex(0);  // back to revolute
    CHECK_NEAR(upper1->value(), 170.0, 1e-9);
    win.refreshNow();
    CHECK(win.urdfText().contains("<joint name=\"joint_1\" type=\"revolute\">"));
  }

  // Cartesian <-> polar conversion keeps the joint where it is.
  auto* mode2 = find<QComboBox>(page, "link2_mode");
  auto* a2 = find<QDoubleSpinBox>(page, "link2_a");
  auto* b2 = find<QDoubleSpinBox>(page, "link2_b");
  auto* c2 = find<QDoubleSpinBox>(page, "link2_c");
  CHECK(mode2 && a2 && b2 && c2);
  if (mode2 && a2 && b2 && c2) {
    a2->setValue(0.1);
    b2->setValue(0.1);
    c2->setValue(0.2);
    mode2->setCurrentIndex(1);  // polar
    CHECK_NEAR(a2->value(), std::sqrt(0.02), 1e-4);
    CHECK_NEAR(b2->value(), 45.0, 0.01);
    CHECK_NEAR(c2->value(), 0.2, 1e-9);
    win.refreshNow();
    CHECK(!win.hasErrors());
    CHECK(win.xacroText().contains("joint2_theta"));
    CHECK(win.urdfText().contains("<joint name=\"joint_2\""));
    mode2->setCurrentIndex(0);  // back to cartesian
    CHECK_NEAR(a2->value(), 0.1, 2e-4);
    CHECK_NEAR(b2->value(), 0.1, 2e-4);
    win.refreshNow();
    CHECK(!win.xacroText().contains("joint2_theta"));
    // Put it back.
    a2->setValue(0.0);
    b2->setValue(0.0);
    c2->setValue(0.10);
  }

  // Polar offsets written as r and theta properties.
  auto* mode3 = find<QComboBox>(page, "link3_mode");
  if (mode3) {
    mode3->setCurrentIndex(1);
    setValue(page, "link3_a", 0.12);
    setValue(page, "link3_b", 90.0);
    win.refreshNow();
    CHECK(win.xacroText().contains("<xacro:property name=\"joint3_r\" value=\"0.12\"/>"));
    CHECK(win.xacroText().contains("<xacro:property name=\"joint3_theta\" value=\"1.5707963\"/>"));
  }
}

void testWheeled(MainWindow& win) {
  win.selectType(RobotType::DiffDrive);
  QWidget* page = win.currentPage();
  auto* shape = find<QComboBox>(page, "chassis_shape");
  CHECK(shape != nullptr);
  if (shape) {
    shape->setCurrentIndex(1);  // circular
    setValue(page, "chassis_radius", 0.2);
    win.refreshNow();
    CHECK(!win.hasErrors());
    CHECK(win.xacroText().contains("chassis_radius"));
    CHECK(!win.xacroText().contains("chassis_length"));
    CHECK(win.urdfText().contains("<cylinder radius=\"0.2\" length=\"0.1\"/>"));
    shape->setCurrentIndex(0);
    win.refreshNow();
    CHECK(win.xacroText().contains("chassis_length"));
  }
  setValue(page, "wheel_radius", 0.08);
  setValue(page, "wheel_y", 0.15);
  win.refreshNow();
  CHECK(win.urdfText().contains("<joint name=\"left_wheel_joint\" type=\"continuous\">"));
  CHECK(win.urdfText().contains("<origin xyz=\"0 0.15 -0.03\" rpy=\"0 0 0\"/>"));
  CHECK(win.urdfText().contains("<origin xyz=\"0 -0.15 -0.03\" rpy=\"0 0 0\"/>"));

  auto* casters = find<QCheckBox>(page, "casters");
  if (casters) {
    casters->setChecked(false);
    win.refreshNow();
    CHECK(!win.urdfText().contains("caster"));
    casters->setChecked(true);
    win.refreshNow();
    CHECK(win.urdfText().contains("front_caster_joint"));
  }

  win.selectType(RobotType::Ackermann);
  page = win.currentPage();
  setValue(page, "wheelbase", 0.5);
  setValue(page, "steer_limit", 25.0);
  win.refreshNow();
  CHECK(!win.hasErrors());
  CHECK(win.xacroText().contains("<xacro:property name=\"wheelbase\" value=\"0.5\"/>"));
  CHECK(win.urdfText().contains("<joint name=\"front_left_steer_joint\" type=\"revolute\">"));
  CHECK(win.urdfText().contains("lower=\"-0.43633231\" upper=\"0.43633231\""));

  // Leave the pages as they were, so that later tests start from the defaults.
  setValue(page, "wheelbase", 0.30);
  setValue(page, "steer_limit", 30.0);
  win.refreshNow();
  win.selectType(RobotType::DiffDrive);
  setValue(win.currentPage(), "wheel_radius", 0.05);
  setValue(win.currentPage(), "wheel_y", 0.12);
  win.refreshNow();
  CHECK(win.urdfText().toStdString() == urdfgen::emitUrdf(urdfgen::buildDefault(RobotType::DiffDrive)));
  win.selectType(RobotType::Ackermann);
  CHECK(win.urdfText().toStdString() == urdfgen::emitUrdf(urdfgen::buildDefault(RobotType::Ackermann)));
}


void mouse(QWidget* w, QEvent::Type type, QPoint pos, Qt::MouseButton button, Qt::MouseButtons buttons,
           Qt::KeyboardModifiers mods = Qt::NoModifier) {
  QMouseEvent e(type, QPointF(pos), QPointF(w->mapToGlobal(pos)), button, buttons, mods);
  QApplication::sendEvent(w, &e);
}

void drag(QWidget* w, Qt::MouseButton button, QPoint from, QPoint to, Qt::KeyboardModifiers mods = Qt::NoModifier) {
  mouse(w, QEvent::MouseButtonPress, from, button, button, mods);
  mouse(w, QEvent::MouseMove, to, Qt::NoButton, button, mods);
  mouse(w, QEvent::MouseButtonRelease, to, button, Qt::NoButton, mods);
}

void wheel(QWidget* w, int delta) {
  const QPointF centre(w->width() / 2.0, w->height() / 2.0);
  QWheelEvent e(centre, w->mapToGlobal(centre), QPoint(), QPoint(0, delta), Qt::NoButton, Qt::NoModifier,
                Qt::NoScrollPhase, false);
  QApplication::sendEvent(w, &e);
}

int differingPixels(const QImage& a, const QImage& b) {
  int n = 0;
  for (int y = 0; y < a.height(); ++y) {
    for (int x = 0; x < a.width(); ++x) n += a.pixel(x, y) != b.pixel(x, y);
  }
  return n;
}

void testView(MainWindow& win) {
  RobotView* view = win.view();
  CHECK(view != nullptr);
  if (!view) return;

  // Every robot type shows up in the view, framed, with as many bodies as the library builds.
  for (RobotType t : urdfgen::allRobotTypes()) {
    win.selectType(t);
    win.refreshNow();
    const urdfgen::Scene expected = urdfgen::buildScene(urdfgen::buildDefault(t));
    CHECK(!view->scene().bodies.empty());
    CHECK(view->scene().bodies.size() == expected.bodies.size());
    CHECK(view->camera().contains(view->scene(), view->width(), view->height()));
    CHECK(!view->userMovedCamera());
  }

  // The picture really contains the robot: it differs from an empty scene's picture.
  win.selectType(RobotType::Hexapod);
  win.refreshNow();
  const QSize size(480, 320);
  const QImage withRobot = view->renderImage(size);
  CHECK(withRobot.size() == size);
  RobotView blank;
  blank.resize(size);
  CHECK(differingPixels(withRobot, blank.renderImage(size)) > 3000);

  // Live update: the scene follows the edit. While the robot stays in view the camera stays put,
  // so the change is visible against the grid (shrinking never zooms in).
  const urdfgen::Scene before = view->scene();
  const urdfgen::Camera camBefore = view->camera();
  setValue(win.currentPage(), "femur_length", 0.08);
  win.refreshNow();
  CHECK(view->scene().max.x < before.max.x - 0.015);
  CHECK(view->scene().bodies.size() == before.bodies.size());
  CHECK_NEAR(view->camera().distance, camBefore.distance, 1e-12);
  CHECK_NEAR(view->camera().target.x, camBefore.target.x, 1e-12);
  setValue(win.currentPage(), "femur_length", 0.102);  // a tiny growth also stays inside the margin
  win.refreshNow();
  CHECK(view->scene().max.x > before.max.x);
  CHECK_NEAR(view->camera().distance, camBefore.distance, 1e-12);
  setValue(win.currentPage(), "femur_length", 0.10);
  win.refreshNow();

  // An edit that makes the robot much bigger zooms out by itself while the user has not moved the camera.
  setValue(win.currentPage(), "tibia_length", 1.2);
  win.refreshNow();
  CHECK(view->camera().contains(view->scene(), view->width(), view->height()));
  CHECK(view->camera().distance > camBefore.distance);
  setValue(win.currentPage(), "tibia_length", 0.15);
  win.refreshNow();
  view->resetView();

  // Orbit with the left mouse button.
  const double az0 = view->camera().azimuthDeg, el0 = view->camera().elevationDeg;
  drag(view, Qt::LeftButton, QPoint(200, 150), QPoint(250, 120));
  CHECK(view->userMovedCamera());
  CHECK_NEAR(view->camera().azimuthDeg, az0 - 50 * 0.4, 1e-9);
  CHECK_NEAR(view->camera().elevationDeg, el0 - 30 * 0.4, 1e-9);
  drag(view, Qt::LeftButton, QPoint(200, 150), QPoint(200, 5000));  // cannot flip over the top
  CHECK(view->camera().elevationDeg <= 89.5 + 1e-9);

  // Pan with the right button moves the target sideways in the screen plane, not the distance.
  view->resetView();
  const urdfgen::Camera c0 = view->camera();
  drag(view, Qt::RightButton, QPoint(200, 150), QPoint(260, 150));
  const urdfgen::Camera c1 = view->camera();
  urdfgen::Vec3 right, up, back;
  c0.axes(&right, &up, &back);
  const urdfgen::Vec3 moved = c1.target - c0.target;
  CHECK(urdfgen::length(moved) > 1e-3);
  CHECK(std::fabs(urdfgen::dot(moved, up)) < 1e-9 && std::fabs(urdfgen::dot(moved, back)) < 1e-9);
  CHECK(urdfgen::dot(moved, right) < 0);  // dragging right pulls the scene right, so the target moves left
  CHECK_NEAR(c1.distance, c0.distance, 1e-12);

  // Shift + left drag pans too.
  view->resetView();
  drag(view, Qt::LeftButton, QPoint(200, 150), QPoint(240, 150), Qt::ShiftModifier);
  CHECK(urdfgen::length(view->camera().target - c0.target) > 1e-3);
  CHECK_NEAR(view->camera().azimuthDeg, c0.azimuthDeg, 1e-12);

  // Wheel zooms, within limits.
  view->resetView();
  const double d0 = view->camera().distance;
  wheel(view, 120);
  CHECK(view->camera().distance < d0);
  wheel(view, -240);
  CHECK(view->camera().distance > d0);
  for (int i = 0; i < 80; ++i) wheel(view, 120);
  CHECK(view->camera().distance >= 0.2 * view->scene().radius() - 1e-12);
  for (int i = 0; i < 200; ++i) wheel(view, -120);
  CHECK(view->camera().distance <= 30.0 * view->scene().radius() + 1e-9);

  // A camera the user has moved is left alone when the robot grows; double-click puts everything back.
  view->resetView();
  drag(view, Qt::LeftButton, QPoint(200, 150), QPoint(215, 150));
  CHECK(view->userMovedCamera());
  setValue(win.currentPage(), "tibia_length", 1.2);
  win.refreshNow();
  const double movedDistance = view->camera().distance;
  CHECK(!view->camera().contains(view->scene(), view->width(), view->height()));
  mouse(view, QEvent::MouseButtonDblClick, QPoint(100, 100), Qt::LeftButton, Qt::LeftButton);
  CHECK(!view->userMovedCamera());
  CHECK(view->camera().contains(view->scene(), view->width(), view->height()));
  CHECK(view->camera().distance != movedDistance);
  setValue(win.currentPage(), "tibia_length", 0.15);
  win.refreshNow();

  // Presets.
  view->setPreset(RobotView::Preset::Front);
  CHECK_NEAR(view->camera().azimuthDeg, 0.0, 1e-12);
  CHECK_NEAR(view->camera().elevationDeg, 0.0, 1e-12);
  view->setPreset(RobotView::Preset::Side);
  CHECK_NEAR(view->camera().azimuthDeg, -90.0, 1e-12);
  view->setPreset(RobotView::Preset::Top);
  CHECK_NEAR(view->camera().elevationDeg, 89.5, 1e-12);
  CHECK(view->camera().contains(view->scene(), view->width(), view->height()));
  view->resetView();

  // Link frames are an overlay: toggling them changes the picture and nothing else.
  win.selectType(RobotType::Arm);
  win.refreshNow();
  const QImage plain = view->renderImage(size);
  win.framesBox()->setChecked(true);
  CHECK(view->showFrames());
  CHECK(differingPixels(plain, view->renderImage(size)) > 50);
  win.framesBox()->setChecked(false);
  CHECK(differingPixels(plain, view->renderImage(size)) == 0);

  // Rendering is deterministic and not blank for every robot, from every preset.
  for (RobotType t : urdfgen::allRobotTypes()) {
    win.selectType(t);
    win.refreshNow();
    for (auto preset : {RobotView::Preset::Isometric, RobotView::Preset::Front, RobotView::Preset::Side, RobotView::Preset::Top}) {
      view->setPreset(preset);
      const QImage a = view->renderImage(size), b = view->renderImage(size);
      CHECK(differingPixels(a, b) == 0);
      CHECK(differingPixels(a, blank.renderImage(size)) > 1500);
    }
  }

  // With an error in the configuration the view keeps showing the last valid robot.
  win.selectType(RobotType::Quadruped);
  win.refreshNow();
  const std::size_t bodies = view->scene().bodies.size();
  if (auto* name = find<QLineEdit>(win.currentPage(), "robot_name")) {
    name->setText("");
    win.refreshNow();
    CHECK(win.hasErrors());
    CHECK(view->scene().bodies.size() == bodies);
    name->setText("quadruped");
    win.refreshNow();
    CHECK(!win.hasErrors());
  }

  // Switching type re-frames the new robot even if the camera was moved.
  drag(view, Qt::LeftButton, QPoint(200, 150), QPoint(230, 150));
  CHECK(view->userMovedCamera());
  win.selectType(RobotType::Biped);
  CHECK(!view->userMovedCamera());
  CHECK(view->camera().contains(view->scene(), view->width(), view->height()));
}

const urdfgen::SceneFrame* frameOf(const urdfgen::Scene& s, const std::string& link) {
  for (const auto& f : s.frames) {
    if (f.link == link) return &f;
  }
  return nullptr;
}

constexpr double kDeg = urdfgen::kPi / 180.0;

void testJointSliders(MainWindow& win) {
  RobotView* view = win.view();
  JointPanel* panel = win.jointPanel();
  CHECK(view && panel);
  if (!view || !panel) return;

  // ---- hexapod: one row per movable joint, at rest
  win.selectType(RobotType::Hexapod);
  win.refreshNow();
  CHECK(panel->jointCount() == 18);
  CHECK(panel->jointNames().first() == "leg1_coxa_joint");
  CHECK(panel->jointNames().last() == "leg6_tibia_joint");
  auto* femurSlider = find<QSlider>(panel, "joint_leg1_femur_joint_slider");
  auto* femurSpin = find<QDoubleSpinBox>(panel, "joint_leg1_femur_joint_spin");
  CHECK(femurSlider && femurSpin);
  if (!femurSlider || !femurSpin) return;
  CHECK(femurSlider->minimum() == 0 && femurSlider->maximum() == 1000);
  CHECK(femurSlider->value() == 500);  // symmetric limits: zero is in the middle
  CHECK_NEAR(femurSpin->value(), 0.0, 1e-9);
  CHECK_NEAR(femurSpin->minimum(), -90.0, 1e-6);
  CHECK_NEAR(femurSpin->maximum(), 90.0, 1e-6);
  CHECK(femurSpin->suffix().contains(QString::fromUtf8("°")));
  for (const auto& kv : panel->pose()) CHECK_NEAR(kv.second, 0.0, 1e-12);

  const urdfgen::Robot hexRobot = urdfgen::buildDefault(RobotType::Hexapod);
  const urdfgen::Scene restScene = view->scene();
  const urdfgen::Camera cam0 = view->camera();
  const QSize size(480, 320);
  const QImage restImage = view->renderImage(size);

  // ---- typing a value moves the joint, the slider and the 3D scene
  femurSpin->setValue(30.0);
  CHECK_NEAR(panel->jointValue("leg1_femur_joint"), 30.0 * kDeg, 1e-9);
  CHECK(femurSlider->value() == 667);  // 30 of +-90 -> 2/3 of the range
  urdfgen::JointPose expectedPose;
  expectedPose["leg1_femur_joint"] = 30.0 * kDeg;
  {
    const urdfgen::Scene expected = urdfgen::buildScene(hexRobot, expectedPose);
    const auto* got = frameOf(view->scene(), "leg1_foot_link");
    const auto* want = frameOf(expected, "leg1_foot_link");
    CHECK(got && want);
    if (got && want) {
      CHECK_NEAR(got->pose.t.x, want->pose.t.x, 1e-9);
      CHECK_NEAR(got->pose.t.z, want->pose.t.z, 1e-9);
      CHECK(std::fabs(got->pose.t.z - frameOf(restScene, "leg1_foot_link")->pose.t.z) > 0.01);
    }
    CHECK_NEAR(view->scene().groundZ, restScene.groundZ, 1e-12);  // the ground grid stays where it was
  }
  CHECK(differingPixels(restImage, view->renderImage(size)) > 50);
  CHECK_NEAR(view->camera().distance, cam0.distance, 1e-12);  // a pose change does not re-aim the camera
  CHECK(!view->userMovedCamera());

  // ---- dragging the slider moves the value and the spin box, within the limits
  femurSlider->setValue(750);
  CHECK_NEAR(panel->jointValue("leg1_femur_joint"), 45.0 * kDeg, 1e-9);
  CHECK_NEAR(femurSpin->value(), 45.0, 0.051);
  femurSlider->setValue(1000);
  CHECK_NEAR(panel->jointValue("leg1_femur_joint"), 90.0 * kDeg, 1e-9);
  femurSpin->setValue(500.0);  // the spin box does not go past its range
  CHECK_NEAR(femurSpin->value(), 90.0, 1e-9);
  CHECK(!panel->setJointValue("no_such_joint", 1.0));
  CHECK(panel->setJointValue("leg1_femur_joint", 10.0));  // radians, clamped to the limit
  CHECK_NEAR(panel->jointValue("leg1_femur_joint"), 90.0 * kDeg, 1e-9);

  // ---- the pose survives edits of the robot; limits clamp it
  femurSpin->setValue(30.0);
  setValue(win.currentPage(), "femur_length", 0.13);
  win.refreshNow();
  CHECK(panel->jointCount() == 18);
  CHECK_NEAR(panel->jointValue("leg1_femur_joint"), 30.0 * kDeg, 1e-9);
  {
    urdfgen::HexapodConfig c;
    c.femurLength = 0.13;
    const urdfgen::Scene expected = urdfgen::buildScene(urdfgen::buildHexapod(c), expectedPose);
    CHECK_NEAR(frameOf(view->scene(), "leg1_foot_link")->pose.t.x, frameOf(expected, "leg1_foot_link")->pose.t.x, 1e-9);
  }
  setValue(win.currentPage(), "femur_length", 0.10);
  win.refreshNow();
  setValue(win.currentPage(), "femur_limit", 20.0);  // below the current 30 degrees
  win.refreshNow();
  CHECK_NEAR(panel->jointValue("leg1_femur_joint"), 20.0 * kDeg, 1e-9);
  femurSpin = find<QDoubleSpinBox>(panel, "joint_leg1_femur_joint_spin");  // rows may have been rebuilt
  femurSlider = find<QSlider>(panel, "joint_leg1_femur_joint_slider");
  CHECK(femurSpin && femurSlider);
  if (femurSpin && femurSlider) {
    CHECK_NEAR(femurSpin->maximum(), 20.0, 1e-6);
    CHECK_NEAR(femurSpin->value(), 20.0, 1e-6);
    CHECK(femurSlider->value() == 1000);
  }
  {
    urdfgen::HexapodConfig c;
    c.femurLimitDeg = 20.0;
    urdfgen::JointPose p;
    p["leg1_femur_joint"] = 20.0 * kDeg;
    const urdfgen::Scene expected = urdfgen::buildScene(urdfgen::buildHexapod(c), p);
    CHECK_NEAR(frameOf(view->scene(), "leg1_foot_link")->pose.t.z, frameOf(expected, "leg1_foot_link")->pose.t.z, 1e-9);
  }
  setValue(win.currentPage(), "femur_limit", 90.0);
  win.refreshNow();

  // ---- the generated files never depend on the pose
  const QString urdfPosed = win.urdfText();
  win.currentPage();  // (no-op, keeps the pose)
  panel->resetPose();
  win.refreshNow();
  CHECK(win.urdfText() == urdfPosed);

  // ---- Reset pose puts everything back
  panel->setJointValue("leg1_coxa_joint", 0.3);
  panel->setJointValue("leg4_tibia_joint", -0.7);
  auto* reset = find<QPushButton>(panel, "reset_pose");
  CHECK(reset != nullptr);
  if (reset) reset->click();
  for (const auto& kv : panel->pose()) CHECK_NEAR(kv.second, 0.0, 1e-12);
  CHECK_NEAR(frameOf(view->scene(), "leg1_foot_link")->pose.t.x, frameOf(restScene, "leg1_foot_link")->pose.t.x, 1e-9);
  CHECK(differingPixels(restImage, view->renderImage(size)) == 0);

  // ---- switching the robot type forgets the pose
  panel->setJointValue("leg1_coxa_joint", 0.3);
  win.selectType(RobotType::Quadruped);
  win.refreshNow();
  CHECK(panel->jointCount() == 12);
  for (const auto& kv : panel->pose()) CHECK_NEAR(kv.second, 0.0, 1e-12);
  win.selectType(RobotType::Hexapod);
  win.refreshNow();
  CHECK_NEAR(panel->jointValue("leg1_coxa_joint"), 0.0, 1e-12);

  // ---- arm: rows follow the number of links, a changed joint type drops the old value
  win.selectType(RobotType::Arm);
  QWidget* page = win.currentPage();
  win.refreshNow();
  CHECK(panel->jointCount() == 3);
  panel->setJointValue("joint_1", 0.5);
  panel->setJointValue("joint_2", -0.4);
  auto* count = find<QSpinBox>(page, "link_count");
  if (count) count->setValue(4);
  win.refreshNow();
  CHECK(panel->jointCount() == 4);
  CHECK_NEAR(panel->jointValue("joint_1"), 0.5, 1e-9);
  CHECK_NEAR(panel->jointValue("joint_2"), -0.4, 1e-9);
  CHECK_NEAR(panel->jointValue("joint_4"), 0.0, 1e-9);
  if (count) count->setValue(2);
  win.refreshNow();
  CHECK(panel->jointCount() == 2);
  CHECK_NEAR(panel->jointValue("joint_2"), -0.4, 1e-9);
  if (count) count->setValue(3);
  win.refreshNow();
  CHECK_NEAR(panel->jointValue("joint_3"), 0.0, 1e-9);  // a joint that came back starts at rest
  auto* type1 = find<QComboBox>(page, "link1_type");
  if (type1) type1->setCurrentIndex(1);  // joint 1 becomes a slider (prismatic): 0.5 rad is not 0.5 m
  win.refreshNow();
  CHECK_NEAR(panel->jointValue("joint_1"), 0.0, 1e-9);
  auto* slide = find<QDoubleSpinBox>(panel, "joint_joint_1_spin");
  CHECK(slide != nullptr);
  if (slide) {
    CHECK(slide->suffix() == " m");
    CHECK_NEAR(slide->minimum(), 0.0, 1e-9);
    CHECK_NEAR(slide->maximum(), 0.3, 1e-9);
    slide->setValue(0.2);
    CHECK_NEAR(panel->jointValue("joint_1"), 0.2, 1e-9);
    const double toolZ = frameOf(view->scene(), "tool0")->pose.t.z;
    slide->setValue(0.0);
    CHECK_NEAR(toolZ - frameOf(view->scene(), "tool0")->pose.t.z, 0.2, 1e-9);  // the whole arm slid up
  }
  if (type1) type1->setCurrentIndex(0);
  win.refreshNow();

  // ---- the camera backs away, never re-aims, when a pose pushes the robot out of view
  win.selectType(RobotType::Arm);
  win.refreshNow();
  view->resetView();
  const urdfgen::Camera before = view->camera();
  panel->setJointValue("joint_2", 1.5);
  panel->setJointValue("joint_3", 1.5);
  CHECK(view->camera().contains(view->scene(), view->width(), view->height()));
  CHECK(view->camera().distance >= before.distance);
  CHECK_NEAR(view->camera().target.x, before.target.x, 1e-12);
  CHECK_NEAR(view->camera().target.z, before.target.z, 1e-12);
  CHECK(!view->userMovedCamera());
  const double backedAway = view->camera().distance;
  panel->resetPose();
  CHECK_NEAR(view->camera().distance, backedAway, 1e-12);  // and does not zoom back in on its own
  // A camera the user has framed is left alone.
  view->resetView();
  drag(view, Qt::LeftButton, QPoint(200, 150), QPoint(210, 150));
  const urdfgen::Camera framed = view->camera();
  panel->setJointValue("joint_2", 1.5);
  CHECK_NEAR(view->camera().distance, framed.distance, 1e-12);
  CHECK_NEAR(view->camera().azimuthDeg, framed.azimuthDeg, 1e-12);
  panel->resetPose();
  view->resetView();

  // ---- a continuous joint (wheel) turns without limit and without moving
  win.selectType(RobotType::DiffDrive);
  win.refreshNow();
  CHECK(panel->jointCount() == 2);
  const urdfgen::Scene wheelRest = view->scene();
  panel->setJointValue("left_wheel_joint", 100.0);
  CHECK_NEAR(panel->jointValue("left_wheel_joint"), 100.0, 1e-12);  // not limited to one turn
  const auto* w0 = frameOf(wheelRest, "left_wheel_link");
  const auto* w1 = frameOf(view->scene(), "left_wheel_link");
  CHECK(w0 && w1);
  if (w0 && w1) {
    CHECK_NEAR(w1->pose.t.x, w0->pose.t.x, 1e-12);
    CHECK_NEAR(w1->pose.r[0], std::cos(100.0), 1e-9);
    CHECK(std::fabs(w1->pose.r[0] - w0->pose.r[0]) > 0.1);
  }
  auto* wheelSpin = find<QDoubleSpinBox>(panel, "joint_left_wheel_joint_spin");
  if (wheelSpin) {
    CHECK(wheelSpin->minimum() <= -180.0 + 1e-6 && wheelSpin->maximum() >= 180.0 - 1e-6);
    wheelSpin->setValue(90.0);
    CHECK_NEAR(panel->jointValue("left_wheel_joint"), urdfgen::kPi / 2, 1e-9);
  }
  panel->resetPose();

  // ---- the panel can be hidden and shown
  CHECK(win.jointsBox()->isChecked());
  CHECK(!panel->isHidden());
  win.jointsBox()->setChecked(false);
  CHECK(panel->isHidden());
  win.jointsBox()->setChecked(true);
  CHECK(!panel->isHidden());

  // ---- the panel on its own: the empty state, and rows that are kept when nothing changed
  {
    JointPanel solo;
    solo.show();
    CHECK(solo.jointCount() == 0);
    CHECK(!find<QWidget>(&solo, "no_joints")->isHidden());  // "This robot has no movable joints."
    CHECK(!find<QPushButton>(&solo, "reset_pose")->isEnabled());

    std::vector<urdfgen::MovableJoint> joints(2);
    joints[0].name = "a"; joints[0].lower = -1.0; joints[0].upper = 1.0; joints[0].chain = 0;
    joints[1].name = "b"; joints[1].type = urdfgen::JointType::Prismatic; joints[1].lower = 0.0;
    joints[1].upper = 0.5; joints[1].chain = 1;
    solo.setJoints(joints);
    CHECK(solo.jointCount() == 2);
    CHECK(find<QWidget>(&solo, "no_joints")->isHidden());
    CHECK(find<QPushButton>(&solo, "reset_pose")->isEnabled());
    CHECK_NEAR(solo.jointValue("a"), 0.0, 1e-12);

    int changes = 0;
    QObject::connect(&solo, &JointPanel::poseChanged, [&changes]() { ++changes; });
    QSlider* sliderA = find<QSlider>(&solo, "joint_a_slider");
    solo.setJointValue("a", 0.5);
    solo.setJointValue("a", 0.5);  // no change, no signal
    CHECK(changes == 1);
    solo.setJoints(joints);  // identical joints: the rows (and their scroll position) stay
    CHECK(find<QSlider>(&solo, "joint_a_slider") == sliderA);
    CHECK(changes == 1);  // setJoints never emits
    CHECK_NEAR(solo.jointValue("a"), 0.5, 1e-12);
    joints[0].upper = 0.25;  // a narrower range clamps the value without emitting
    solo.setJoints(joints);
    CHECK_NEAR(solo.jointValue("a"), 0.25, 1e-12);
    CHECK(changes == 1);
    // A prismatic joint whose range excludes zero starts at its nearest limit.
    joints[1].lower = 0.1;
    solo.setJoints(joints);
    CHECK_NEAR(solo.jointValue("b"), 0.1, 1e-12);
    solo.setJointValue("b", 0.4);
    solo.resetPose();
    CHECK_NEAR(solo.jointValue("b"), 0.1, 1e-12);  // rest = zero limited into the range
    CHECK_NEAR(solo.jointValue("a"), 0.0, 1e-12);
    CHECK(changes == 3);  // set a, set b, reset (one signal, even though two joints moved)
    solo.clear();
    CHECK(solo.jointCount() == 0 && solo.pose().empty());
    CHECK(!find<QWidget>(&solo, "no_joints")->isHidden());
  }

  // ---- an error keeps the last valid robot, and the sliders still pose it
  win.selectType(RobotType::Biped);
  win.refreshNow();
  panel->setJointValue("left_knee_joint", 0.6);
  auto* name = find<QLineEdit>(win.currentPage(), "robot_name");
  if (name) {
    name->setText("");
    win.refreshNow();
    CHECK(win.hasErrors());
    CHECK_NEAR(panel->jointValue("left_knee_joint"), 0.6, 1e-12);
    const double kneeZ = frameOf(view->scene(), "left_foot_link")->pose.t.z;
    panel->setJointValue("left_knee_joint", 0.0);
    CHECK(frameOf(view->scene(), "left_foot_link")->pose.t.z != kneeZ);
    name->setText("biped");
    win.refreshNow();
  }
  panel->resetPose();
}

void setKind(QWidget* page, const char* name, int index) {
  if (auto* c = find<QComboBox>(page, name)) c->setCurrentIndex(index);
}

int countOf(const QString& text, const QString& needle) { return static_cast<int>(text.count(needle)); }

void testMecanumChoices(MainWindow& win) {
  using urdfgen::WheelKind;
  RobotView* view = win.view();

  // ---- differential drive: every wheel is Standard until told otherwise
  win.selectType(RobotType::DiffDrive);
  QWidget* page = win.currentPage();
  win.refreshNow();
  auto* left = find<QComboBox>(page, "left_wheel_type");
  auto* right = find<QComboBox>(page, "right_wheel_type");
  auto* rollers = find<QSpinBox>(page, "mecanum_rollers");
  CHECK(left && right && rollers);
  if (!left || !right || !rollers) return;
  CHECK(left->count() == 2 && left->itemText(0) == "Standard" && left->itemText(1) == "Mecanum");
  CHECK(left->currentIndex() == 0 && right->currentIndex() == 0);
  CHECK(rollers->value() == 8 && rollers->minimum() == 3 && rollers->maximum() == 16);
  CHECK(!rollers->isEnabled());  // nothing to count while every wheel is a plain wheel
  CHECK(!win.urdfText().contains("roller"));
  const int plainBodies = static_cast<int>(view->scene().bodies.size());

  // ---- the left wheel becomes a mecanum wheel
  left->setCurrentIndex(1);
  CHECK(rollers->isEnabled());
  win.refreshNow();
  CHECK(!win.hasErrors());
  {
    urdfgen::DiffDriveConfig c;
    c.leftWheel = WheelKind::Mecanum;
    CHECK(win.urdfText().toStdString() == urdfgen::emitUrdf(urdfgen::buildDiffDrive(c)));
    CHECK(win.xacroText().toStdString() == urdfgen::emitXacro(urdfgen::buildDiffDrive(c)));
  }
  CHECK(countOf(win.urdfText(), "<link name=\"left_roller_") == 8);
  CHECK(!win.urdfText().contains("right_roller"));
  CHECK(win.xacroText().contains("<xacro:mecanum_wheel prefix=\"left\""));
  CHECK(win.xacroText().contains("<xacro:drive_wheel prefix=\"right\""));
  CHECK(static_cast<int>(view->scene().bodies.size()) == plainBodies + 8);  // the 3D view shows the rollers
  CHECK(win.jointPanel()->jointCount() == 2);  // the rollers are fixed to the hub: no extra sliders
  CHECK(win.jointPanel()->jointNames().contains("left_wheel_joint"));

  // ---- the roller count
  rollers->setValue(5);
  win.refreshNow();
  CHECK(countOf(win.urdfText(), "<link name=\"left_roller_") == 5);
  {
    urdfgen::DiffDriveConfig c;
    c.leftWheel = WheelKind::Mecanum;
    c.mecanumRollers = 5;
    CHECK(win.urdfText().toStdString() == urdfgen::emitUrdf(urdfgen::buildDiffDrive(c)));
  }
  rollers->setValue(8);

  // ---- a turned mecanum wheel can be posed with its slider; the rollers go with it
  CHECK(frameOf(view->scene(), "left_roller_3_link") != nullptr);
  const urdfgen::Vec3 rollerRest = frameOf(view->scene(), "left_roller_3_link")->pose.t;
  win.jointPanel()->setJointValue("left_wheel_joint", 0.5);
  CHECK(urdfgen::length(frameOf(view->scene(), "left_roller_3_link")->pose.t - rollerRest) > 0.005);
  win.jointPanel()->resetPose();

  // ---- both wheels, then back
  right->setCurrentIndex(1);
  win.refreshNow();
  CHECK(countOf(win.urdfText(), "<link name=\"right_roller_") == 8);
  CHECK(!win.xacroText().contains("drive_wheel"));  // no plain wheel left: the macro goes away
  left->setCurrentIndex(0);
  right->setCurrentIndex(0);
  CHECK(!rollers->isEnabled());
  win.refreshNow();
  {
    CHECK(win.urdfText().toStdString() == urdfgen::emitUrdf(urdfgen::buildDefault(RobotType::DiffDrive)));
    CHECK(!win.xacroText().contains("mecanum"));
    CHECK(static_cast<int>(view->scene().bodies.size()) == plainBodies);
  }

  // ---- 4-wheeled: the quick buttons set every wheel
  win.selectType(RobotType::FourWheel);
  page = win.currentPage();
  win.refreshNow();
  const char* fourNames[4] = {"front_left_wheel_type", "front_right_wheel_type", "rear_left_wheel_type", "rear_right_wheel_type"};
  for (const char* n : fourNames) CHECK(find<QComboBox>(page, n) && find<QComboBox>(page, n)->currentIndex() == 0);
  auto* allMecanum = find<QPushButton>(page, "all_mecanum");
  auto* allStandard = find<QPushButton>(page, "all_standard");
  CHECK(allMecanum && allStandard);
  if (allMecanum && allStandard) {
    allMecanum->click();
    for (const char* n : fourNames) CHECK(find<QComboBox>(page, n)->currentIndex() == 1);
    CHECK(find<QSpinBox>(page, "mecanum_rollers")->isEnabled());
    win.refreshNow();
    CHECK(!win.hasErrors());
    urdfgen::FourWheelConfig c;
    c.frontLeftWheel = c.frontRightWheel = c.rearLeftWheel = c.rearRightWheel = WheelKind::Mecanum;
    CHECK(win.urdfText().toStdString() == urdfgen::emitUrdf(urdfgen::buildFourWheel(c)));
    CHECK(win.urdfText().count(QRegularExpression("<link name=\"[a-z_]+_roller_[0-9]+_link\">")) == 4 * 8);
    CHECK(countOf(win.urdfText(), "<link name=\"") == 38);
    CHECK(win.jointPanel()->jointCount() == 4);
    CHECK(view->scene().bodies.size() == 1 + 4 * 9);

    // Mixed: only some wheels, with their own handedness; the plain ones stay plain.
    allStandard->click();
    find<QComboBox>(page, "front_right_wheel_type")->setCurrentIndex(1);
    find<QComboBox>(page, "rear_left_wheel_type")->setCurrentIndex(1);
    find<QSpinBox>(page, "mecanum_rollers")->setValue(9);
    win.refreshNow();
    urdfgen::FourWheelConfig m;
    m.frontRightWheel = m.rearLeftWheel = WheelKind::Mecanum;
    m.mecanumRollers = 9;
    CHECK(win.urdfText().toStdString() == urdfgen::emitUrdf(urdfgen::buildFourWheel(m)));
    CHECK(win.xacroText().contains("<xacro:mecanum_wheel prefix=\"front_right\"") &&
          win.xacroText().contains("hand=\"-1\""));
    allStandard->click();
    find<QSpinBox>(page, "mecanum_rollers")->setValue(8);
    win.refreshNow();
    CHECK(win.urdfText().toStdString() == urdfgen::emitUrdf(urdfgen::buildDefault(RobotType::FourWheel)));
  }

  // ---- a wheel too wide for rollers is an error that says so; as a plain wheel it is fine
  setValue(page, "wheel_radius", 0.03);
  setValue(page, "wheel_width", 0.07);
  find<QComboBox>(page, "front_left_wheel_type")->setCurrentIndex(1);
  win.refreshNow();
  CHECK(win.hasErrors());
  CHECK(!win.canSave());
  bool said = false;
  if (auto* issues = win.findChild<QListWidget*>("issues")) {
    for (int i = 0; i < issues->count(); ++i) said = said || issues->item(i)->text().contains("mecanum wheel must be narrower");
  }
  CHECK(said);
  find<QComboBox>(page, "front_left_wheel_type")->setCurrentIndex(0);
  win.refreshNow();
  CHECK(!win.hasErrors());
  setValue(page, "wheel_radius", 0.06);
  setValue(page, "wheel_width", 0.04);
  win.refreshNow();
  CHECK(win.urdfText().toStdString() == urdfgen::emitUrdf(urdfgen::buildDefault(RobotType::FourWheel)));

  // ---- Ackermann: front wheels sit below the steering joints, rear wheels on the chassis
  win.selectType(RobotType::Ackermann);
  page = win.currentPage();
  win.refreshNow();
  setKind(page, "front_left_wheel_type", 1);
  setKind(page, "rear_right_wheel_type", 1);
  win.refreshNow();
  CHECK(!win.hasErrors());
  {
    urdfgen::AckermannConfig c;
    c.frontLeftWheel = c.rearRightWheel = WheelKind::Mecanum;
    CHECK(win.urdfText().toStdString() == urdfgen::emitUrdf(urdfgen::buildAckermann(c)));
    CHECK(win.xacroText().toStdString() == urdfgen::emitXacro(urdfgen::buildAckermann(c)));
  }
  CHECK(win.xacroText().contains("<xacro:steered_mecanum_wheel prefix=\"front_left\""));
  CHECK(win.xacroText().contains("<xacro:steered_wheel prefix=\"front_right\""));
  CHECK(win.xacroText().contains("<xacro:mecanum_wheel prefix=\"rear_right\""));
  CHECK(win.xacroText().contains("<xacro:drive_wheel prefix=\"rear_left\""));
  CHECK(win.jointPanel()->jointCount() == 6);
  // Steering with a mecanum front wheel moves the hub and its rollers together.
  const urdfgen::Scene ackRest = view->scene();
  win.jointPanel()->setJointValue("front_left_steer_joint", 0.4);
  {
    const auto find3 = [](const urdfgen::Scene& sc, const char* link) {
      for (const auto& f : sc.frames) {
        if (f.link == link) return f.pose.t;
      }
      return urdfgen::Vec3{};
    };
    const urdfgen::Vec3 hub0 = find3(ackRest, "front_left_wheel_link"), hub1 = find3(view->scene(), "front_left_wheel_link");
    const urdfgen::Vec3 r0 = find3(ackRest, "front_left_roller_2_link"), r1 = find3(view->scene(), "front_left_roller_2_link");
    CHECK(urdfgen::length(r1 - r0) > 0.005);  // the roller swung round the steering axis
    CHECK_NEAR(urdfgen::length(r1 - hub1), urdfgen::length(r0 - hub0), 1e-9);  // but kept its place on the hub
  }
  win.jointPanel()->resetPose();
  setKind(page, "front_left_wheel_type", 0);
  setKind(page, "rear_right_wheel_type", 0);
  win.refreshNow();
  CHECK(win.urdfText().toStdString() == urdfgen::emitUrdf(urdfgen::buildDefault(RobotType::Ackermann)));

  // ---- the saved files contain the rollers, and the pose never leaks into them
  setKind(page, "front_right_wheel_type", 1);
  win.refreshNow();
  QTemporaryDir dir;
  CHECK(dir.isValid());
  win.jointPanel()->setJointValue("front_right_wheel_joint", 1.0);
  QString err;
  CHECK(win.saveBothTo(dir.path(), &err));
  QFile u(QDir(dir.path()).filePath(page->findChild<QLineEdit*>("robot_name")->text() + ".urdf"));
  CHECK(u.open(QIODevice::ReadOnly));
  const QString saved = QString::fromUtf8(u.readAll());
  CHECK(saved.contains("front_right_roller_1_link") && wellFormed(saved));
  win.jointPanel()->resetPose();
  setKind(page, "front_right_wheel_type", 0);
  win.refreshNow();
}

// ------------------------------------------------------------------- sensors

void addSensor(QWidget* page, int typeIndex) {
  if (auto* type = find<QComboBox>(page, "sensor_add_type")) type->setCurrentIndex(typeIndex);
  if (auto* add = find<QPushButton>(page, "sensor_add")) add->click();
}

const urdfgen::SceneFrame* frameNamed(const urdfgen::Scene& s, const std::string& link) {
  for (const auto& f : s.frames) {
    if (f.link == link) return &f;
  }
  return nullptr;
}

QStringList itemsOf(QComboBox* c) {
  QStringList out;
  for (int i = 0; i < c->count(); ++i) out << c->itemText(i);
  return out;
}

bool issueMentions(MainWindow& win, const QString& needle) {
  auto* list = win.findChild<QListWidget*>("issues");
  if (!list) return false;
  for (int i = 0; i < list->count(); ++i) {
    if (list->item(i)->text().contains(needle)) return true;
  }
  return false;
}

void testSensors(MainWindow& win) {
  // Every robot type has its own, initially empty, sensor list.
  for (RobotType t : urdfgen::allRobotTypes()) {
    win.selectType(t);
    win.refreshNow();
    QWidget* page = win.currentPage();
    SensorPanel* panel = win.currentPage()->sensorPanel();
    CHECK(panel != nullptr && panel->count() == 0);
    auto* addType = find<QComboBox>(page, "sensor_add_type");
    CHECK(addType != nullptr && addType->count() == 5);
    if (addType) {
      CHECK(addType->itemText(0) == "Camera" && addType->itemText(1) == "IMU" && addType->itemText(2) == "LiDAR" &&
            addType->itemText(3) == "GPS" && addType->itemText(4) == "Odometry");
    }
    CHECK(find<QPushButton>(page, "sensor_add") != nullptr);
    CHECK(win.view()->scene().sensors.empty());
  }

  win.selectType(RobotType::Hexapod);
  win.refreshNow();
  QWidget* page = win.currentPage();
  SensorPanel* panel = win.currentPage()->sensorPanel();
  RobotView* view = win.view();
  const QString plainUrdf = win.urdfText();

  // ---- add a camera: it starts on top of the main body
  addSensor(page, 0);
  CHECK(panel->count() == 1);
  win.refreshNow();
  auto* name = find<QLineEdit>(page, "sensor1_name");
  auto* type = find<QComboBox>(page, "sensor1_type");
  auto* link = find<QComboBox>(page, "sensor1_link");
  auto* dist = find<QDoubleSpinBox>(page, "sensor1_distance");
  auto* slider = find<QSlider>(page, "sensor1_distance_slider");
  CHECK(name && type && link && dist && slider);  // a missing widget is a failure, not a silent skip
  if (!name || !type || !link || !dist || !slider) return;
  CHECK(name->text() == "camera");
  CHECK(type->currentIndex() == 0);
  CHECK(link->currentText() == "base_link");
  CHECK_NEAR(find<QDoubleSpinBox>(page, "sensor1_off_z")->value(), 0.025 + 0.015, 1e-9);
  CHECK(!win.hasErrors());
  CHECK(win.urdfText().contains("<link name=\"camera_link\">"));
  CHECK(win.urdfText().contains("<link name=\"camera_optical_frame\"/>"));
  CHECK(win.urdfText().contains("<joint name=\"camera_joint\" type=\"fixed\">"));
  CHECK(win.xacroText().contains("<xacro:property name=\"sensor_camera_offset_z\" value=\"0.04\"/>"));
  CHECK(wellFormed(win.xacroText()) && wellFormed(win.urdfText()));
  CHECK(view->scene().sensors.size() == 1);
  CHECK(view->scene().sensors[0].name == "camera" && view->scene().sensors[0].parent == "base_link");
  CHECK(!find<QLabel>(page, "no_sensors")->isVisible());

  // The link list is the robot's attachable links, in URDF order.
  QStringList expectedLinks;
  for (const std::string& l : urdfgen::attachableLinks(urdfgen::buildDefault(RobotType::Hexapod))) {
    expectedLinks << QString::fromStdString(l);
  }
  CHECK(itemsOf(link) == expectedLinks);
  CHECK(link->findText("leg6_tibia_link") >= 0);
  CHECK(link->findText("camera_link") < 0);  // not on a sensor

  // ---- slide along a link: the range is the length of the tibia
  auto* yaw = find<QDoubleSpinBox>(page, "sensor1_yaw");
  yaw->setValue(30);
  link->setCurrentIndex(link->findText("leg1_tibia_link"));
  win.refreshNow();
  CHECK_NEAR(dist->minimum(), 0.0, 1e-9);
  CHECK_NEAR(dist->maximum(), 0.15, 1e-9);
  CHECK_NEAR(dist->value(), 0.0, 1e-9);  // at the root of the new link
  CHECK_NEAR(find<QDoubleSpinBox>(page, "sensor1_off_z")->value(), 0.0, 1e-12);  // offsets start over
  CHECK_NEAR(yaw->value(), 30.0, 1e-9);                                             // the rotation stays
  CHECK(slider->isEnabled() && dist->isEnabled());
  slider->setValue(500);
  CHECK_NEAR(dist->value(), 0.075, 1e-6);
  win.refreshNow();
  {
    const auto& s = view->scene().sensors.at(0);
    CHECK(s.parent == "leg1_tibia_link");
    CHECK_NEAR(urdfgen::length(s.pose.t - s.root), 0.075, 1e-6);
    CHECK(s.pose.t.z < s.root.z);  // the tibia hangs down
    const urdfgen::SceneFrame* tibia = frameNamed(view->scene(), "leg1_tibia_link");
    CHECK(tibia != nullptr && std::fabs(tibia->pose.t.x - s.root.x) < 1e-12 && std::fabs(tibia->pose.t.z - s.root.z) < 1e-12);
    // yaw 30 degrees: the sensor's x axis points 30 degrees to the left of the link's.
    CHECK_NEAR(s.pose.r[0], std::cos(30 * urdfgen::kPi / 180), 1e-6);
    CHECK_NEAR(s.pose.r[3], std::sin(30 * urdfgen::kPi / 180), 1e-6);
  }
  dist->setValue(0.15);
  CHECK(slider->value() == 1000);
  dist->setValue(0.5);  // beyond the end: limited to the link
  CHECK_NEAR(dist->value(), 0.15, 1e-9);
  dist->setValue(0.1);
  CHECK(slider->value() == static_cast<int>(std::lround(0.1 / 0.15 * 1000)));
  slider->setValue(0);
  CHECK_NEAR(dist->value(), 0.0, 1e-9);
  dist->setValue(0.1);
  win.refreshNow();
  CHECK(win.xacroText().contains("<xacro:property name=\"sensor_camera_distance\" value=\"0.1\"/>"));
  CHECK(win.xacroText().contains("${-sensor_camera_distance+sensor_camera_offset_z}"));  // the tibia hangs down

  // The distance you asked for is remembered while the link is shorter than that.
  setValue(page, "tibia_length", 0.05);
  win.refreshNow();
  CHECK_NEAR(dist->maximum(), 0.05, 1e-9);
  CHECK_NEAR(dist->value(), 0.05, 1e-9);
  CHECK(win.xacroText().contains("<xacro:property name=\"sensor_camera_distance\" value=\"0.05\"/>"));
  setValue(page, "tibia_length", 0.15);
  win.refreshNow();
  CHECK_NEAR(dist->maximum(), 0.15, 1e-9);
  CHECK_NEAR(dist->value(), 0.1, 1e-9);

  // A different link: the root of the new one.
  link->setCurrentIndex(link->findText("leg2_femur_link"));
  CHECK_NEAR(dist->maximum(), 0.1, 1e-9);
  CHECK_NEAR(dist->value(), 0.0, 1e-9);
  // A body centred on its root slides both ways.
  link->setCurrentIndex(link->findText("base_link"));
  CHECK_NEAR(dist->minimum(), -0.025, 1e-9);
  CHECK_NEAR(dist->maximum(), 0.025, 1e-9);
  CHECK(slider->value() == 500);
  // A frame without a body has nothing to slide along.
  win.selectType(RobotType::DiffDrive);
  win.refreshNow();
  {
    QWidget* ddPage = win.currentPage();
    addSensor(ddPage, 1);
    win.refreshNow();
    auto* l = find<QComboBox>(ddPage, "sensor1_link");
    auto* d = find<QDoubleSpinBox>(ddPage, "sensor1_distance");
    auto* sl = find<QSlider>(ddPage, "sensor1_distance_slider");
    CHECK(l && d && sl);
    if (l && d && sl) {
      CHECK(l->currentText() == "base_link" && d->isEnabled() && sl->isEnabled());
      l->setCurrentIndex(l->findText("base_footprint"));
      CHECK(!d->isEnabled() && !sl->isEnabled());
      CHECK_NEAR(d->maximum(), 0.0, 1e-12);
      l->setCurrentIndex(l->findText("left_wheel_link"));  // centred on the root, along the axle
      CHECK_NEAR(d->minimum(), -0.015, 1e-9);
      CHECK_NEAR(d->maximum(), 0.015, 1e-9);
      d->setValue(0.01);
      win.refreshNow();
      const auto& s = view->scene().sensors.at(0);
      const urdfgen::SceneFrame* wheel = frameNamed(view->scene(), "left_wheel_link");
      CHECK(wheel != nullptr);
      if (wheel) {  // 1 cm along the axle (the wheel's y axis)
        CHECK_NEAR(s.pose.t.x - wheel->pose.t.x, 0.01 * wheel->pose.r[1], 1e-9);
        CHECK_NEAR(s.pose.t.y - wheel->pose.t.y, 0.01 * wheel->pose.r[4], 1e-9);
        CHECK_NEAR(s.pose.t.z - wheel->pose.t.z, 0.01 * wheel->pose.r[7], 1e-9);
        CHECK_NEAR(s.pose.t.y - wheel->pose.t.y, 0.01, 1e-9);  // and that is to the left
      }
    }
    win.currentPage()->sensorPanel()->clear();
  }

  // ---- the sensor follows the leg when the joints are posed
  win.selectType(RobotType::Hexapod);
  win.refreshNow();
  CHECK(panel->count() == 1);  // the hexapod page kept its own sensor
  link = find<QComboBox>(page, "sensor1_link");
  dist = find<QDoubleSpinBox>(page, "sensor1_distance");
  CHECK(link && link->currentText() == "base_link");
  link->setCurrentIndex(link->findText("leg1_tibia_link"));
  dist->setValue(0.1);
  yaw = find<QDoubleSpinBox>(page, "sensor1_yaw");
  yaw->setValue(0);
  win.refreshNow();
  const urdfgen::Vec3 rest = view->scene().sensors.at(0).pose.t;
  CHECK(win.jointPanel()->setJointValue("leg1_coxa_joint", 0.4));
  CHECK(win.jointPanel()->setJointValue("leg1_femur_joint", -0.3));
  {
    const auto& s = view->scene().sensors.at(0);
    CHECK(urdfgen::length(s.pose.t - rest) > 0.01);
    const urdfgen::SceneFrame* tibia = frameNamed(view->scene(), "leg1_tibia_link");
    const urdfgen::SceneFrame* own = frameNamed(view->scene(), "camera_link");
    CHECK(tibia != nullptr && own != nullptr);
    if (tibia && own) {
      CHECK_NEAR(urdfgen::length(s.root - tibia->pose.t), 0.0, 1e-12);
      CHECK_NEAR(urdfgen::length(s.pose.t - own->pose.t), 0.0, 1e-12);
      CHECK_NEAR(urdfgen::length(s.pose.t - s.root), 0.1, 1e-9);  // still 10 cm from the root
    }
  }
  win.jointPanel()->resetPose();

  // ---- a link the robot no longer has: reported, remembered, and fine again when it is back
  win.selectType(RobotType::Arm);
  win.refreshNow();
  {
    QWidget* armPage = win.currentPage();
    addSensor(armPage, 2);
    win.refreshNow();
    auto* l = find<QComboBox>(armPage, "sensor1_link");
    auto* count = find<QSpinBox>(armPage, "link_count");
    CHECK(l && count);
    if (l && count) {
      l->setCurrentIndex(l->findText("link_3"));
      win.refreshNow();
      CHECK(!win.hasErrors());
      count->setValue(2);
      win.refreshNow();
      CHECK(win.hasErrors());
      CHECK(issueMentions(win, "link_3"));
      CHECK(l->currentText() == "link_3");  // still shown, so you can see what is wrong
      CHECK(l->findText("link_2") >= 0 && l->findText("link_3") >= 0);
      l->setCurrentIndex(l->findText("link_2"));  // the fix: pick a link the robot has
      win.refreshNow();
      CHECK(!win.hasErrors());
      CHECK(l->findText("link_3") < 0);
      count->setValue(3);
      l->setCurrentIndex(l->findText("tool0"));
      win.refreshNow();
      CHECK(!win.hasErrors());
      CHECK(win.urdfText().contains("<parent link=\"tool0\"/>"));
      // The arm's own geometry feeds the range: link_1 is a 5 cm cylinder up from the base.
      l->setCurrentIndex(l->findText("link_1"));
      auto* d = find<QDoubleSpinBox>(armPage, "sensor1_distance");
      CHECK(d && d->maximum() > 0.0 && d->minimum() == 0.0);
    }
    armPage->findChild<SensorPanel*>("sensor_panel")->clear();
    win.refreshNow();
    CHECK(!win.hasErrors());
  }

  // ---- several sensors; remove one; names and numbering
  win.selectType(RobotType::Hexapod);
  win.refreshNow();
  panel->clear();
  addSensor(page, 0);
  addSensor(page, 1);
  addSensor(page, 2);
  addSensor(page, 0);
  win.refreshNow();
  CHECK(panel->count() == 4);
  {
    const auto sensors = panel->sensors();
    CHECK(sensors[0].name == "camera" && sensors[1].name == "imu" && sensors[2].name == "lidar" &&
          sensors[3].name == "camera_2");
    CHECK(sensors[0].type == urdfgen::SensorType::Camera && sensors[1].type == urdfgen::SensorType::Imu &&
          sensors[2].type == urdfgen::SensorType::Lidar);
  }
  CHECK(!win.hasErrors());
  CHECK(view->scene().sensors.size() == 4);
  CHECK(win.urdfText().contains("<link name=\"camera_2_link\">"));
  CHECK(win.urdfText().contains("<link name=\"imu_link\">"));
  if (auto* remove = find<QPushButton>(page, "sensor2_remove")) remove->click();  // the IMU
  win.refreshNow();
  CHECK(panel->count() == 3);
  CHECK(find<QLineEdit>(page, "sensor2_name")->text() == "lidar");
  CHECK(find<QLineEdit>(page, "sensor3_name")->text() == "camera_2");
  CHECK(page->findChild<QLineEdit*>("sensor4_name") == nullptr);
  CHECK(!win.urdfText().contains("imu_link"));
  CHECK(!win.xacroText().contains("sensor_imu_"));
  CHECK(view->scene().sensors.size() == 3);
  CHECK(!win.hasErrors());

  // Changing the type renames a made-up name but not one you chose.
  find<QComboBox>(page, "sensor1_type")->setCurrentIndex(2);  // camera -> LiDAR; "lidar" is taken
  CHECK(find<QLineEdit>(page, "sensor1_name")->text() == "lidar_2");
  find<QComboBox>(page, "sensor1_type")->setCurrentIndex(3);  // -> GPS
  CHECK(find<QLineEdit>(page, "sensor1_name")->text() == "gps");
  find<QLineEdit>(page, "sensor1_name")->setText("antenna");
  find<QComboBox>(page, "sensor1_type")->setCurrentIndex(4);  // -> odometry
  CHECK(find<QLineEdit>(page, "sensor1_name")->text() == "antenna");
  win.refreshNow();
  CHECK(!win.hasErrors());
  CHECK(win.urdfText().contains("<link name=\"antenna_link\">"));
  CHECK(win.urdfText().contains("<material name=\"sensor_odometry\">"));
  CHECK(view->scene().sensors.at(0).type == urdfgen::SensorType::Odometry);

  // Bad and repeated names are errors that say so, and the output keeps its last valid state.
  const QString goodUrdf = win.urdfText();
  find<QLineEdit>(page, "sensor1_name")->setText("bad name");
  win.refreshNow();
  CHECK(win.hasErrors() && !win.canSave());
  CHECK(issueMentions(win, "bad name"));
  CHECK(win.urdfText() == goodUrdf);
  find<QLineEdit>(page, "sensor1_name")->setText("lidar");  // same as sensor 2
  win.refreshNow();
  CHECK(win.hasErrors());
  CHECK(issueMentions(win, "more than one sensor named 'lidar'"));
  find<QLineEdit>(page, "sensor1_name")->setText("");
  win.refreshNow();
  CHECK(win.hasErrors());
  find<QLineEdit>(page, "sensor1_name")->setText("antenna");
  win.refreshNow();
  CHECK(!win.hasErrors() && win.canSave());

  // ---- each page keeps its own sensors
  win.selectType(RobotType::Biped);
  win.refreshNow();
  CHECK(win.currentPage()->sensorPanel()->count() == 0);
  CHECK(view->scene().sensors.empty());
  CHECK(!win.urdfText().contains("antenna"));
  win.selectType(RobotType::Hexapod);
  win.refreshNow();
  CHECK(panel->count() == 3);
  CHECK(view->scene().sensors.size() == 3);
  CHECK(win.urdfText().contains("antenna_link"));

  // ---- sensors on a mecanum wheel hub; the rollers are not offered
  win.selectType(RobotType::FourWheel);
  win.refreshNow();
  {
    QWidget* fwPage = win.currentPage();
    find<QComboBox>(fwPage, "front_left_wheel_type")->setCurrentIndex(1);
    addSensor(fwPage, 4);
    win.refreshNow();
    auto* l = find<QComboBox>(fwPage, "sensor1_link");
    CHECK(l != nullptr);
    if (l) {
      CHECK(l->findText("front_left_wheel_link") >= 0);
      for (int i = 0; i < l->count(); ++i) CHECK(!l->itemText(i).contains("roller"));
      l->setCurrentIndex(l->findText("front_left_wheel_link"));
      win.refreshNow();
      CHECK(!win.hasErrors());
      CHECK(view->scene().sensors.size() == 1 && view->scene().sensors[0].parent == "front_left_wheel_link");
      find<QComboBox>(fwPage, "front_left_wheel_type")->setCurrentIndex(0);  // back to a plain wheel
      win.refreshNow();
      CHECK(!win.hasErrors());
      CHECK(l->currentText() == "front_left_wheel_link");
    }
    win.currentPage()->sensorPanel()->clear();
    find<QComboBox>(fwPage, "front_left_wheel_type")->setCurrentIndex(0);
  }

  // ---- the picture: the sensor axes are drawn, and can be switched off
  win.selectType(RobotType::Hexapod);
  win.refreshNow();
  CHECK(win.sensorAxesBox() != nullptr && win.sensorAxesBox()->isChecked() && view->showSensorAxes());
  const QSize size(480, 320);
  view->setPreset(RobotView::Preset::Isometric);
  view->renderImage(size);  // warm-up: the first render in a process differs by a few pixels around the letters
  const QImage withAxes = view->renderImage(size);
  win.sensorAxesBox()->setChecked(false);
  CHECK(!view->showSensorAxes());
  const QImage withoutAxes = view->renderImage(size);
  CHECK(differingPixels(withAxes, withoutAxes) > 400);
  win.sensorAxesBox()->setChecked(true);
  CHECK(differingPixels(withAxes, view->renderImage(size)) == 0);
  // The link frames are thin lines and do not include the sensor's own frame.
  {
    RobotView probe;
    probe.resize(size);
    probe.setScene(urdfgen::buildScene(urdfgen::buildDefault(RobotType::Hexapod)), true);
    probe.setShowSensorAxes(false);
    probe.setShowFrames(true);
    urdfgen::Robot bot = urdfgen::buildDefault(RobotType::Hexapod);
    urdfgen::addSensors(bot, {urdfgen::defaultSensor(bot, urdfgen::SensorType::Imu, {})});
    urdfgen::Scene sc = urdfgen::buildScene(bot);
    probe.setScene(sc, false);
    const QImage frameImage = probe.renderImage(size);
    sc.sensors.clear();  // the same scene, but now the sensor link is an ordinary link
    probe.setScene(sc, false);
    CHECK(differingPixels(frameImage, probe.renderImage(size)) > 20);
  }
  // Without sensors the switch changes nothing.
  win.currentPage()->sensorPanel()->clear();
  win.refreshNow();
  const QImage bare = view->renderImage(size);
  win.sensorAxesBox()->setChecked(false);
  CHECK(differingPixels(bare, view->renderImage(size)) == 0);
  win.sensorAxesBox()->setChecked(true);
  CHECK(win.urdfText() == plainUrdf);  // back to exactly the sensor-free robot
  CHECK(find<QLabel>(page, "no_sensors")->isVisible());

  // ---- every type: add one of each sensor, valid output
  for (RobotType t : urdfgen::allRobotTypes()) {
    win.selectType(t);
    win.refreshNow();
    QWidget* p = win.currentPage();
    for (int i = 0; i < 5; ++i) addSensor(p, i);
    win.refreshNow();
    CHECK(!win.hasErrors());
    CHECK(win.currentPage()->sensorPanel()->count() == 5);
    CHECK(view->scene().sensors.size() == 5);
    CHECK(win.urdfText().contains("<link name=\"odometry_link\">"));
    CHECK(wellFormed(win.xacroText()) && wellFormed(win.urdfText()));
    win.currentPage()->sensorPanel()->clear();
    win.refreshNow();
    CHECK(view->scene().sensors.empty());
  }
}

void testErrorsAndSaving(MainWindow& win) {
  win.selectType(RobotType::Biped);
  win.refreshNow();
  CHECK(win.canSave());
  const QString good = win.urdfText();

  // An empty robot name is an error; the last valid output stays visible.
  auto* name = find<QLineEdit>(win.currentPage(), "robot_name");
  CHECK(name != nullptr);
  if (name) {
    name->setText("");
    win.refreshNow();
    CHECK(win.hasErrors());
    CHECK(!win.canSave());
    CHECK(win.issueCount() >= 1);
    CHECK(win.urdfText() == good);
    QString err;
    CHECK(!win.saveBothTo(QDir::tempPath(), &err));
    CHECK(!err.isEmpty());

    name->setText("my_biped");
    win.refreshNow();
    CHECK(!win.hasErrors());
    CHECK(win.canSave());
    CHECK(win.issueCount() == 0);
    CHECK(win.urdfText().contains("<robot name=\"my_biped\">"));
  }

  // A joint with lower == upper is an error that points at the joint.
  setValue(win.currentPage(), "ankle_limit", 1.0);  // fine (1 degree)
  win.refreshNow();
  CHECK(!win.hasErrors());

  // Saving writes both files with exactly the previewed text.
  QTemporaryDir dir;
  CHECK(dir.isValid());
  QString err;
  CHECK(win.saveBothTo(dir.path(), &err));
  QFile x(QDir(dir.path()).filePath("my_biped.xacro"));
  QFile u(QDir(dir.path()).filePath("my_biped.urdf"));
  CHECK(x.open(QIODevice::ReadOnly) && QString::fromUtf8(x.readAll()) == win.xacroText());
  CHECK(u.open(QIODevice::ReadOnly) && QString::fromUtf8(u.readAll()) == win.urdfText());

  // Saving into a folder that does not exist reports an error instead of crashing.
  QString err2;
  CHECK(!win.saveBothTo(QDir(dir.path()).filePath("no/such/folder"), &err2));
  CHECK(!err2.isEmpty());

  // Switching type clears the old robot's output.
  win.selectType(RobotType::Quadruped);
  CHECK(win.urdfText().contains("<robot name=\"quadruped\">"));
  CHECK(!win.urdfText().contains("my_biped"));
}

}  // namespace

int main(int argc, char** argv) {
  if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM")) qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(argc, argv);
  QApplication::setOrganizationName("urdf_generator_test");
  QApplication::setApplicationName("urdf_generator_test");

  MainWindow win;
  win.show();
  testDefaultsMatchCore(win);
  testHexapodEdit(win);
  testArm(win);
  testWheeled(win);
  testView(win);
  testJointSliders(win);
  testMecanumChoices(win);
  testSensors(win);
  testErrorsAndSaving(win);

  std::printf("%d checks, %d failed\n", g_checks, g_failed);
  return g_failed == 0 ? 0 : 1;
}
