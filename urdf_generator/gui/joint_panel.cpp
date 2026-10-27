// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include "joint_panel.hpp"

#include <QDoubleSpinBox>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPalette>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSlider>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>

namespace urdfgui {
namespace {

constexpr int kTicks = 1000;  // slider resolution across a joint's range

QString typeName(urdfgen::JointType t) {
  switch (t) {
    case urdfgen::JointType::Fixed: return "fixed";
    case urdfgen::JointType::Revolute: return "revolute";
    case urdfgen::JointType::Continuous: return "continuous";
    case urdfgen::JointType::Prismatic: return "prismatic";
  }
  return QString();
}

}  // namespace

JointPanel::JointPanel(QWidget* parent) : QWidget(parent) {
  setObjectName("joint_panel");
  auto* v = new QVBoxLayout(this);
  v->setContentsMargins(0, 0, 0, 0);

  auto* header = new QHBoxLayout;
  header->addWidget(new QLabel("<b>Joint pose</b>"));
  header->addStretch(1);
  reset_ = new QPushButton("Reset pose");
  reset_->setObjectName("reset_pose");
  reset_->setAutoDefault(false);
  reset_->setToolTip("Put every joint back at zero");
  connect(reset_, &QPushButton::clicked, this, &JointPanel::resetPose);
  header->addWidget(reset_);
  v->addLayout(header);

  auto* note = new QLabel("For looking only: the generated files are not affected.");
  note->setWordWrap(true);
  QPalette notePalette = note->palette();
  notePalette.setColor(QPalette::WindowText, notePalette.color(QPalette::PlaceholderText));
  note->setPalette(notePalette);
  v->addWidget(note);

  scroll_ = new QScrollArea;
  scroll_->setObjectName("joint_scroll");
  scroll_->setFrameShape(QFrame::NoFrame);
  scroll_->setWidgetResizable(true);
  scroll_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  v->addWidget(scroll_, 1);

  empty_ = new QLabel("This robot has no movable joints.");
  empty_->setObjectName("no_joints");
  empty_->setAlignment(Qt::AlignCenter);
  empty_->setWordWrap(true);
  v->addWidget(empty_);

  setMinimumWidth(230);
  rebuild();
}

double JointPanel::displayFactor(const urdfgen::MovableJoint& j) const {
  return j.type == urdfgen::JointType::Prismatic ? 1.0 : 180.0 / urdfgen::kPi;
}

double JointPanel::restValue(const urdfgen::MovableJoint& j) const {
  return urdfgen::clampJointValue(j, 0.0);
}

int JointPanel::tickOf(const urdfgen::MovableJoint& j, double value) const {
  const double span = j.upper - j.lower;
  if (span <= 0) return 0;
  return static_cast<int>(std::lround((value - j.lower) / span * kTicks));
}

double JointPanel::valueOfTick(const urdfgen::MovableJoint& j, int tick) const {
  return j.lower + (j.upper - j.lower) * tick / static_cast<double>(kTicks);
}

QStringList JointPanel::jointNames() const {
  QStringList names;
  for (const Row& r : rows_) names << QString::fromStdString(r.joint.name);
  return names;
}

void JointPanel::setJoints(const std::vector<urdfgen::MovableJoint>& joints) {
  // Keep the value of every joint that is still there; everything else starts at rest.
  urdfgen::JointPose next;
  for (const urdfgen::MovableJoint& j : joints) {
    double value = restValue(j);
    for (const Row& old : rows_) {
      if (old.joint.name == j.name && old.joint.type == j.type) {
        const auto it = pose_.find(j.name);
        if (it != pose_.end()) value = urdfgen::clampJointValue(j, it->second);
        break;
      }
    }
    next[j.name] = value;
  }

  // Rebuilding while nothing changed would only reset the scroll position and focus.
  bool same = joints.size() == rows_.size();
  for (std::size_t i = 0; same && i < joints.size(); ++i) {
    const urdfgen::MovableJoint& a = joints[i];
    const urdfgen::MovableJoint& b = rows_[i].joint;
    same = a.name == b.name && a.type == b.type && a.chain == b.chain &&
           std::fabs(a.lower - b.lower) < 1e-12 && std::fabs(a.upper - b.upper) < 1e-12;
  }

  pose_ = next;
  if (same) {
    for (Row& r : rows_) {  // the values may still have been clamped
      const double v = pose_[r.joint.name];
      const QSignalBlocker b1(r.slider), b2(r.spin);
      r.slider->setValue(tickOf(r.joint, v));
      r.spin->setValue(v * displayFactor(r.joint));
    }
    return;
  }

  rows_.clear();
  for (const urdfgen::MovableJoint& j : joints) {
    Row r;
    r.joint = j;
    rows_.push_back(r);
  }
  rebuild();
}

void JointPanel::rebuild() {
  auto* content = new QWidget;
  auto* column = new QVBoxLayout(content);
  column->setContentsMargins(0, 0, 6, 0);
  column->setSpacing(2);

  int lastChain = -1;
  for (std::size_t i = 0; i < rows_.size(); ++i) {
    Row& r = rows_[i];
    const urdfgen::MovableJoint& j = r.joint;
    if (i > 0 && j.chain != lastChain) {  // a new leg / wheel / arm starts here
      auto* line = new QFrame;
      line->setFrameShape(QFrame::HLine);
      line->setFrameShadow(QFrame::Sunken);
      column->addSpacing(4);
      column->addWidget(line);
      column->addSpacing(4);
    }
    lastChain = j.chain;

    const QString name = QString::fromStdString(j.name);
    const bool prismatic = j.type == urdfgen::JointType::Prismatic;
    const double k = displayFactor(j);

    auto* row = new QWidget;
    auto* grid = new QGridLayout(row);
    grid->setContentsMargins(0, 0, 0, 0);
    grid->setVerticalSpacing(0);

    auto* label = new QLabel(name);
    label->setObjectName("joint_" + name + "_label");
    QString tip = QString("%1 joint, axis %2 %3 %4").arg(typeName(j.type)).arg(j.axis[0], 0, 'g', 3)
                      .arg(j.axis[1], 0, 'g', 3).arg(j.axis[2], 0, 'g', 3);
    if (j.type == urdfgen::JointType::Continuous) {
      tip += ", turns without limit (wheels are plain cylinders: switch on \"Show link frames\" to see one spin)";
    } else if (prismatic) {
      tip += QString(", range %1 to %2 m").arg(j.lower, 0, 'f', 3).arg(j.upper, 0, 'f', 3);
    } else {
      tip += QString(", range %1 to %2 degrees").arg(j.lower * k, 0, 'f', 1).arg(j.upper * k, 0, 'f', 1);
    }
    label->setToolTip(tip);

    r.spin = new QDoubleSpinBox;
    r.spin->setObjectName("joint_" + name + "_spin");
    r.spin->setDecimals(prismatic ? 3 : 1);
    r.spin->setRange(j.lower * k, j.upper * k);
    r.spin->setSingleStep(prismatic ? 0.005 : 1.0);
    r.spin->setSuffix(prismatic ? " m" : QString::fromUtf8("°"));
    r.spin->setKeyboardTracking(false);
    r.spin->setToolTip(tip);
    r.spin->setValue(pose_[j.name] * k);

    r.slider = new QSlider(Qt::Horizontal);
    r.slider->setObjectName("joint_" + name + "_slider");
    r.slider->setRange(0, kTicks);
    r.slider->setSingleStep(5);
    r.slider->setPageStep(50);
    r.slider->setValue(tickOf(j, pose_[j.name]));
    r.slider->setToolTip(tip);

    grid->addWidget(label, 0, 0);
    grid->addWidget(r.spin, 0, 1);
    grid->addWidget(r.slider, 1, 0, 1, 2);
    grid->setColumnStretch(0, 1);
    column->addWidget(row);

    connect(r.slider, &QSlider::valueChanged, this,
            [this, i](int tick) { apply(i, valueOfTick(rows_[i].joint, tick), true, false); });
    connect(r.spin, &QDoubleSpinBox::valueChanged, this,
            [this, i](double shown) { apply(i, shown / displayFactor(rows_[i].joint), false, true); });
  }
  column->addStretch(1);
  scroll_->setWidget(content);  // also deletes the previous rows
  scroll_->setVisible(!rows_.empty());
  empty_->setVisible(rows_.empty());
  reset_->setEnabled(!rows_.empty());
}

void JointPanel::apply(std::size_t index, double value, bool fromSlider, bool fromSpin) {
  if (index >= rows_.size()) return;
  Row& r = rows_[index];
  value = urdfgen::clampJointValue(r.joint, value);
  const auto it = pose_.find(r.joint.name);
  if (it != pose_.end() && std::fabs(it->second - value) < 1e-12) return;
  pose_[r.joint.name] = value;
  if (!fromSlider) {
    const QSignalBlocker b(r.slider);
    r.slider->setValue(tickOf(r.joint, value));
  }
  if (!fromSpin) {
    const QSignalBlocker b(r.spin);
    r.spin->setValue(value * displayFactor(r.joint));
  }
  emit poseChanged();
}

bool JointPanel::setJointValue(const QString& joint, double value) {
  for (std::size_t i = 0; i < rows_.size(); ++i) {
    if (QString::fromStdString(rows_[i].joint.name) == joint) {
      apply(i, value, false, false);
      return true;
    }
  }
  return false;
}

double JointPanel::jointValue(const QString& joint) const {
  const auto it = pose_.find(joint.toStdString());
  return it == pose_.end() ? 0.0 : it->second;
}

void JointPanel::resetPose() {
  bool moved = false;
  for (Row& r : rows_) {
    const double rest = restValue(r.joint);
    if (std::fabs(pose_[r.joint.name] - rest) < 1e-12) continue;
    pose_[r.joint.name] = rest;
    moved = true;
    const QSignalBlocker b1(r.slider), b2(r.spin);
    r.slider->setValue(tickOf(r.joint, rest));
    r.spin->setValue(rest * displayFactor(r.joint));
  }
  if (moved) emit poseChanged();
}

}  // namespace urdfgui
