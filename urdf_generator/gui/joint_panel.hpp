// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// One slider and spin box per movable joint, for posing the robot in the 3D view.
// The pose is only for looking at the robot: the generated files do not depend on it.
#pragma once

#include <QString>
#include <QStringList>
#include <QWidget>
#include <vector>

#include "urdf_generator/scene.hpp"

class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QScrollArea;
class QSlider;

namespace urdfgui {

class JointPanel : public QWidget {
  Q_OBJECT

 public:
  explicit JointPanel(QWidget* parent = nullptr);

  /// Shows one row per joint. A joint that was already there (same name and type) keeps its
  /// value, limited to its possibly new range; all others start at zero (or the nearest limit).
  /// Does not emit poseChanged: the caller asks for pose() afterwards.
  void setJoints(const std::vector<urdfgen::MovableJoint>& joints);

  /// Removes every row and forgets all values (a different robot).
  void clear() { setJoints({}); pose_.clear(); }

  /// Current values: radians for revolute and continuous joints, metres for prismatic ones.
  const urdfgen::JointPose& pose() const { return pose_; }
  int jointCount() const { return static_cast<int>(rows_.size()); }
  QStringList jointNames() const;

  /// Moves one joint (radians or metres, limited to its range) and emits poseChanged. Returns
  /// false if there is no such joint.
  bool setJointValue(const QString& joint, double value);
  double jointValue(const QString& joint) const;

  /// Every joint back to its rest value. Emits poseChanged if anything moved.
  void resetPose();

 signals:
  void poseChanged();

 private:
  struct Row {
    urdfgen::MovableJoint joint;
    QSlider* slider = nullptr;
    QDoubleSpinBox* spin = nullptr;
  };

  void rebuild();
  void apply(std::size_t index, double value, bool fromSlider, bool fromSpin);
  double displayFactor(const urdfgen::MovableJoint& j) const;
  int tickOf(const urdfgen::MovableJoint& j, double value) const;
  double valueOfTick(const urdfgen::MovableJoint& j, int tick) const;
  double restValue(const urdfgen::MovableJoint& j) const;

  QLabel* empty_ = nullptr;
  QPushButton* reset_ = nullptr;
  QScrollArea* scroll_ = nullptr;
  std::vector<Row> rows_;
  urdfgen::JointPose pose_;
};

}  // namespace urdfgui
