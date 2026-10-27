// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// One configuration page per robot type. A page owns its input widgets,
// emits changed() whenever any of them is edited, and turns the current
// values into a urdfgen::Robot with build().
#pragma once

#include <QString>
#include <QWidget>

#include "urdf_generator/builders.hpp"

namespace urdfgui {

class SensorPanel;

class ConfigPage : public QWidget {
  Q_OBJECT

 public:
  explicit ConfigPage(QWidget* parent = nullptr) : QWidget(parent) {}

  virtual urdfgen::RobotType type() const = 0;
  /// Builds the robot from the current values, sensors included. Never throws for any widget
  /// state; problems with the values are reported by urdfgen::validate().
  virtual urdfgen::Robot build() const = 0;
  /// One-line description shown above the form.
  virtual QString hint() const = 0;
  /// The sensors of this page (every robot type has its own list).
  SensorPanel* sensorPanel() const { return sensors_; }

 signals:
  void changed();

 protected:
  SensorPanel* sensors_ = nullptr;
};

/// Creates the page for the given robot type, filled with default values.
ConfigPage* createPage(urdfgen::RobotType type, QWidget* parent = nullptr);

}  // namespace urdfgui
