// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// The "Sensors" box of a configuration page: add a camera, IMU, LiDAR, GPS or odometry frame,
// choose the link it is mounted on, slide it along that link and orient it.
#pragma once

#include <QGroupBox>
#include <QString>
#include <memory>
#include <vector>

#include "urdf_generator/sensors.hpp"

class QComboBox;
class QLabel;
class QPushButton;
class QVBoxLayout;

namespace urdfgui {

class SensorPanel : public QGroupBox {
  Q_OBJECT

 public:
  explicit SensorPanel(QWidget* parent = nullptr);
  ~SensorPanel() override;

  /// Brings the panel in line with the robot the sensors are mounted on (the robot without
  /// any sensors): the list of links, and the range each sensor can slide along its link.
  /// A sensor keeps the position it was given while its link is shorter than that.
  /// Does not emit changed().
  void setRobot(const urdfgen::Robot& base);

  std::vector<urdfgen::SensorConfig> sensors() const;
  int count() const { return static_cast<int>(rows_.size()); }

  /// Adds a sensor at the end of the list (and emits changed()).
  void append(const urdfgen::SensorConfig& sensor);
  /// Removes a sensor (and emits changed()); the ones after it move up and are renamed
  /// sensor<N>_* accordingly.
  void removeAt(int index);
  void clear();

 signals:
  void changed();
  /// The "Add sensor" button: the page decides where the new sensor goes (see defaultSensor()).
  void addRequested(urdfgen::SensorType type);

 private:
  struct Row;

  Row* newRow(const urdfgen::SensorConfig& sensor);
  Row* rowOf(const QObject* box) const;
  int indexOf(const Row* row) const;
  void renumber();
  void refreshRow(Row& row);
  void applyRange(Row& row);
  void updateTitle(Row& row);
  void updateEmptyLabel();
  void showRow(Row& row);

  QComboBox* addType_ = nullptr;
  QPushButton* addButton_ = nullptr;
  QLabel* empty_ = nullptr;
  QVBoxLayout* rowsLayout_ = nullptr;
  std::vector<std::unique_ptr<Row>> rows_;
  std::vector<std::string> links_;
  std::vector<std::pair<std::string, urdfgen::LinkAxis>> axes_;
};

}  // namespace urdfgui
