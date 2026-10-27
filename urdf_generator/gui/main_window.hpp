// Copyright 2026. Licensed under the Apache License, Version 2.0.
#pragma once

#include <QMainWindow>
#include <QString>
#include <QTimer>
#include <optional>

#include "config_pages.hpp"
#include "joint_panel.hpp"
#include "robot_view.hpp"
#include "urdf_generator/model.hpp"

class QAction;
class QCheckBox;
class QComboBox;
class QLabel;
class QListWidget;
class QPlainTextEdit;
class QPushButton;
class QStackedWidget;
class QTabWidget;
class QTreeWidget;

namespace urdfgui {

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(QWidget* parent = nullptr);

  // ---- used by the tests (and handy for scripting) ----
  void selectType(urdfgen::RobotType type);
  ConfigPage* currentPage() const;
  RobotView* view() const { return view_; }
  QCheckBox* framesBox() const { return framesBox_; }
  QCheckBox* sensorAxesBox() const { return sensorAxesBox_; }
  JointPanel* jointPanel() const { return joints_; }
  QCheckBox* jointsBox() const { return jointsBox_; }
  /// Regenerates the preview immediately instead of waiting for the short debounce.
  void refreshNow();
  const QString& xacroText() const { return xacro_; }
  const QString& urdfText() const { return urdf_; }
  bool hasErrors() const { return hasErrors_; }
  bool canSave() const { return !hasErrors_ && !urdf_.isEmpty(); }
  int issueCount() const;
  /// Writes <name>.xacro and <name>.urdf into dir. Returns false and fills `error` on failure.
  bool saveBothTo(const QString& dir, QString* error = nullptr);

 private slots:
  void onTypeChanged(int index);
  void scheduleRefresh();
  void saveXacro();
  void saveUrdf();
  void saveBoth();
  void copyCurrent();
  void onPoseChanged();

 private:
  void buildUi();
  void fillTree(const urdfgen::Robot& robot);
  void showIssues(const std::vector<urdfgen::Issue>& issues);
  void setPreview(QPlainTextEdit* view, const QString& text);
  void updateButtons();
  QString defaultPath(const QString& extension) const;
  void rememberDir(const QString& path);
  bool writeText(const QString& path, const QString& text, QString* error);
  void saveSingle(const QString& title, const QString& filter, const QString& extension,
                  const QString& text);

  QComboBox* typeBox_ = nullptr;
  QLabel* hint_ = nullptr;
  QStackedWidget* stack_ = nullptr;
  std::vector<ConfigPage*> pages_;

  RobotView* view_ = nullptr;
  QCheckBox* framesBox_ = nullptr;
  QCheckBox* sensorAxesBox_ = nullptr;
  bool fitNext_ = true;  // re-aim the 3D camera at the next valid scene
  JointPanel* joints_ = nullptr;
  QCheckBox* jointsBox_ = nullptr;
  std::optional<urdfgen::Robot> lastRobot_;  // the robot the 3D view shows, for re-posing it

  QTabWidget* tabs_ = nullptr;
  QPlainTextEdit* xacroView_ = nullptr;
  QPlainTextEdit* urdfView_ = nullptr;
  QTreeWidget* tree_ = nullptr;
  QLabel* stale_ = nullptr;
  QLabel* summary_ = nullptr;
  QListWidget* issues_ = nullptr;
  QPushButton *copyButton_ = nullptr, *saveXacroButton_ = nullptr, *saveUrdfButton_ = nullptr,
              *saveBothButton_ = nullptr;
  QAction *saveXacroAct_ = nullptr, *saveUrdfAct_ = nullptr, *saveBothAct_ = nullptr,
          *copyAct_ = nullptr;

  QTimer timer_;
  QString xacro_, urdf_, robotName_;
  bool hasErrors_ = false;
};

}  // namespace urdfgui
