// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// The "Run" tab: build the generated package with colcon, launch it with `ros2 launch`, show what
// it prints, and check every bridged topic (pass / fail per row).
//
// Everything that has to be decided about the output of the ROS tools lives in
// gz_bridge_helper/verify.hpp (and is unit tested there); this class only starts the processes
// and puts the results in the table.
#pragma once

#include <QString>
#include <QStringList>
#include <QWidget>
#include <functional>
#include <map>
#include <vector>

#include "gz_bridge_helper/verify.hpp"

class QCheckBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QSpinBox;
class QTableWidget;
class QTimer;

namespace gzgui {

class ProcessJob;

class RunPanel : public QWidget {
  Q_OBJECT

 public:
  /// Writes the package into `srcDir` (`<srcDir>/<package>/...`); false and `error` on failure.
  using PackageWriter = std::function<bool(const QString& srcDir, QString* error)>;

  explicit RunPanel(QWidget* parent = nullptr);
  ~RunPanel() override;

  /// Called by the main window whenever the plan changes. `ready` is false while there are errors.
  void setPackage(const QString& packageName, const std::vector<gzb::TopicCheck>& checks, bool ready, PackageWriter writer);

  QString workspace() const;
  void setWorkspace(const QString& path);
  /// Ask before replacing an existing `<workspace>/src/<package>` (on by default).
  void setConfirmOverwrite(bool on) { confirmOverwrite_ = on; }

  bool building() const { return build_ != nullptr; }
  bool launched() const { return launch_ != nullptr; }
  bool checking() const { return checking_; }
  /// The rows of the table (the plan as it was when the current run started).
  const std::vector<gzb::TopicCheck>& checks() const { return checks_; }
  const std::vector<gzb::CheckResult>& results() const { return results_; }
  /// The terminal commands that do the same as the buttons, for the label under the buttons.
  QString equivalentCommands() const;

  /// Stops the launch and the checks, waiting (at most `maxMs`) for Gazebo to shut down.
  void shutdown(int maxMs = 12000);

  // Controls (also used by the tests).
  QPushButton* buildButton() const { return buildButton_; }
  QPushButton* launchButton() const { return launchButton_; }
  QPushButton* stopButton() const { return stopButton_; }
  QPushButton* checkButton() const { return checkButton_; }
  QCheckBox* autoCheckBox() const { return autoCheck_; }
  QCheckBox* guiBox() const { return guiBox_; }
  QSpinBox* settleSpin() const { return settle_; }
  QSpinBox* sampleSpin() const { return sample_; }
  QTableWidget* table() const { return table_; }
  QPlainTextEdit* log() const { return log_; }
  QLabel* summaryLabel() const { return summary_; }

 public slots:
  void build();
  void launch();
  void stop();
  void startChecks();
  void cancelChecks();

 signals:
  void statusMessage(const QString& text);
  void stateChanged();    // buttons or results changed
  void checksFinished();  // every row has a result

 private:
  struct CheckRun;

  void buildUi();
  void fillTable();
  void updateButtons();
  void updateSummary();
  void setResult(size_t index, const gzb::CheckResult& r);
  void appendLog(const QString& text);
  void logLine(const QString& tag, const QString& text);
  QStringList setupScripts() const;
  void onBuildFinished(const QString& pkg, bool ok, const QString& why);
  void onLaunchFinished(const QString& why);
  void tick();

  void onTopicList(const QString& output, bool missing, bool failed);
  void startNextChecks();
  void runCheck(size_t index);
  void runPublished(size_t index);
  void runCommanded(size_t index);
  void finishCheckRun(CheckRun* run, const gzb::CheckResult& r);
  void finishChecks();

  bool frozen() const { return build_ || launch_ || checking_; }
  void syncTable();

  QString package_;                          // the newest plan
  std::vector<gzb::TopicCheck> latest_;
  std::vector<gzb::TopicCheck> checks_;      // what the table shows; frozen while something runs
  std::vector<gzb::CheckResult> results_;
  bool ready_ = false;
  bool confirmOverwrite_ = true;
  PackageWriter writer_;

  QLineEdit* workspaceEdit_ = nullptr;
  QPushButton *browseButton_ = nullptr, *buildButton_ = nullptr, *launchButton_ = nullptr, *stopButton_ = nullptr,
              *checkButton_ = nullptr, *clearButton_ = nullptr;
  QCheckBox *guiBox_ = nullptr, *autoCheck_ = nullptr;
  QSpinBox *settle_ = nullptr, *sample_ = nullptr;
  QLabel *commands_ = nullptr, *state_ = nullptr, *summary_ = nullptr, *planChanged_ = nullptr;
  QTableWidget* table_ = nullptr;
  QPlainTextEdit* log_ = nullptr;

  ProcessJob* build_ = nullptr;
  ProcessJob* launch_ = nullptr;
  QTimer* tick_ = nullptr;
  int countdown_ = -1;               // seconds until the automatic check; -1: none pending
  QString launchedPackage_;
  std::vector<gzb::TopicCheck> launchedChecks_;

  bool checking_ = false;
  std::map<std::string, std::vector<std::string>> topics_;
  std::vector<size_t> queue_;
  std::vector<CheckRun*> active_;
  QStringList writtenPackages_;      // packages this window already wrote into the workspace
  ProcessJob* listJob_ = nullptr;
};

}  // namespace gzgui
