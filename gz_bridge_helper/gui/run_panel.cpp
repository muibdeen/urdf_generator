// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include "run_panel.hpp"

#include <algorithm>

#include <QCheckBox>
#include <QDir>
#include <QEventLoop>
#include <QFileDialog>
#include <QFileInfo>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPalette>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollBar>
#include <QSettings>
#include <QSpinBox>
#include <QSplitter>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>

#include "highlighter.hpp"
#include "process_job.hpp"

namespace gzgui {
namespace {

constexpr int kParallelChecks = 4;

QStringList toQ(const std::vector<std::string>& v) {
  QStringList out;
  for (const std::string& s : v) out << QString::fromStdString(s);
  return out;
}

bool sameChecks(const std::vector<gzb::TopicCheck>& a, const std::vector<gzb::TopicCheck>& b) {
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i) {
    if (a[i].rosTopic != b[i].rosTopic || a[i].rosType != b[i].rosType || a[i].gzTopic != b[i].gzTopic ||
        a[i].gzType != b[i].gzType || a[i].direction != b[i].direction || a[i].payload != b[i].payload) {
      return false;
    }
  }
  return true;
}

QString directionText(gzb::Direction d) {
  switch (d) {
    case gzb::Direction::GzToRos: return "Gazebo → ROS";
    case gzb::Direction::RosToGz: return "ROS → Gazebo";
    case gzb::Direction::Bidirectional: return "both ways";
  }
  return QString();
}

QString lastLine(const QString& text) {
  const QStringList lines = text.split('\n', Qt::SkipEmptyParts);
  return lines.isEmpty() ? QString() : lines.last().trimmed();
}

QColor stateColor(gzb::CheckState s) {
  const bool dark = QGuiApplication::palette().color(QPalette::Base).lightness() < 128;
  switch (s) {
    case gzb::CheckState::Pass: return QColor(dark ? "#85e89d" : "#1a7f37");
    case gzb::CheckState::Fail: return QColor(dark ? "#ff7b72" : "#b42318");
    case gzb::CheckState::Skipped: return QColor(dark ? "#e3b341" : "#8a5a00");
    case gzb::CheckState::Running: return QColor(dark ? "#79b8ff" : "#0b5394");
    case gzb::CheckState::Pending: break;
  }
  return QGuiApplication::palette().color(QPalette::PlaceholderText);
}

QString stateText(gzb::CheckState s) {
  switch (s) {
    case gzb::CheckState::Pass: return "Pass";
    case gzb::CheckState::Fail: return "Fail";
    case gzb::CheckState::Skipped: return "Skipped";
    case gzb::CheckState::Running: return "Checking...";
    case gzb::CheckState::Pending: break;
  }
  return "—";
}

}  // namespace

/// The processes of one row of the check.
struct RunPanel::CheckRun {
  size_t index = 0;
  ProcessJob *info = nullptr, *hz = nullptr, *gz = nullptr, *pub = nullptr;
  QTimer* delay = nullptr;
  QString infoText;
  bool done = false;
  bool pubDone = false;
  ProcessJob::Result pubResult;
};

RunPanel::RunPanel(QWidget* parent) : QWidget(parent) {
  buildUi();
  tick_ = new QTimer(this);
  tick_->setInterval(1000);
  connect(tick_, &QTimer::timeout, this, &RunPanel::tick);
  updateButtons();
}

RunPanel::~RunPanel() = default;  // the jobs are children: each stops its own process group

void RunPanel::buildUi() {
  auto* v = new QVBoxLayout(this);
  v->setContentsMargins(4, 4, 4, 4);

  auto* wsRow = new QHBoxLayout;
  wsRow->addWidget(new QLabel("Workspace:"));
  workspaceEdit_ = new QLineEdit;
  workspaceEdit_->setObjectName("workspace");
  workspaceEdit_->setPlaceholderText("colcon workspace, e.g. ~/ros2_ws (empty: the package is already built and sourced)");
  workspaceEdit_->setToolTip(
      "Build writes the package to <workspace>/src/<package> and runs colcon build there.\n"
      "Launch sources <workspace>/install/setup.bash first.");
  QSettings settings("gz_bridge_helper", "gz_bridge_helper");
  workspaceEdit_->setText(settings.value("workspace").toString());
  connect(workspaceEdit_, &QLineEdit::textChanged, this, [this](const QString& text) {
    QSettings s("gz_bridge_helper", "gz_bridge_helper");
    s.setValue("workspace", text);
    commands_->setText(equivalentCommands());
    updateButtons();
  });
  browseButton_ = new QPushButton("Browse...");
  browseButton_->setAutoDefault(false);
  connect(browseButton_, &QPushButton::clicked, this, [this]() {
    const QString dir = QFileDialog::getExistingDirectory(this, "Colcon workspace", workspace().isEmpty() ? QDir::homePath() : workspace());
    if (!dir.isEmpty()) setWorkspace(dir);
  });
  wsRow->addWidget(workspaceEdit_, 1);
  wsRow->addWidget(browseButton_);
  v->addLayout(wsRow);

  auto* row = new QHBoxLayout;
  buildButton_ = new QPushButton("Build");
  buildButton_->setObjectName("build");
  buildButton_->setToolTip("Write the package into <workspace>/src and run: colcon build --packages-select <package>");
  launchButton_ = new QPushButton("Launch");
  launchButton_->setObjectName("launch");
  launchButton_->setToolTip("ros2 launch <package> sim.launch.py");
  stopButton_ = new QPushButton("Stop");
  stopButton_->setObjectName("stop");
  checkButton_ = new QPushButton("Check topics");
  checkButton_->setObjectName("check");
  checkButton_->setToolTip("For every bridged topic: is it in `ros2 topic list` with the right type, and do messages flow?");
  for (QPushButton* b : {buildButton_, launchButton_, stopButton_, checkButton_}) b->setAutoDefault(false);
  guiBox_ = new QCheckBox("Gazebo window");
  guiBox_->setObjectName("show_gazebo_window");
  guiBox_->setChecked(true);
  guiBox_->setToolTip("Untick to run Gazebo without its window (gui:=false)");
  autoCheck_ = new QCheckBox("Check after");
  autoCheck_->setObjectName("auto_check");
  autoCheck_->setChecked(true);
  autoCheck_->setToolTip("Check the topics by itself some seconds after the launch (Gazebo needs time to start)");
  settle_ = new QSpinBox;
  settle_->setObjectName("settle_seconds");
  settle_->setRange(1, 600);
  settle_->setValue(20);
  settle_->setSuffix(" s");
  sample_ = new QSpinBox;
  sample_->setObjectName("sample_seconds");
  sample_->setRange(1, 60);
  sample_->setValue(6);
  sample_->setSuffix(" s");
  sample_->setToolTip("How long `ros2 topic hz` listens to each topic");
  row->addWidget(buildButton_);
  row->addWidget(launchButton_);
  row->addWidget(stopButton_);
  row->addWidget(guiBox_);
  row->addStretch(1);
  row->addWidget(autoCheck_);
  row->addWidget(settle_);
  row->addWidget(new QLabel("listen"));
  row->addWidget(sample_);
  row->addWidget(checkButton_);
  v->addLayout(row);

  commands_ = new QLabel;
  commands_->setObjectName("terminal_commands");
  commands_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  commands_->setWordWrap(true);
  QPalette hint = commands_->palette();
  hint.setColor(QPalette::WindowText, hint.color(QPalette::PlaceholderText));
  commands_->setPalette(hint);
  commands_->setVisible(false);
  auto* commandsBox = new QCheckBox("Show the same commands for a terminal");
  commandsBox->setObjectName("show_commands");
  connect(commandsBox, &QCheckBox::toggled, commands_, &QLabel::setVisible);
  v->addWidget(commandsBox);
  v->addWidget(commands_);

  state_ = new QLabel("Not running.");
  state_->setObjectName("run_state");
  planChanged_ = new QLabel("The plan changed after this run started. Stop and launch again to use the new one.");
  planChanged_->setWordWrap(true);
  planChanged_->setVisible(false);
  summary_ = new QLabel;
  summary_->setObjectName("check_summary");

  table_ = new QTableWidget;
  table_->setObjectName("check_table");
  table_->setColumnCount(4);
  table_->setHorizontalHeaderLabels({"ROS topic", "Direction", "Status", "Result"});
  table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setStretchLastSection(true);
  table_->verticalHeader()->setVisible(false);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  table_->setWordWrap(false);

  log_ = makePreview(Language::Log);
  log_->setObjectName("run_log");
  log_->setMaximumBlockCount(5000);
  log_->setPlaceholderText("What colcon, ros2 launch and the checks print appears here.");
  clearButton_ = new QPushButton("Clear output");
  clearButton_->setAutoDefault(false);
  connect(clearButton_, &QPushButton::clicked, log_, &QPlainTextEdit::clear);

  auto* top = new QWidget;
  auto* tv = new QVBoxLayout(top);
  tv->setContentsMargins(0, 0, 0, 0);
  tv->addWidget(state_);
  tv->addWidget(planChanged_);
  tv->addWidget(table_, 1);
  tv->addWidget(summary_);
  auto* bottom = new QWidget;
  auto* bv = new QVBoxLayout(bottom);
  bv->setContentsMargins(0, 0, 0, 0);
  auto* logBar = new QHBoxLayout;
  logBar->addWidget(new QLabel("<b>Output</b>"));
  logBar->addStretch(1);
  logBar->addWidget(clearButton_);
  bv->addLayout(logBar);
  bv->addWidget(log_, 1);
  auto* split = new QSplitter(Qt::Vertical);
  split->addWidget(top);
  split->addWidget(bottom);
  split->setChildrenCollapsible(false);
  split->setStretchFactor(0, 3);
  split->setStretchFactor(1, 2);
  split->setSizes({240, 140});
  v->addWidget(split, 1);

  connect(guiBox_, &QCheckBox::toggled, this, [this]() { commands_->setText(equivalentCommands()); });
  connect(buildButton_, &QPushButton::clicked, this, &RunPanel::build);
  connect(launchButton_, &QPushButton::clicked, this, &RunPanel::launch);
  connect(stopButton_, &QPushButton::clicked, this, &RunPanel::stop);
  connect(checkButton_, &QPushButton::clicked, this, [this]() {
    if (checking_) cancelChecks();
    else startChecks();
  });
}

// ----------------------------------------------------------------------------- the plan

void RunPanel::setPackage(const QString& packageName, const std::vector<gzb::TopicCheck>& checks, bool ready, PackageWriter writer) {
  package_ = packageName;
  latest_ = checks;
  ready_ = ready;
  writer_ = std::move(writer);
  if (!frozen()) {
    syncTable();
    planChanged_->setVisible(false);
  } else {
    planChanged_->setVisible(!sameChecks(checks_, latest_) || (launch_ && launchedPackage_ != package_));
  }
  commands_->setText(equivalentCommands());
  updateButtons();
}

void RunPanel::syncTable() {
  if (sameChecks(checks_, latest_) && results_.size() == checks_.size()) return;
  checks_ = latest_;
  results_.assign(checks_.size(), gzb::CheckResult{});
  fillTable();
  updateSummary();
}

void RunPanel::fillTable() {
  table_->setRowCount(static_cast<int>(checks_.size()));
  for (size_t i = 0; i < checks_.size(); ++i) {
    const gzb::TopicCheck& c = checks_[i];
    const int r = static_cast<int>(i);
    auto cell = [&](int col, const QString& text) {
      auto* item = new QTableWidgetItem(text);
      table_->setItem(r, col, item);
      return item;
    };
    auto* t = cell(0, QString::fromStdString(c.rosTopic));
    t->setToolTip(QString("%1\nROS type: %2\nGazebo topic: %3 (%4)")
                      .arg(QString::fromStdString(c.description), QString::fromStdString(c.rosType), QString::fromStdString(c.gzTopic),
                           QString::fromStdString(c.gzType)));
    cell(1, directionText(c.direction));
    cell(2, QString());
    cell(3, QString());
    const gzb::CheckResult res = i < results_.size() ? results_[i] : gzb::CheckResult{};
    auto* st = table_->item(r, 2);
    st->setText(stateText(res.state));
    st->setForeground(stateColor(res.state));
    table_->item(r, 3)->setText(QString::fromStdString(res.detail));
    table_->item(r, 3)->setToolTip(QString::fromStdString(res.detail));
  }
}

void RunPanel::setResult(size_t index, const gzb::CheckResult& r) {
  if (index >= results_.size()) return;
  results_[index] = r;
  const int row = static_cast<int>(index);
  if (auto* st = table_->item(row, 2)) {
    st->setText(stateText(r.state));
    st->setForeground(stateColor(r.state));
    QFont f = st->font();
    f.setBold(r.state == gzb::CheckState::Pass || r.state == gzb::CheckState::Fail);
    st->setFont(f);
  }
  if (auto* d = table_->item(row, 3)) {
    d->setText(QString::fromStdString(r.detail));
    d->setToolTip(QString::fromStdString(r.detail));
  }
  updateSummary();
  emit stateChanged();
}

void RunPanel::updateSummary() {
  int pass = 0, fail = 0, skipped = 0, pending = 0;
  for (const gzb::CheckResult& r : results_) {
    switch (r.state) {
      case gzb::CheckState::Pass: ++pass; break;
      case gzb::CheckState::Fail: ++fail; break;
      case gzb::CheckState::Skipped: ++skipped; break;
      default: ++pending; break;
    }
  }
  if (results_.empty()) {
    summary_->setText("No topics to check: the plan has no bridged topics.");
  } else if (pending == static_cast<int>(results_.size())) {
    summary_->setText(QString("%1 bridged topics, not checked yet.").arg(results_.size()));
  } else {
    QString text = QString("%1 pass, %2 fail").arg(pass).arg(fail);
    if (skipped) text += QString(", %1 skipped").arg(skipped);
    if (pending) text += QString(", %1 not finished").arg(pending);
    summary_->setText(text);
  }
}

QString RunPanel::workspace() const { return workspaceEdit_->text().trimmed(); }

void RunPanel::setWorkspace(const QString& path) { workspaceEdit_->setText(path); }

QString RunPanel::equivalentCommands() const {
  const QString pkg = package_.isEmpty() ? QString("<package>") : package_;
  QStringList lines;
  const QString ws = workspace();
  if (!ws.isEmpty()) {
    lines << QString("cd %1 && colcon build --packages-select %2 && source install/setup.bash").arg(ws, pkg);
  }
  QString launch = QString("ros2 launch %1 sim.launch.py").arg(pkg);
  if (guiBox_ && !guiBox_->isChecked()) launch += " gui:=false";
  lines << launch;
  lines << "ros2 topic list -t   ·   ros2 topic hz <topic>   ·   ros2 topic info <topic>";
  return lines.join("\n");
}

QStringList RunPanel::setupScripts() const {
  QStringList scripts;
  if (!qEnvironmentVariableIsSet("ROS_DISTRO") && QFileInfo::exists("/opt/ros/jazzy/setup.bash")) {
    scripts << "/opt/ros/jazzy/setup.bash";
  }
  const QString ws = workspace();
  if (!ws.isEmpty() && QFileInfo::exists(QDir(ws).filePath("install/setup.bash"))) {
    scripts << QDir(ws).filePath("install/setup.bash");
  }
  return scripts;
}

void RunPanel::appendLog(const QString& text) {
  QScrollBar* bar = log_->verticalScrollBar();
  const bool atEnd = bar->value() >= bar->maximum() - 4;
  QTextCursor cursor = log_->textCursor();
  cursor.movePosition(QTextCursor::End);
  cursor.insertText(text);
  if (atEnd) bar->setValue(bar->maximum());
}

void RunPanel::logLine(const QString& tag, const QString& text) {
  QString line = "[" + tag + "] " + text;
  if (!line.endsWith('\n')) line += '\n';
  QTextCursor c = log_->textCursor();
  c.movePosition(QTextCursor::End);
  // Start on a fresh line even when the last output had no newline.
  if (c.positionInBlock() != 0) line.prepend('\n');
  appendLog(line);
}

void RunPanel::updateButtons() {
  const bool busy = build_ != nullptr;
  const bool wsOk = !workspace().isEmpty() && static_cast<bool>(writer_);
  buildButton_->setEnabled(ready_ && wsOk && !busy && !launch_ && !checking_);
  launchButton_->setEnabled(ready_ && !busy && !launch_);
  stopButton_->setEnabled(busy || launch_);
  checkButton_->setEnabled(checking_ || (!busy && !checks_.empty() && (ready_ || launch_)));
  checkButton_->setText(checking_ ? "Cancel checks" : "Check topics");
  workspaceEdit_->setEnabled(!busy && !launch_);
  browseButton_->setEnabled(!busy && !launch_);
  guiBox_->setEnabled(!launch_);
  const char* why = "";
  if (!ready_) why = "Fix the errors listed under the previews first.";
  else if (workspace().isEmpty()) why = "Enter a workspace to build in.";
  buildButton_->setToolTip(buildButton_->isEnabled() ? "Write the package into <workspace>/src and run: colcon build --packages-select <package>" : QString(why));

  if (busy) {
    state_->setText(QString("Building %1 with colcon...").arg(package_));
  } else if (checking_) {
    int done = 0;
    for (const gzb::CheckResult& r : results_) done += (r.state == gzb::CheckState::Pass || r.state == gzb::CheckState::Fail || r.state == gzb::CheckState::Skipped);
    state_->setText(QString("Checking topics: %1 of %2 done.").arg(done).arg(results_.size()));
  } else if (launch_ && countdown_ > 0) {
    state_->setText(QString("%1 is running. Checking the topics in %2 s.").arg(launchedPackage_).arg(countdown_));
  } else if (launch_) {
    state_->setText(QString("%1 is running.").arg(launchedPackage_));
  } else {
    state_->setText("Not running.");
  }
  commands_->setText(equivalentCommands());
  emit stateChanged();
}

// ------------------------------------------------------------------------------- build

void RunPanel::build() {
  if (build_ || launch_ || checking_ || !ready_) return;
  const QString ws = workspace();
  if (ws.isEmpty() || !writer_) return;
  QDir wsDir(ws);
  if (!wsDir.exists()) {
    logLine("build", "The workspace folder does not exist: " + ws);
    return;
  }
  const QString srcDir = wsDir.filePath("src");
  QDir().mkpath(srcDir);
  const QString pkg = package_;
  const QString target = QDir(srcDir).filePath(pkg);
  if (confirmOverwrite_ && QFileInfo::exists(target) && !writtenPackages_.contains(target)) {
    const auto answer = QMessageBox::question(this, "Replace the package?",
                                              QString("%1 already exists. The files this helper generates will be replaced; "
                                                      "other files in the folder are left alone.").arg(target));
    if (answer != QMessageBox::Yes) return;
  }
  QString error;
  if (!writer_(srcDir, &error)) {
    logLine("build", "Could not write the package: " + error);
    return;
  }
  writtenPackages_ << target;
  logLine("build", "Wrote the package to " + target);
  logLine("build", "colcon build --packages-select " + pkg);

  build_ = new ProcessJob(this);
  connect(build_, &ProcessJob::outputAvailable, this, &RunPanel::appendLog);
  connect(build_, &ProcessJob::finished, this, [this, pkg](const ProcessJob::Result& r) {
    ProcessJob* job = build_;
    build_ = nullptr;
    job->deleteLater();
    QString why;
    bool ok = r.exitCode == 0 && !r.crashed && !r.failedToStart;
    if (r.failedToStart) why = "bash could not be started.";
    else if (r.programMissing()) why = "`colcon` was not found. Start this window from a terminal where ROS 2 is sourced (colcon is installed with python3-colcon-common-extensions).";
    else if (r.stopped) why = "stopped.";
    else if (!ok) why = QString("colcon exited with code %1; the messages above say why.").arg(r.exitCode);
    onBuildFinished(pkg, ok, why);
  });
  build_->start(setupScripts(), {"colcon", "build", "--packages-select", pkg}, ws);
  updateButtons();
}

void RunPanel::onBuildFinished(const QString& pkg, bool ok, const QString& why) {
  if (ok) {
    logLine("build", "Finished: " + pkg + " is installed in " + QDir(workspace()).filePath("install"));
    emit statusMessage("Built " + pkg);
  } else {
    logLine("build", "FAIL: " + why);
    emit statusMessage("Build failed");
  }
  updateButtons();
}

// ------------------------------------------------------------------------------ launch

void RunPanel::launch() {
  if (build_ || launch_ || !ready_) return;
  const QString ws = workspace();
  if (!ws.isEmpty() && !QFileInfo::exists(QDir(ws).filePath("install/setup.bash"))) {
    logLine("launch", QDir(ws).filePath("install/setup.bash") + " does not exist yet: press Build first.");
    return;
  }
  if (!frozen()) syncTable();
  launchedPackage_ = package_;
  QStringList argv = toQ(gzb::launchCommand(package_.toStdString()));
  if (!guiBox_->isChecked()) argv << "gui:=false";
  logLine("launch", argv.join(' '));

  launch_ = new ProcessJob(this);
  connect(launch_, &ProcessJob::outputAvailable, this, &RunPanel::appendLog);
  connect(launch_, &ProcessJob::finished, this, [this](const ProcessJob::Result& r) {
    ProcessJob* job = launch_;
    launch_ = nullptr;
    job->deleteLater();
    QString why;
    if (r.failedToStart) why = "bash could not be started.";
    else if (r.programMissing()) why = "`ros2` was not found. Start this window from a terminal where ROS 2 is sourced.";
    else if (r.stopped) why = "stopped.";
    else if (r.exitCode == 0) why = "ended.";
    else why = QString("exited with code %1; the messages above say why (a missing package means it was not built or sourced).").arg(r.exitCode);
    onLaunchFinished(why);
  });
  launch_->start(setupScripts(), argv, ws);
  countdown_ = autoCheck_->isChecked() ? settle_->value() : -1;
  tick_->start();
  planChanged_->setVisible(false);
  updateButtons();
}

void RunPanel::onLaunchFinished(const QString& why) {
  tick_->stop();
  countdown_ = -1;
  logLine("launch", why);
  emit statusMessage("Launch " + why);
  if (!checking_) {
    syncTable();
    planChanged_->setVisible(false);
  }
  updateButtons();
}

void RunPanel::tick() {
  if (countdown_ > 0) {
    --countdown_;
    if (countdown_ == 0) {
      countdown_ = -1;
      if (launch_ && !checking_) startChecks();
    }
  }
  updateButtons();
}

void RunPanel::stop() {
  countdown_ = -1;
  if (checking_) cancelChecks();
  if (build_) build_->stop();
  if (launch_) {
    logLine("launch", "Stopping (Ctrl-C to ros2 launch, which shuts Gazebo and the bridge down)...");
    launch_->stop();
  }
  updateButtons();
}

void RunPanel::shutdown(int maxMs) {
  cancelChecks();
  countdown_ = -1;
  if (!build_ && !launch_) return;
  QEventLoop loop;
  QTimer limit;
  limit.setSingleShot(true);
  connect(&limit, &QTimer::timeout, &loop, &QEventLoop::quit);
  QTimer poll;
  poll.setInterval(100);
  connect(&poll, &QTimer::timeout, &loop, [&]() {
    if (!build_ && !launch_) loop.quit();
  });
  stop();
  limit.start(maxMs);
  poll.start();
  loop.exec();
  // Anything still alive is killed by the job destructors together with its process group.
}

// ------------------------------------------------------------------------------ checks

void RunPanel::startChecks() {
  if (checking_ || build_) return;
  if (!frozen()) syncTable();
  if (checks_.empty()) return;
  countdown_ = -1;
  results_.assign(checks_.size(), gzb::CheckResult{});
  fillTable();
  updateSummary();
  checking_ = true;
  logLine("check", "ros2 topic list -t");

  ProcessJob* job = new ProcessJob(this);
  listJob_ = job;
  connect(job, &ProcessJob::finished, this, [this, job](const ProcessJob::Result& r) {
    if (listJob_ == job) listJob_ = nullptr;
    job->deleteLater();
    if (!checking_ || r.stopped) return;  // cancelled
    onTopicList(r.output, r.programMissing(), r.failedToStart || r.timedOut);
  });
  job->start(setupScripts(), toQ(gzb::topicListCommand()), workspace(), 30000);
  updateButtons();
}

void RunPanel::cancelChecks() {
  if (!checking_) return;
  checking_ = false;
  queue_.clear();
  if (listJob_) {
    listJob_->stop(15, 1000);
    listJob_ = nullptr;
  }
  for (CheckRun* run : active_) {
    for (ProcessJob* j : {run->info, run->hz, run->gz, run->pub}) {
      if (j) {
        j->disconnect(this);
        j->stop(15, 1000);
        j->deleteLater();
      }
    }
    delete run->delay;
    delete run;
  }
  active_.clear();
  for (size_t i = 0; i < results_.size(); ++i) {
    if (results_[i].state == gzb::CheckState::Pending || results_[i].state == gzb::CheckState::Running) {
      gzb::CheckResult r;
      r.state = gzb::CheckState::Skipped;
      r.detail = "Cancelled.";
      setResult(i, r);
    }
  }
  logLine("check", "Cancelled.");
  if (!frozen()) syncTable();
  updateButtons();
}

void RunPanel::onTopicList(const QString& output, bool missing, bool failed) {
  if (missing || failed) {
    const QString why = missing ? "`ros2` was not found. Start this window from a terminal where ROS 2 is sourced."
                                : "`ros2 topic list` did not finish.";
    for (size_t i = 0; i < results_.size(); ++i) {
      gzb::CheckResult r;
      r.state = gzb::CheckState::Fail;
      r.detail = why.toStdString();
      setResult(i, r);
    }
    logLine("check", "FAIL: " + why);
    finishChecks();
    return;
  }
  topics_ = gzb::parseTopicList(output.toStdString());
  logLine("check", QString("%1 topics in ROS. Listening to each bridged topic for %2 s (%3 at a time).")
                       .arg(topics_.size()).arg(sample_->value()).arg(kParallelChecks));
  queue_.clear();
  for (size_t i = 0; i < checks_.size(); ++i) queue_.push_back(i);
  startNextChecks();
}

void RunPanel::startNextChecks() {
  if (!checking_) return;
  while (static_cast<int>(active_.size()) < kParallelChecks && !queue_.empty()) {
    const size_t i = queue_.front();
    queue_.erase(queue_.begin());
    runCheck(i);
    if (!checking_) return;
  }
  if (queue_.empty() && active_.empty()) finishChecks();
}

void RunPanel::runCheck(size_t index) {
  gzb::CheckResult running;
  running.state = gzb::CheckState::Running;
  setResult(index, running);
  if (checks_[index].direction == gzb::Direction::RosToGz) runCommanded(index);
  else runPublished(index);
  updateButtons();
}

void RunPanel::runPublished(size_t index) {
  auto* run = new CheckRun;
  run->index = index;
  active_.push_back(run);
  const gzb::TopicCheck& c = checks_[index];
  if (!topics_.count(c.rosTopic)) {
    finishCheckRun(run, gzb::evaluatePublished(c, topics_, ""));
    return;
  }
  run->hz = new ProcessJob(this);
  connect(run->hz, &ProcessJob::finished, this, [this, run](const ProcessJob::Result& r) {
    finishCheckRun(run, gzb::evaluatePublished(checks_[run->index], topics_, r.output.toStdString()));
  });
  run->hz->start(setupScripts(), toQ(gzb::hzCommand(c.rosTopic)), workspace(), sample_->value() * 1000);
}

void RunPanel::runCommanded(size_t index) {
  auto* run = new CheckRun;
  run->index = index;
  active_.push_back(run);
  const gzb::TopicCheck& c = checks_[index];
  if (!topics_.count(c.rosTopic)) {
    finishCheckRun(run, gzb::evaluateCommanded(c, topics_, "", true, false));
    return;
  }

  // 1. Is anything in ROS listening (the bridge's subscriber)?
  run->info = new ProcessJob(this);
  connect(run->info, &ProcessJob::finished, this, [this, run](const ProcessJob::Result& infoResult) {
    const gzb::TopicCheck& c = checks_[run->index];
    const std::string info = infoResult.output.toStdString();
    run->infoText = infoResult.output;
    if (gzb::parseSubscriptionCount(info) == 0 || c.payload.empty()) {
      finishCheckRun(run, gzb::evaluateCommanded(c, topics_, info, true, false));
      return;
    }
    // 2. Listen on the Gazebo side, then publish a harmless message on the ROS side. `gz topic -e -n 1`
    //    ends by itself when one message arrived, so its exit status is the evidence.
    run->gz = new ProcessJob(this);
    connect(run->gz, &ProcessJob::finished, this, [this, run](const ProcessJob::Result& gzResult) {
      const gzb::TopicCheck& c = checks_[run->index];
      if (run->delay) run->delay->stop();
      if (run->pub && !run->pubDone) run->pub->stop(2, 1000);
      const bool gzAvailable = !gzResult.programMissing() && !gzResult.failedToStart;
      const bool received = gzAvailable && gzResult.exitCode == 0 && !gzResult.timedOut && !gzResult.crashed;
      gzb::CheckResult r = gzb::evaluateCommanded(c, topics_, run->infoText.toStdString(), gzAvailable, received);
      if (!received && gzAvailable && run->pubDone && run->pubResult.exitCode != 0 && !run->pubResult.timedOut && !run->pubResult.stopped) {
        r.state = gzb::CheckState::Fail;
        r.detail = "Publishing the sample message failed: " + lastLine(run->pubResult.output).toStdString();
      }
      finishCheckRun(run, r);
    });
    run->gz->start(setupScripts(), toQ(gzb::gzEchoCommand(c.gzTopic)), workspace(), 10000);

    run->delay = new QTimer(this);  // give `gz topic -e` a moment to subscribe
    run->delay->setSingleShot(true);
    connect(run->delay, &QTimer::timeout, this, [this, run]() {
      run->pub = new ProcessJob(this);
      connect(run->pub, &ProcessJob::finished, this, [run](const ProcessJob::Result& r) {
        run->pubDone = true;
        run->pubResult = r;
      });
      run->pub->start(setupScripts(), toQ(gzb::publishCommand(checks_[run->index])), workspace(), 15000);
    });
    run->delay->start(1000);
  });
  run->info->start(setupScripts(), toQ(gzb::infoCommand(c.rosTopic)), workspace(), 15000);
}

void RunPanel::finishCheckRun(CheckRun* run, const gzb::CheckResult& r) {
  if (run->done) return;
  run->done = true;
  const size_t index = run->index;
  setResult(index, r);
  logLine("check", QString("%1: %2 - %3")
                       .arg(QString::fromStdString(checks_[index].rosTopic), stateText(r.state), QString::fromStdString(r.detail)));
  for (ProcessJob* j : {run->info, run->hz, run->gz, run->pub}) {
    if (!j) continue;
    j->disconnect(this);
    if (j->running()) j->stop(15, 1000);
    j->deleteLater();
  }
  delete run->delay;
  active_.erase(std::remove(active_.begin(), active_.end(), run), active_.end());
  delete run;
  startNextChecks();
  updateButtons();
}

void RunPanel::finishChecks() {
  checking_ = false;
  queue_.clear();
  int fail = 0;
  for (const gzb::CheckResult& r : results_) fail += r.state == gzb::CheckState::Fail;
  logLine("check", fail == 0 ? "PASS: every checked topic works." : QString("FAIL: %1 topic(s) did not work.").arg(fail));
  if (!frozen()) syncTable();
  planChanged_->setVisible(launch_ && (!sameChecks(checks_, latest_) || launchedPackage_ != package_));
  updateButtons();
  emit checksFinished();
}

}  // namespace gzgui
