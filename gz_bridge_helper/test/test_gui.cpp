// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// Drives the real window (offscreen) and the Run tab. The ROS tools are replaced by the scripts
// in test/mock_bin, whose behaviour is set by files in a per-test state directory, so the
// whole path "click Build -> Launch -> Check topics -> pass/fail per row" runs for real
// (processes, process groups, timers) without a ROS installation.
#include <QApplication>
#include <QCheckBox>
#include <QClipboard>
#include <QComboBox>
#include <QDir>
#include <QEventLoop>
#include <QFile>
#include <QFileInfo>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTabWidget>
#include <QTableWidget>
#include <QTemporaryDir>
#include <QTimer>
#include <QTreeWidget>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <set>

#include "gz_bridge_helper/verify.hpp"
#include "highlighter.hpp"
#include "main_window.hpp"
#include "process_job.hpp"
#include "run_panel.hpp"

#ifndef GZBH_TEST_DATA
#error "GZBH_TEST_DATA must point at test/data"
#endif
#ifndef GZBH_MOCK_BIN
#error "GZBH_MOCK_BIN must point at test/mock_bin"
#endif

using namespace gzgui;

namespace {

int g_checks = 0, g_failed = 0;

void report(bool ok, const char* expr, const char* file, int line, const QString& extra = QString()) {
  ++g_checks;
  if (!ok) {
    ++g_failed;
    std::cerr << file << ":" << line << ": CHECK failed: " << expr << (extra.isEmpty() ? "" : "  (" + extra.toStdString() + ")") << "\n";
  }
}

#define CHECK(cond) report((cond), #cond, __FILE__, __LINE__)
#define CHECK_MSG(cond, msg) report((cond), #cond, __FILE__, __LINE__, (msg))
#define CHECK_EQ_Q(a, b) report((a) == (b), #a " == " #b, __FILE__, __LINE__, QString("got '%1' and '%2'").arg(a).arg(b))

QString data(const QString& name) { return QString(GZBH_TEST_DATA) + "/" + name; }

/// Runs the event loop until `cond` holds; false after `ms` milliseconds.
bool waitFor(const std::function<bool()>& cond, int ms = 15000) {
  QEventLoop loop;
  QTimer poll, limit;
  limit.setSingleShot(true);
  QObject::connect(&poll, &QTimer::timeout, &loop, [&]() {
    if (cond()) loop.quit();
  });
  QObject::connect(&limit, &QTimer::timeout, &loop, &QEventLoop::quit);
  if (cond()) return true;
  poll.start(20);
  limit.start(ms);
  loop.exec();
  return cond();
}

void pause(int ms) {
  QEventLoop loop;
  QTimer::singleShot(ms, &loop, &QEventLoop::quit);
  loop.exec();
}

QString readFile(const QString& path) {
  QFile f(path);
  return f.open(QIODevice::ReadOnly) ? QString::fromUtf8(f.readAll()) : QString();
}

void writeFile(const QString& path, const QString& text) {
  QDir().mkpath(QFileInfo(path).absolutePath());
  QFile f(path);
  f.open(QIODevice::WriteOnly);
  f.write(text.toUtf8());
}

bool pidAlive(qint64 pid) { return pid > 0 && ::kill(static_cast<pid_t>(pid), 0) == 0; }

/// The state directory of the mock ROS tools for one scenario.
struct Mock {
  QTemporaryDir dir;
  Mock() { qputenv("GZBH_MOCK_STATE", dir.path().toUtf8()); }
  QString path(const QString& rel) const { return QDir(dir.path()).filePath(rel); }
  bool has(const QString& rel) const { return QFileInfo::exists(path(rel)); }
  void touch(const QString& rel, const QString& text = "x") { writeFile(path(rel), text); }
  QString text(const QString& rel) const { return readFile(path(rel)); }
  static QString key(const std::string& topic) { return QString::fromStdString(topic).replace('/', '_'); }

  /// Everything the plan bridges exists in ROS with the right type and is alive.
  void healthy(const std::vector<gzb::TopicCheck>& checks) {
    QString topics;
    for (const gzb::TopicCheck& c : checks) {
      topics += QString("%1 [%2]\n").arg(QString::fromStdString(c.rosTopic), QString::fromStdString(c.rosType));
      if (c.direction != gzb::Direction::RosToGz) touch("hz/" + key(c.rosTopic), "29.5");
    }
    touch("topics", topics);
  }
};

int rowOf(const RunPanel& p, const std::string& topic) {
  for (size_t i = 0; i < p.checks().size(); ++i) {
    if (p.checks()[i].rosTopic == topic) return static_cast<int>(i);
  }
  return -1;
}

int firstRow(const RunPanel& p, bool commanded) {
  for (size_t i = 0; i < p.checks().size(); ++i) {
    if ((p.checks()[i].direction == gzb::Direction::RosToGz) == commanded) return static_cast<int>(i);
  }
  return -1;
}

QString statusOf(const RunPanel& p, int row) { return p.table()->item(row, 2)->text(); }
QString detailOf(const RunPanel& p, int row) { return p.table()->item(row, 3)->text(); }

/// A window with the diffbot that already has sensors, opened with the mock tools on the PATH.
struct Fixture {
  MainWindow w;
  Fixture(const QString& urdf = "diffbot_gz.urdf") {
    w.runPanel()->setConfirmOverwrite(false);
    w.runPanel()->setWorkspace(QString());  // the last test's workspace is remembered in the settings
    w.runPanel()->sampleSpin()->setValue(1);
    w.runPanel()->autoCheckBox()->setChecked(false);
    CHECK(w.openUrdf(data(urdf)));
  }
};

// =============================================================== ProcessJob

void testProcessJob() {
  using R = ProcessJob::Result;
  auto run = [](const QStringList& setup, const QStringList& argv, int timeout, R& out) {
    ProcessJob job;
    bool done = false;
    QObject::connect(&job, &ProcessJob::finished, [&](const R& r) {
      out = r;
      done = true;
    });
    job.start(setup, argv, QString(), timeout);
    return waitFor([&]() { return done; }, 10000);
  };

  R r;
  CHECK(run({}, {"printf", "hello\\n"}, 0, r));
  CHECK(r.exitCode == 0 && !r.crashed && r.output == "hello\n");

  // Arguments reach the program exactly as given: no shell interprets them.
  CHECK(run({}, {"printf", "%s|", "a b", "$HOME", "'q'", "x;y"}, 0, r));
  CHECK_EQ_Q(r.output, QString("a b|$HOME|'q'|x;y|"));

  // A program that does not exist is reported as such (the shell's exit code 127).
  CHECK(run({}, {"definitely-not-a-program-xyz"}, 0, r));
  CHECK(r.programMissing());

  // Scripts are sourced first, in order.
  QTemporaryDir tmp;
  writeFile(tmp.filePath("a.bash"), "export FIRST=one\n");
  writeFile(tmp.filePath("b.bash"), "export SECOND=\"$FIRST-two\"\n");
  CHECK(run({tmp.filePath("a.bash"), tmp.filePath("b.bash")}, {"bash", "-c", "echo $SECOND"}, 0, r));
  CHECK_EQ_Q(r.output.trimmed(), QString("one-two"));

  // Exit codes and stderr are kept; colour codes are removed.
  CHECK(run({}, {"bash", "-c", "printf '\\033[31mred\\033[0m\\n' >&2; exit 3"}, 0, r));
  CHECK(r.exitCode == 3 && r.output == "red\n");
  CHECK_EQ_Q(stripTerminalCodes("a\x1b[1;32mb\x1b[0m\r\nc\rd"), QString("ab\nc\nd"));

  // The time limit stops the process.
  CHECK(run({}, {"sleep", "30"}, 300, r));
  CHECK(r.timedOut && !r.stopped);

  // stop() reaches the children too, and a process that ignores Ctrl-C is escalated to SIGTERM.
  for (const bool stubborn : {false, true}) {
    QTemporaryDir state;
    const QString script = tmp.filePath(stubborn ? "stubborn.sh" : "polite.sh");
    writeFile(script, QString("#!/usr/bin/env bash\n%1sleep 300 &\necho $! > %2/child.pid\necho $$ > %2/leader.pid\n"
                              "while true; do sleep 0.05; done\n")
                          .arg(stubborn ? "trap '' INT\n" : "trap 'kill $! ; exit 0' INT\n", state.path()));
    QFile::setPermissions(script, QFile::permissions(script) | QFileDevice::ExeUser);
    ProcessJob job;
    bool done = false;
    R result;
    QObject::connect(&job, &ProcessJob::finished, [&](const R& x) {
      result = x;
      done = true;
    });
    job.start({}, {script});
    CHECK(waitFor([&]() { return QFileInfo::exists(state.filePath("child.pid")) && QFileInfo::exists(state.filePath("leader.pid")); }));
    pause(100);
    const qint64 child = readFile(state.filePath("child.pid")).trimmed().toLongLong();
    CHECK(pidAlive(child));
    job.stop(2, 400);
    CHECK(waitFor([&]() { return done; }, 8000));
    CHECK(result.stopped);
    CHECK(waitFor([&]() { return !pidAlive(child); }, 5000));  // nothing is left behind
    if (!pidAlive(child)) continue;
    ::kill(static_cast<pid_t>(child), SIGKILL);
  }
}

void testHighlighter() {
  CHECK(languageForFile("a/b/sim.launch.py") == Language::Python);
  CHECK(languageForFile("x_bridge.yaml") == Language::Yaml);
  CHECK(languageForFile("robot.urdf") == Language::Xml);
  CHECK(languageForFile("package.xml") == Language::Xml);
  CHECK(languageForFile("README.md") == Language::Plain);
  CHECK(languageForFile("CMakeLists.txt") == Language::Plain);
  QPlainTextEdit* e = makePreview(Language::Yaml);
  setPreviewText(e, "a: 1\n# c\n");
  CHECK_EQ_Q(e->toPlainText(), QString("a: 1\n# c\n"));
  delete e;
}

// ============================================================ the window

void testWindowBasics() {
  Mock mock;
  MainWindow w;
  CHECK(!w.canSave());
  CHECK(w.bridgeTable()->rowCount() == 0);
  CHECK(w.findChild<QLabel*>("summary_label")->text().contains("Open a URDF"));
  CHECK(!w.findChild<QPushButton*>("save_package")->isEnabled());

  CHECK(w.openUrdf(data("diffbot_gz.urdf")));
  CHECK(w.project().ready());
  CHECK(w.canSave());
  CHECK(w.findChild<QLabel*>("urdf_summary")->text().contains("diffbot"));
  CHECK(w.windowTitle().contains("diffbot_gz.urdf"));
  CHECK(w.itemsTree()->topLevelItemCount() >= 3);  // Sensors, Drive, Joints, Odometry ...
  CHECK(w.findChild<QLabel*>("mode_label")->text().contains("Gazebo's own systems"));

  // The table shows exactly the plan.
  const gzb::Plan& plan = w.project().plan();
  CHECK(!plan.bridge.empty());
  CHECK(w.bridgeTable()->rowCount() == static_cast<int>(plan.bridge.size()));
  for (size_t i = 0; i < plan.bridge.size(); ++i) {
    CHECK(w.bridgeTable()->item(static_cast<int>(i), 0)->text().toStdString() == plan.bridge[i].rosTopic);
    CHECK(w.bridgeTable()->item(static_cast<int>(i), 2)->text().toStdString() == plan.bridge[i].gzTopic);
  }
  bool clockRow = false;
  for (int r = 0; r < w.bridgeTable()->rowCount(); ++r) clockRow = clockRow || w.bridgeTable()->item(r, 0)->text() == "/clock";
  CHECK(clockRow);

  // The previews are the generated files.
  const gzb::Generated& g = w.project().generated();
  CHECK(!w.view("bridge_yaml_view")->toPlainText().isEmpty());
  CHECK(w.view("bridge_yaml_view")->toPlainText().contains("ros_topic_name: /clock") || w.view("bridge_yaml_view")->toPlainText().contains("/clock"));
  CHECK(w.view("launch_view")->toPlainText().contains("def generate_launch_description"));
  CHECK(w.view("world_view")->toPlainText().contains("<sdf"));
  CHECK(w.view("urdf_view")->toPlainText().contains("<robot name=\"diffbot\""));
  CHECK(w.view("urdf_changes_view")->toPlainText().contains("gz-sim-diff-drive-system"));
  CHECK(w.packageFileBox()->count() >= 3);  // package.xml, CMakeLists.txt, README.md
  CHECK(w.packageFileBox()->findText("package.xml") >= 0);
  w.packageFileBox()->setCurrentIndex(w.packageFileBox()->findText("package.xml"));
  CHECK(w.view("package_file_view")->toPlainText().contains("<package"));
  CHECK(g.file("package.xml") != nullptr);

  // The original is shown unchanged: the patched URDF contains it byte for byte, plus additions.
  CHECK(w.view("urdf_view")->toPlainText().contains("<link name=\"caster_link\">"));

  // Ticking and unticking an item changes the table.
  const int before = w.bridgeTable()->rowCount();
  CHECK(w.itemChecked("drive:diff"));
  w.setItemChecked("drive:diff", false);
  w.refreshNow();
  CHECK(!w.itemChecked("drive:diff"));
  CHECK(w.bridgeTable()->rowCount() < before);
  bool cmdVel = false;
  for (int r = 0; r < w.bridgeTable()->rowCount(); ++r) cmdVel = cmdVel || w.bridgeTable()->item(r, 0)->text() == "/cmd_vel";
  CHECK(!cmdVel);
  w.setItemChecked("drive:diff", true);
  w.refreshNow();
  CHECK(w.bridgeTable()->rowCount() == before);

  // Selecting an item shows why it is suggested.
  QTreeWidgetItem* cat = w.itemsTree()->topLevelItem(0);
  QTreeWidgetItem* child = cat->child(0);
  w.itemsTree()->setCurrentItem(child);
  CHECK(!w.findChild<QLabel*>("item_detail")->text().isEmpty());

  // "Reset to suggestions".
  w.setItemChecked("drive:diff", false);
  for (QPushButton* b : w.findChildren<QPushButton*>()) {
    if (b->text() == "Reset to suggestions") b->click();
  }
  CHECK(w.itemChecked("drive:diff"));
}

void testSettingsFields() {
  Mock mock;
  Fixture f;
  MainWindow& w = f.w;
  const std::string defaultPackage = w.project().plan().packageName;
  CHECK(defaultPackage == "diffbot_sim");
  CHECK_EQ_Q(w.field("package_name")->placeholderText(), QString::fromStdString(defaultPackage));
  CHECK(!w.field("spawn_z")->placeholderText().isEmpty());

  w.field("package_name")->setText("my_pkg");
  w.field("model_name")->setText("bot");
  w.field("spawn_x")->setText("2.5");
  w.field("spawn_yaw")->setText("1.57");
  w.refreshNow();
  const gzb::Plan& p = w.project().plan();
  CHECK(p.packageName == "my_pkg" && p.modelName == "bot");
  CHECK(std::fabs(p.spawnXyz.x - 2.5) < 1e-12 && std::fabs(p.spawnYaw - 1.57) < 1e-12);
  CHECK(p.spawnXyz.z > 0.05 && p.spawnXyz.z < 0.5);  // still the computed height
  CHECK(w.findChild<QLabel*>("summary_label")->text().contains("my_pkg"));
  CHECK(w.view("launch_view")->toPlainText().contains("my_pkg"));
  // The Gazebo topics follow the model name.
  bool botTopic = false;
  for (const gzb::BridgeEntry& e : p.bridge) botTopic = botTopic || e.gzTopic.find("/model/bot/") == 0;
  CHECK(botTopic);

  // Not a number: marked and ignored.
  w.field("spawn_y")->setText("abc");
  w.refreshNow();
  CHECK(std::fabs(w.project().plan().spawnXyz.y) < 1e-12);
  CHECK(w.field("spawn_y")->styleSheet().contains("border"));
  w.field("spawn_y")->setText("-1");
  w.refreshNow();
  CHECK(std::fabs(w.project().plan().spawnXyz.y + 1) < 1e-12);
  CHECK(w.field("spawn_y")->styleSheet().isEmpty());

  // An invalid package name is an error and blocks saving.
  w.field("package_name")->setText("My Pkg!");
  w.refreshNow();
  CHECK(!w.canSave());
  CHECK(!w.findChild<QPushButton*>("save_package")->isEnabled());
  CHECK(w.issuesList()->isVisible() || w.issuesList()->count() > 0);
  bool sawError = false;
  for (int i = 0; i < w.issuesList()->count(); ++i) sawError = sawError || w.issuesList()->item(i)->text().startsWith("Error");
  CHECK(sawError);
  CHECK(!w.runPanel()->buildButton()->isEnabled());
  CHECK(!w.runPanel()->launchButton()->isEnabled());

  w.field("package_name")->clear();
  w.refreshNow();
  CHECK(w.canSave());
  CHECK(w.project().plan().packageName == defaultPackage);

  // Opening another URDF clears the typed values.
  CHECK(w.openUrdf(data("diffbot_plain.urdf")));
  CHECK(w.field("model_name")->text().isEmpty());
  CHECK(w.field("spawn_x")->text().isEmpty());
}

void testYamlFiles() {
  Mock mock;
  Fixture f;
  MainWindow& w = f.w;
  CHECK(w.yamlList()->count() == 0);

  // The user's choices survive adding a YAML file.
  CHECK(w.itemChecked("joint_states"));
  w.setItemChecked("joint_states", false);
  CHECK(!w.itemChecked("joint_states"));
  CHECK(w.addYaml(data("bridge_existing.yaml")));
  CHECK(w.yamlList()->count() == 1);
  CHECK(w.yamlList()->item(0)->text().contains("bridge_existing.yaml"));
  CHECK(w.yamlList()->item(0)->text().contains("bridge"));
  CHECK(!w.itemChecked("joint_states"));  // the choice made before survived the re-detection
  CHECK(w.itemChecked("drive:diff"));
  bool battery = false;
  for (int r = 0; r < w.bridgeTable()->rowCount(); ++r) battery = battery || w.bridgeTable()->item(r, 0)->text() == "/battery";
  CHECK(battery);
  bool userRow = false;
  for (int r = 0; r < w.bridgeTable()->rowCount(); ++r) userRow = userRow || w.bridgeTable()->item(r, 6)->text().contains("bridge_existing.yaml");
  CHECK(userRow);
  CHECK(w.issuesList()->count() > 0);  // the file has deliberately broken entries

  // The same file twice is ignored.
  CHECK(!w.addYaml(data("bridge_existing.yaml")));
  CHECK(w.yamlList()->count() == 1);

  CHECK(w.addYaml(data("overrides.yaml")));
  CHECK(w.yamlList()->count() == 2);
  CHECK(w.yamlList()->item(1)->text().contains("overrides"));

  w.yamlList()->setCurrentRow(0);
  w.findChild<QPushButton*>("remove_yaml")->click();
  CHECK(w.yamlList()->count() == 1);
  CHECK(w.yamlList()->item(0)->text().contains("overrides.yaml"));
  battery = false;
  for (int r = 0; r < w.bridgeTable()->rowCount(); ++r) battery = battery || w.bridgeTable()->item(r, 0)->text() == "/battery";
  CHECK(!battery);

  // A missing file becomes an error in the list instead of a crash.
  CHECK(!w.addYaml(data("does_not_exist.yaml")));
  bool unreadable = false;
  for (int i = 0; i < w.issuesList()->count(); ++i) unreadable = unreadable || w.issuesList()->item(i)->text().contains("Cannot read");
  CHECK(unreadable);
  CHECK(!w.canSave());
  w.yamlList()->setCurrentRow(w.yamlList()->count() - 1);
  w.removeYaml(w.yamlList()->currentRow());
  CHECK(w.canSave());
}

void testRos2ControlWindow() {
  Mock mock;
  Fixture f("diffbot_ros2control.urdf");
  MainWindow& w = f.w;
  CHECK(w.findChild<QLabel*>("mode_label")->text().contains("ros2_control"));
  CHECK(w.addYaml(data("controllers.yaml")));
  CHECK(w.yamlList()->item(0)->text().contains("controllers"));
  // Only /clock (and sensors) are bridged; commands and odometry come from the controllers.
  for (int r = 0; r < w.bridgeTable()->rowCount(); ++r) {
    CHECK(w.bridgeTable()->item(r, 0)->text() != "/cmd_vel");
  }
  CHECK(w.view("launch_view")->toPlainText().contains("controller_manager"));
  CHECK(w.packageFileBox()->findText("config/controllers.yaml") >= 0);
}

void testBadUrdfAndReload() {
  Mock mock;
  QTemporaryDir tmp;
  MainWindow w;

  // A file that is not a URDF.
  writeFile(tmp.filePath("bad.urdf"), "this is not xml");
  CHECK(!w.openUrdf(tmp.filePath("bad.urdf")));
  CHECK(!w.canSave());
  CHECK(w.issuesList()->count() > 0);
  CHECK(w.issuesList()->item(0)->text().startsWith("Error"));
  CHECK(w.bridgeTable()->rowCount() == 0);
  CHECK(w.findChild<QLabel*>("urdf_summary")->text().contains("cannot be used"));

  // A file that is not there.
  CHECK(!w.openUrdf(tmp.filePath("missing.urdf")));
  CHECK(w.issuesList()->item(0)->text().contains("Cannot read"));

  // Opening a good one and then a bad one never shows the first robot's output as "last valid".
  CHECK(w.openUrdf(data("diffbot_plain.urdf")));
  CHECK(!w.view("launch_view")->toPlainText().isEmpty());
  CHECK(!w.openUrdf(tmp.filePath("bad.urdf")));
  CHECK(w.view("launch_view")->toPlainText().isEmpty());
  CHECK(!w.findChild<QLabel*>("stale_label")->isVisible());

  // Reload picks up changes in the file and keeps the ticks.
  QString text = readFile(data("diffbot_plain.urdf"));
  writeFile(tmp.filePath("robot.urdf"), text);
  CHECK(w.openUrdf(tmp.filePath("robot.urdf")));
  const int rows = w.bridgeTable()->rowCount();
  CHECK(w.itemChecked("joint_states"));
  w.setItemChecked("joint_states", false);
  CHECK(w.project().detection().sensors.empty());
  QString withCamera = text;
  withCamera.replace("</robot>",
                     "<link name=\"front_camera_link\"><inertial><mass value=\"0.05\"/><inertia ixx=\"1e-5\" ixy=\"0\" ixz=\"0\" iyy=\"1e-5\" iyz=\"0\" izz=\"1e-5\"/></inertial></link>"
                     "<joint name=\"front_camera_joint\" type=\"fixed\"><parent link=\"base_link\"/><child link=\"front_camera_link\"/><origin xyz=\"0.2 0 0.1\"/></joint></robot>");
  writeFile(tmp.filePath("robot.urdf"), withCamera);
  w.reloadAll();
  CHECK(w.project().detection().sensors.size() == 1);
  CHECK(w.bridgeTable()->rowCount() > rows);   // the new camera brings topics
  CHECK(!w.itemChecked("joint_states"));       // the choice made before is kept
  CHECK(w.itemChecked("sensor:front_camera"));

  // Saving.
  QTemporaryDir out;
  QString error;
  CHECK(w.savePackageTo(out.path(), &error));
  CHECK(QFileInfo(out.filePath("diffbot_sim/package.xml")).isFile());
  CHECK(QFileInfo(out.filePath("diffbot_sim/launch/sim.launch.py")).isFile());
  CHECK(QFileInfo(out.filePath("diffbot_sim/urdf/diffbot.urdf")).isFile());
  const QString yamlPath = out.filePath("saved_bridge.yaml");
  CHECK(w.saveBridgeYamlTo(yamlPath, &error));
  CHECK_EQ_Q(readFile(yamlPath), w.view("bridge_yaml_view")->toPlainText());
  // The package file on disk is what the preview shows.
  CHECK_EQ_Q(readFile(out.filePath("diffbot_sim/launch/sim.launch.py")), w.view("launch_view")->toPlainText());

  // With errors nothing is written.
  w.field("package_name")->setText("Bad Name");
  w.refreshNow();
  QTemporaryDir out2;
  CHECK(!w.savePackageTo(out2.path(), &error));
  CHECK(!error.isEmpty());
  CHECK(QDir(out2.path()).entryList(QDir::NoDotAndDotDot | QDir::AllEntries).isEmpty());
  CHECK(!w.saveBridgeYamlTo(out2.filePath("x.yaml"), &error));
}

void testDropAndClipboard() {
  Mock mock;
  MainWindow w;
  // (Qt does not deliver a drop event made by hand, so the handler's work is tested directly.)
  w.openPaths({data("diffbot_gz.urdf")});
  CHECK(w.project().model().valid);
  CHECK(w.project().model().name == "diffbot");
  w.openPaths({data("controllers.yaml"), data("overrides.yaml")});
  CHECK(w.yamlList()->count() == 2);

  w.tabs()->setCurrentIndex(0);
  for (QPushButton* b : w.findChildren<QPushButton*>()) {
    if (b->text() == "Copy to clipboard") b->click();
  }
  CHECK_EQ_Q(QApplication::clipboard()->text(), w.view("bridge_yaml_view")->toPlainText());
  w.tabs()->setCurrentIndex(1);
  for (QPushButton* b : w.findChildren<QPushButton*>()) {
    if (b->text() == "Copy to clipboard") b->click();
  }
  CHECK_EQ_Q(QApplication::clipboard()->text(), w.view("launch_view")->toPlainText());
}

// ============================================================ the Run tab

void testRunPanelHappyPath() {
  Mock mock;
  Fixture f;
  MainWindow& w = f.w;
  RunPanel* run = w.runPanel();
  QTemporaryDir ws;
  CHECK(!run->buildButton()->isEnabled());  // no workspace yet
  CHECK(run->launchButton()->isEnabled());
  CHECK(run->table()->rowCount() == static_cast<int>(w.project().plan().bridge.size()));
  CHECK(run->equivalentCommands().contains("ros2 launch diffbot_sim sim.launch.py"));

  run->setWorkspace(ws.path());
  CHECK(run->buildButton()->isEnabled());
  CHECK(run->equivalentCommands().contains("colcon build --packages-select diffbot_sim"));

  // Launch before building: refused, with the reason.
  run->launchButton()->click();
  CHECK(!run->launched());
  CHECK(run->log()->toPlainText().contains("Build first"));

  // Build: writes the package into <ws>/src and runs colcon.
  run->buildButton()->click();
  CHECK(run->building());
  CHECK(!run->launchButton()->isEnabled());
  CHECK(waitFor([&]() { return !run->building(); }));
  CHECK(QFileInfo(QDir(ws.path()).filePath("src/diffbot_sim/package.xml")).isFile());
  CHECK(QFileInfo(QDir(ws.path()).filePath("src/diffbot_sim/launch/sim.launch.py")).isFile());
  CHECK(QFileInfo(QDir(ws.path()).filePath("install/setup.bash")).isFile());
  CHECK(run->log()->toPlainText().contains("Finished <<< diffbot_sim"));
  CHECK(mock.text("calls.log").contains("colcon build --packages-select diffbot_sim (in " + ws.path()));

  // Launch.
  mock.healthy(run->checks());
  run->launchButton()->click();
  CHECK(run->launched());
  CHECK(run->stopButton()->isEnabled());
  CHECK(!run->launchButton()->isEnabled());
  CHECK(waitFor([&]() { return mock.has("launched"); }));
  CHECK(waitFor([&]() { return run->log()->toPlainText().contains("sourced=1"); }));  // install/setup.bash was sourced
  CHECK(mock.text("launch_args").contains("launch diffbot_sim sim.launch.py"));
  CHECK(!mock.text("launch_args").contains("gui:=false"));
  CHECK(run->log()->toPlainText().contains("[launch] ros2 launch diffbot_sim sim.launch.py"));

  // Check every bridged topic.
  bool finished = false;
  QObject::connect(run, &RunPanel::checksFinished, [&]() { finished = true; });
  run->checkButton()->click();
  CHECK(run->checking());
  CHECK(run->checkButton()->text() == "Cancel checks");
  CHECK(waitFor([&]() { return finished; }, 60000));
  CHECK(!run->checking());
  int pass = 0, other = 0;
  for (size_t i = 0; i < run->results().size(); ++i) {
    if (run->results()[i].state == gzb::CheckState::Pass) ++pass;
    else {
      ++other;
      std::cerr << "not pass: " << run->checks()[i].rosTopic << ": " << run->results()[i].detail << "\n";
    }
  }
  CHECK(pass == static_cast<int>(run->checks().size()) && other == 0);
  CHECK(run->summaryLabel()->text().startsWith(QString("%1 pass, 0 fail").arg(pass)));
  const int cmd = firstRow(*run, true);
  const int pub = firstRow(*run, false);
  CHECK(cmd >= 0 && pub >= 0);
  CHECK_EQ_Q(statusOf(*run, cmd), QString("Pass"));
  CHECK(detailOf(*run, cmd).contains("reached Gazebo"));
  CHECK(detailOf(*run, pub).contains("29.5 Hz"));
  // The sample message was really published on the ROS side of the commanded topic.
  CHECK(mock.text("published.log").contains(QString::fromStdString(run->checks()[static_cast<size_t>(cmd)].rosTopic)));
  CHECK(run->log()->toPlainText().contains("PASS: every checked topic works."));

  // Stop: Ctrl-C reaches ros2 launch, which shuts its children down.
  const qint64 child = mock.text("child.pid").trimmed().toLongLong();
  CHECK(pidAlive(child));
  run->stopButton()->click();
  CHECK(waitFor([&]() { return !run->launched(); }));
  CHECK(mock.has("launch_stopped"));
  CHECK(waitFor([&]() { return !pidAlive(child); }, 5000));
  CHECK(run->launchButton()->isEnabled());
  CHECK(run->log()->toPlainText().contains("stopped."));
  CHECK_EQ_Q(run->findChild<QLabel*>("run_state")->text(), QString("Not running."));

  if (const char* dir = std::getenv("GZBH_SCREENSHOT_DIR")) {
    w.resize(1280, 840);
    w.tabs()->setCurrentIndex(6);
    w.show();
    pause(50);
    w.grab().save(QDir(dir).filePath("run_tab.png"));
    w.tabs()->setCurrentIndex(0);
    pause(50);
    w.grab().save(QDir(dir).filePath("main.png"));
  }
}

void testRunPanelFailures() {
  Mock mock;
  Fixture f;
  RunPanel* run = f.w.runPanel();
  run->launchButton()->click();  // no workspace: uses the environment, i.e. the mock tools
  CHECK(waitFor([&]() { return mock.has("launched"); }));

  const auto& checks = run->checks();
  const int pub = firstRow(*run, false), cmd = firstRow(*run, true);
  CHECK(pub >= 0 && cmd >= 0);
  // Pick distinct published topics for the individual failures.
  std::vector<int> published;
  for (size_t i = 0; i < checks.size(); ++i) {
    if (checks[i].direction != gzb::Direction::RosToGz) published.push_back(static_cast<int>(i));
  }
  CHECK(published.size() >= 4);
  const int missing = published[0], wrongType = published[1], silent = published[2], fine = published[3];

  mock.healthy(checks);
  QString topics;
  for (size_t i = 0; i < checks.size(); ++i) {
    const int row = static_cast<int>(i);
    if (row == missing) continue;
    topics += QString("%1 [%2]\n").arg(QString::fromStdString(checks[i].rosTopic),
                                       row == wrongType ? QString("std_msgs/msg/String") : QString::fromStdString(checks[i].rosType));
  }
  mock.touch("topics", topics);
  QFile::remove(mock.path("hz/" + Mock::key(checks[static_cast<size_t>(silent)].rosTopic)));
  mock.touch("subs/" + Mock::key(checks[static_cast<size_t>(cmd)].rosTopic), "0");

  bool finished = false;
  QObject::connect(run, &RunPanel::checksFinished, [&]() { finished = true; });
  run->checkButton()->click();
  CHECK(waitFor([&]() { return finished; }, 60000));
  CHECK_EQ_Q(statusOf(*run, missing), QString("Fail"));
  CHECK(detailOf(*run, missing).contains("not in `ros2 topic list`"));
  CHECK_EQ_Q(statusOf(*run, wrongType), QString("Fail"));
  CHECK(detailOf(*run, wrongType).contains("std_msgs/msg/String instead of"));
  CHECK_EQ_Q(statusOf(*run, silent), QString("Fail"));
  CHECK(detailOf(*run, silent).contains("no messages arrived"));
  CHECK_EQ_Q(statusOf(*run, fine), QString("Pass"));
  CHECK_EQ_Q(statusOf(*run, cmd), QString("Fail"));
  CHECK(detailOf(*run, cmd).contains("Nothing in ROS is subscribed"));
  CHECK(run->summaryLabel()->text().contains("fail"));
  CHECK(run->log()->toPlainText().contains("FAIL:"));

  // Delivery to Gazebo: ROS publishes but Gazebo never sees it.
  mock.touch("subs/" + Mock::key(checks[static_cast<size_t>(cmd)].rosTopic), "1");
  mock.touch("gz_deaf", QString::fromStdString(checks[static_cast<size_t>(cmd)].gzTopic) + "\n");
  finished = false;
  run->checkButton()->click();
  CHECK(waitFor([&]() { return finished; }, 60000));
  CHECK_EQ_Q(statusOf(*run, cmd), QString("Fail"));
  CHECK(detailOf(*run, cmd).contains("nothing arrived on the Gazebo topic"));

  // `gz` missing: the bridge listens, which is all that can be said.
  QFile::remove(mock.path("gz_deaf"));
  mock.touch("gz_missing");
  finished = false;
  run->checkButton()->click();
  CHECK(waitFor([&]() { return finished; }, 60000));
  CHECK_EQ_Q(statusOf(*run, cmd), QString("Pass"));
  CHECK(detailOf(*run, cmd).contains("not verified"));

  // `ros2 topic pub` itself fails (bad message): reported as that, not as "nothing arrived".
  QFile::remove(mock.path("gz_missing"));
  QDir(mock.path("pub")).removeRecursively();
  mock.touch("pub_fail");
  mock.touch("gz_deaf", QString::fromStdString(checks[static_cast<size_t>(cmd)].gzTopic) + "\n");
  finished = false;
  run->checkButton()->click();
  CHECK(waitFor([&]() { return finished; }, 60000));
  CHECK_EQ_Q(statusOf(*run, cmd), QString("Fail"));
  CHECK(detailOf(*run, cmd).contains("Publishing the sample message failed"));
  CHECK(detailOf(*run, cmd).contains("message type is invalid"));

  run->stop();
  CHECK(waitFor([&]() { return !run->launched(); }));
}

void testRunPanelProblems() {
  // ---- build failure
  {
    Mock mock;
    Fixture f;
    RunPanel* run = f.w.runPanel();
    QTemporaryDir ws;
    run->setWorkspace(ws.path());
    mock.touch("build_fail");
    run->buildButton()->click();
    CHECK(waitFor([&]() { return !run->building(); }));
    CHECK(run->log()->toPlainText().contains("FAIL: colcon exited with code 1"));
    CHECK(!QFileInfo::exists(QDir(ws.path()).filePath("install/setup.bash")));
    run->launchButton()->click();
    CHECK(!run->launched());
  }
  // ---- launch that dies at once (package not found, ...)
  {
    Mock mock;
    Fixture f;
    RunPanel* run = f.w.runPanel();
    mock.touch("launch_fail");
    run->launchButton()->click();
    CHECK(waitFor([&]() { return !run->launched() && run->log()->toPlainText().contains("exited with code 1"); }));
    CHECK(run->log()->toPlainText().contains("Package 'diffbot_sim' not found"));
    CHECK(run->launchButton()->isEnabled());
  }
  // ---- no ROS tools at all
  {
    Mock mock;
    Fixture f;
    RunPanel* run = f.w.runPanel();
    const QByteArray oldPath = qgetenv("PATH");
    qputenv("PATH", "/usr/bin:/bin");
    bool finished = false;
    QObject::connect(run, &RunPanel::checksFinished, [&]() { finished = true; });
    run->checkButton()->click();
    CHECK(waitFor([&]() { return finished; }, 30000));
    for (size_t i = 0; i < run->results().size(); ++i) {
      CHECK(run->results()[i].state == gzb::CheckState::Fail);
      CHECK(QString::fromStdString(run->results()[i].detail).contains("`ros2` was not found"));
    }
    run->launchButton()->click();
    CHECK(waitFor([&]() { return !run->launched(); }, 10000));
    CHECK(run->log()->toPlainText().contains("`ros2` was not found"));
    qputenv("PATH", oldPath);
  }
  // ---- the workspace folder does not exist
  {
    Mock mock;
    Fixture f;
    RunPanel* run = f.w.runPanel();
    run->setWorkspace("/nonexistent/ws");
    run->buildButton()->click();
    CHECK(!run->building());
    CHECK(run->log()->toPlainText().contains("The workspace folder does not exist"));
  }
}

void testRunPanelAutoCheckAndCancel() {
  Mock mock;
  Fixture f;
  RunPanel* run = f.w.runPanel();
  run->autoCheckBox()->setChecked(true);
  run->settleSpin()->setValue(1);
  mock.healthy(run->checks());
  bool finished = false;
  QObject::connect(run, &RunPanel::checksFinished, [&]() { finished = true; });
  run->guiBox()->setChecked(false);
  CHECK(run->equivalentCommands().contains("gui:=false"));
  run->launchButton()->click();
  CHECK(run->findChild<QLabel*>("run_state")->text().contains("Checking the topics in"));
  CHECK(waitFor([&]() { return finished; }, 60000));  // started by itself
  CHECK(mock.text("launch_args").contains("gui:=false"));
  for (const gzb::CheckResult& r : run->results()) CHECK(r.state == gzb::CheckState::Pass);

  // Cancel in the middle of a run: finished rows keep their result, the rest is "Skipped".
  run->sampleSpin()->setValue(30);  // each hz would listen for 30 s
  finished = false;
  run->checkButton()->click();
  CHECK(run->checking());
  CHECK(waitFor([&]() {
    int running = 0;
    for (const gzb::CheckResult& r : run->results()) running += r.state == gzb::CheckState::Running;
    return running > 0;
  }));
  run->checkButton()->click();  // now "Cancel checks"
  CHECK(!run->checking());
  int skipped = 0;
  for (const gzb::CheckResult& r : run->results()) {
    CHECK(r.state != gzb::CheckState::Running && r.state != gzb::CheckState::Pending);
    skipped += r.state == gzb::CheckState::Skipped;
  }
  CHECK(skipped > 0);
  CHECK(run->log()->toPlainText().contains("Cancelled."));
  CHECK(run->checkButton()->text() == "Check topics");

  // The plan changes while Gazebo runs: the table stays on what was launched, and says so.
  const size_t rows = run->checks().size();
  f.w.field("package_name")->setText("other_pkg");
  f.w.refreshNow();
  CHECK(run->launched());
  CHECK(run->checks().size() == rows);
  CHECK(!run->findChild<QLabel*>("run_state")->text().contains("other_pkg"));
  CHECK(run->equivalentCommands().contains("other_pkg"));
  bool noteShown = false;
  for (QLabel* l : run->findChildren<QLabel*>()) noteShown = noteShown || (l->text().startsWith("The plan changed") && !l->isHidden());
  CHECK(noteShown);

  // Dropping a sensor changes the rows once nothing runs any more.
  f.w.field("package_name")->clear();
  f.w.setItemChecked("sensor:front_camera", false);
  f.w.refreshNow();
  CHECK(run->checks().size() == rows);  // still frozen
  run->stopButton()->click();
  CHECK(waitFor([&]() { return !run->launched(); }));
  CHECK(run->checks().size() < rows);
  CHECK(static_cast<int>(run->checks().size()) == run->table()->rowCount());
  noteShown = false;
  for (QLabel* l : run->findChildren<QLabel*>()) noteShown = noteShown || (l->text().startsWith("The plan changed") && !l->isHidden());
  CHECK(!noteShown);
}

void testWindowClose() {
  Mock mock;
  Fixture f;
  RunPanel* run = f.w.runPanel();
  run->launchButton()->click();
  CHECK(waitFor([&]() { return mock.has("launched"); }));
  const qint64 child = mock.text("child.pid").trimmed().toLongLong();
  CHECK(pidAlive(child));
  f.w.show();
  f.w.close();  // shuts the launch down before the window goes
  CHECK(!run->launched());
  CHECK(mock.has("launch_stopped"));
  CHECK(waitFor([&]() { return !pidAlive(child); }, 5000));
}

}  // namespace

int main(int argc, char** argv) {
  QTemporaryDir config;
  qputenv("XDG_CONFIG_HOME", config.path().toUtf8());
  qputenv("ROS_DISTRO", "jazzy");  // the helper does not look for /opt/ros itself
  qputenv("PATH", QString(QString(GZBH_MOCK_BIN) + ":" + QString::fromLocal8Bit(qgetenv("PATH"))).toLocal8Bit());
  qputenv("GZBH_MOCK_STATE", config.path().toUtf8());
  QApplication app(argc, argv);
  QApplication::setApplicationName("gz_bridge_helper_test");

  testHighlighter();
  testProcessJob();
  testWindowBasics();
  testSettingsFields();
  testYamlFiles();
  testRos2ControlWindow();
  testBadUrdfAndReload();
  testDropAndClipboard();
  testRunPanelHappyPath();
  testRunPanelFailures();
  testRunPanelProblems();
  testRunPanelAutoCheckAndCancel();
  testWindowClose();

  std::cout << g_checks << " checks, " << g_failed << " failed\n";
  return g_failed == 0 ? 0 : 1;
}
