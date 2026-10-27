// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include "main_window.hpp"

#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QClipboard>
#include <QComboBox>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFontDatabase>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QListWidget>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPalette>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRegularExpression>
#include <QSaveFile>
#include <QScrollArea>
#include <QScrollBar>
#include <QSettings>
#include <QSplitter>
#include <QStackedWidget>
#include <QStatusBar>
#include <QSyntaxHighlighter>
#include <QTabWidget>
#include <QTextCharFormat>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <functional>
#include <map>
#include <set>

#include "urdf_generator/builders.hpp"
#include "urdf_generator/emitter.hpp"
#include "urdf_generator/scene.hpp"

namespace urdfgui {
namespace {

// Colours chosen for both a light and a dark palette.
class XmlHighlighter : public QSyntaxHighlighter {
 public:
  explicit XmlHighlighter(QTextDocument* doc) : QSyntaxHighlighter(doc) {
    const bool dark = QGuiApplication::palette().color(QPalette::Base).lightness() < 128;
    tag_.setForeground(QColor(dark ? "#79b8ff" : "#0b5394"));
    tag_.setFontWeight(QFont::Bold);
    attr_.setForeground(QColor(dark ? "#ffab70" : "#8a4b00"));
    string_.setForeground(QColor(dark ? "#85e89d" : "#1a7f37"));
    expr_.setForeground(QColor(dark ? "#d2a8ff" : "#8250df"));
    comment_.setForeground(QColor(dark ? "#8b949e" : "#6a737d"));
    comment_.setFontItalic(true);
  }

 protected:
  void highlightBlock(const QString& text) override {
    int pos = 0;
    setCurrentBlockState(0);
    if (previousBlockState() == 1) {  // inside a comment that started on an earlier line
      const int end = text.indexOf("-->");
      if (end < 0) {
        setFormat(0, text.length(), comment_);
        setCurrentBlockState(1);
        return;
      }
      setFormat(0, end + 3, comment_);
      pos = end + 3;
    }
    while (pos < text.length()) {
      const int open = text.indexOf("<!--", pos);
      const int segEnd = open < 0 ? text.length() : open;
      highlightMarkup(text, pos, segEnd);
      if (open < 0) break;
      const int close = text.indexOf("-->", open + 4);
      if (close < 0) {
        setFormat(open, text.length() - open, comment_);
        setCurrentBlockState(1);
        return;
      }
      setFormat(open, close + 3 - open, comment_);
      pos = close + 3;
    }
  }

 private:
  void apply(const QString& text, int from, int to, const QRegularExpression& re,
             const QTextCharFormat& fmt) {
    auto it = re.globalMatch(text, from);
    while (it.hasNext()) {
      const auto m = it.next();
      if (m.capturedStart() >= to) break;
      setFormat(m.capturedStart(), m.capturedLength(), fmt);
    }
  }

  void highlightMarkup(const QString& text, int from, int to) {
    static const QRegularExpression tags(QStringLiteral("</?[A-Za-z_:][\\w:.\\-]*|/?>|<\\?|\\?>"));
    static const QRegularExpression attrs(QStringLiteral("[A-Za-z_:][\\w:.\\-]*(?==\")"));
    static const QRegularExpression strings(QStringLiteral("\"[^\"]*\""));
    static const QRegularExpression exprs(QStringLiteral("\\$\\{[^}]*\\}"));
    apply(text, from, to, tags, tag_);
    apply(text, from, to, attrs, attr_);
    apply(text, from, to, strings, string_);
    apply(text, from, to, exprs, expr_);
  }

  QTextCharFormat tag_, attr_, string_, expr_, comment_;
};

QPlainTextEdit* makePreview() {
  auto* e = new QPlainTextEdit;
  e->setReadOnly(true);
  e->setLineWrapMode(QPlainTextEdit::NoWrap);
  e->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  e->setTabStopDistance(2 * e->fontMetrics().horizontalAdvance(' '));
  new XmlHighlighter(e->document());
  return e;
}

QString jointTypeLabel(urdfgen::JointType t) {
  switch (t) {
    case urdfgen::JointType::Fixed: return "fixed";
    case urdfgen::JointType::Revolute: return "revolute";
    case urdfgen::JointType::Continuous: return "continuous";
    case urdfgen::JointType::Prismatic: return "prismatic";
  }
  return QString();
}

}  // namespace

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
  setWindowTitle("URDF Generator");
  buildUi();

  timer_.setSingleShot(true);
  timer_.setInterval(120);
  connect(&timer_, &QTimer::timeout, this, &MainWindow::refreshNow);

  onTypeChanged(0);
}

void MainWindow::buildUi() {
  // ------------------------------------------------------------- left: inputs
  auto* left = new QWidget;
  auto* lv = new QVBoxLayout(left);
  lv->addWidget(new QLabel("<b>Robot type</b>"));
  typeBox_ = new QComboBox;
  typeBox_->setObjectName("robot_type");
  lv->addWidget(typeBox_);
  hint_ = new QLabel;
  hint_->setWordWrap(true);
  lv->addWidget(hint_);

  stack_ = new QStackedWidget;
  for (urdfgen::RobotType t : urdfgen::allRobotTypes()) {
    ConfigPage* page = createPage(t);
    pages_.push_back(page);
    typeBox_->addItem(QString::fromStdString(urdfgen::robotTypeLabel(t)), static_cast<int>(t));
    auto* scroll = new QScrollArea;
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setWidgetResizable(true);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setWidget(page);
    stack_->addWidget(scroll);
    connect(page, &ConfigPage::changed, this, &MainWindow::scheduleRefresh);
  }
  lv->addWidget(stack_, 1);
  left->setMinimumWidth(400);
  connect(typeBox_, &QComboBox::currentIndexChanged, this, &MainWindow::onTypeChanged);

  // ------------------------------------------------------------ right: output
  auto* bottom = new QWidget;
  auto* rv = new QVBoxLayout(bottom);
  rv->setContentsMargins(0, 0, 0, 0);
  xacroView_ = makePreview();
  xacroView_->setObjectName("xacro_view");
  urdfView_ = makePreview();
  urdfView_->setObjectName("urdf_view");
  tree_ = new QTreeWidget;
  tree_->setObjectName("structure_tree");
  tree_->setColumnCount(4);
  tree_->setHeaderLabels({"Link", "Joint to parent", "Type", "Offset xyz [m]"});
  tree_->header()->setSectionResizeMode(QHeaderView::ResizeToContents);
  tabs_ = new QTabWidget;
  tabs_->addTab(xacroView_, "Xacro");
  tabs_->addTab(urdfView_, "URDF");
  tabs_->addTab(tree_, "Structure");

  stale_ = new QLabel("The preview shows the last valid output. Fix the errors below to update it.");
  stale_->setWordWrap(true);
  stale_->setVisible(false);
  summary_ = new QLabel;
  issues_ = new QListWidget;
  issues_->setObjectName("issues");
  issues_->setMaximumHeight(110);
  issues_->setSelectionMode(QAbstractItemView::NoSelection);
  issues_->setVisible(false);

  auto* buttons = new QHBoxLayout;
  copyButton_ = new QPushButton("Copy to clipboard");
  saveXacroButton_ = new QPushButton("Save xacro...");
  saveUrdfButton_ = new QPushButton("Save URDF...");
  saveBothButton_ = new QPushButton("Save both...");
  buttons->addWidget(copyButton_);
  buttons->addStretch(1);
  buttons->addWidget(saveXacroButton_);
  buttons->addWidget(saveUrdfButton_);
  buttons->addWidget(saveBothButton_);
  connect(copyButton_, &QPushButton::clicked, this, &MainWindow::copyCurrent);
  connect(saveXacroButton_, &QPushButton::clicked, this, &MainWindow::saveXacro);
  connect(saveUrdfButton_, &QPushButton::clicked, this, &MainWindow::saveUrdf);
  connect(saveBothButton_, &QPushButton::clicked, this, &MainWindow::saveBoth);

  rv->addWidget(tabs_, 1);
  rv->addWidget(stale_);
  rv->addWidget(summary_);
  rv->addWidget(issues_);
  rv->addLayout(buttons);

  // 3D view above the text previews; drag the handle between them to give either more room.
  auto* viewPane = new QWidget;
  auto* vv = new QVBoxLayout(viewPane);
  vv->setContentsMargins(0, 0, 0, 0);
  view_ = new RobotView;
  view_->setObjectName("robot_view");
  auto* bar = new QHBoxLayout;
  const std::pair<const char*, RobotView::Preset> presets[] = {{"Isometric", RobotView::Preset::Isometric},
                                                               {"Front", RobotView::Preset::Front},
                                                               {"Side", RobotView::Preset::Side},
                                                               {"Top", RobotView::Preset::Top}};
  bar->addWidget(new QLabel("<b>3D view</b>"));
  for (const auto& pr : presets) {
    auto* b = new QPushButton(pr.first);
    b->setAutoDefault(false);
    b->setToolTip("Re-aim the camera and fit the whole robot");
    const RobotView::Preset preset = pr.second;
    connect(b, &QPushButton::clicked, this, [this, preset]() { view_->setPreset(preset); });
    bar->addWidget(b);
  }
  bar->addStretch(1);
  framesBox_ = new QCheckBox("Link frames");
  framesBox_->setObjectName("show_frames");
  framesBox_->setToolTip("Draw the x (red), y (green) and z (blue) axes of every link frame as thin lines");
  connect(framesBox_, &QCheckBox::toggled, view_, &RobotView::setShowFrames);
  bar->addWidget(framesBox_);
  sensorAxesBox_ = new QCheckBox("Sensor axes");
  sensorAxesBox_->setObjectName("show_sensor_axes");
  sensorAxesBox_->setChecked(true);
  sensorAxesBox_->setToolTip(
      "Draw the frame of every sensor (x forward, y left, z up) as thick arrows with letters and a name tag, "\
      "and a dashed line to the root of the link it is mounted on");
  connect(sensorAxesBox_, &QCheckBox::toggled, view_, &RobotView::setShowSensorAxes);
  bar->addWidget(sensorAxesBox_);
  auto* viewHint = new QLabel("Drag to orbit \u00B7 right-drag to pan \u00B7 wheel to zoom \u00B7 double-click to reset \u00B7 sliders pose the joints");
  QPalette hintPalette = viewHint->palette();
  hintPalette.setColor(QPalette::WindowText, hintPalette.color(QPalette::PlaceholderText));
  viewHint->setPalette(hintPalette);
  jointsBox_ = new QCheckBox("Joint sliders");
  jointsBox_->setObjectName("show_joints");
  jointsBox_->setChecked(true);
  jointsBox_->setToolTip("Show a slider for every movable joint to pose the robot");
  bar->addWidget(jointsBox_);

  joints_ = new JointPanel;
  connect(joints_, &JointPanel::poseChanged, this, &MainWindow::onPoseChanged);
  connect(jointsBox_, &QCheckBox::toggled, joints_, &QWidget::setVisible);

  auto* viewSplit = new QSplitter(Qt::Horizontal);
  viewSplit->addWidget(view_);
  viewSplit->addWidget(joints_);
  viewSplit->setStretchFactor(0, 1);
  viewSplit->setStretchFactor(1, 0);
  viewSplit->setSizes({480, 270});
  viewSplit->setChildrenCollapsible(false);
  vv->addLayout(bar);
  vv->addWidget(viewSplit, 1);
  vv->addWidget(viewHint);

  auto* right = new QSplitter(Qt::Vertical);
  right->addWidget(viewPane);
  right->addWidget(bottom);
  right->setStretchFactor(0, 1);
  right->setStretchFactor(1, 1);
  right->setSizes({400, 400});
  right->setChildrenCollapsible(false);

  auto* split = new QSplitter(Qt::Horizontal);
  split->addWidget(left);
  split->addWidget(right);
  split->setStretchFactor(0, 0);
  split->setStretchFactor(1, 1);
  split->setSizes({480, 720});
  setCentralWidget(split);
  resize(1240, 800);

  // -------------------------------------------------------------------- menu
  QMenu* file = menuBar()->addMenu("&File");
  auto makeAction = [this](const QString& text, const QKeySequence& key, void (MainWindow::*slot)()) {
    auto* a = new QAction(text, this);
    if (!key.isEmpty()) a->setShortcut(key);
    connect(a, &QAction::triggered, this, slot);
    return a;
  };
  saveXacroAct_ = makeAction("Save &xacro...", QKeySequence::Save, &MainWindow::saveXacro);
  saveUrdfAct_ = makeAction("Save &URDF...", QKeySequence(), &MainWindow::saveUrdf);
  saveBothAct_ = makeAction("Save &both to folder...", QKeySequence("Ctrl+Shift+S"), &MainWindow::saveBoth);
  copyAct_ = makeAction("&Copy current preview", QKeySequence(), &MainWindow::copyCurrent);
  file->addAction(saveXacroAct_);
  file->addAction(saveUrdfAct_);
  file->addAction(saveBothAct_);
  file->addSeparator();
  file->addAction(copyAct_);
  file->addSeparator();
  auto* quit = new QAction("&Quit", this);
  quit->setShortcut(QKeySequence::Quit);
  connect(quit, &QAction::triggered, this, &QWidget::close);
  file->addAction(quit);

  statusBar()->showMessage("Edit any value on the left; the xacro and URDF update as you type.");
}

ConfigPage* MainWindow::currentPage() const {
  const int i = typeBox_->currentIndex();
  return i >= 0 && i < static_cast<int>(pages_.size()) ? pages_[static_cast<std::size_t>(i)] : nullptr;
}

void MainWindow::selectType(urdfgen::RobotType type) {
  for (int i = 0; i < typeBox_->count(); ++i) {
    if (typeBox_->itemData(i).toInt() == static_cast<int>(type)) {
      typeBox_->setCurrentIndex(i);
      return;
    }
  }
}

void MainWindow::onTypeChanged(int index) {
  if (index < 0 || index >= static_cast<int>(pages_.size())) return;
  stack_->setCurrentIndex(index);
  hint_->setText(pages_[static_cast<std::size_t>(index)]->hint());
  fitNext_ = true;  // frame the new robot in the 3D view
  joints_->clear();  // a different robot: start from the rest pose
  lastRobot_.reset();
  xacro_.clear();  // never show another robot's output as "last valid"
  urdf_.clear();
  refreshNow();
}

void MainWindow::scheduleRefresh() { timer_.start(); }

int MainWindow::issueCount() const { return issues_->count(); }

void MainWindow::refreshNow() {
  timer_.stop();
  ConfigPage* page = currentPage();
  if (!page) return;

  std::vector<urdfgen::Issue> issues;
  try {
    const urdfgen::Robot robot = page->build();
    issues = urdfgen::validate(robot);
    if (!urdfgen::hasErrors(issues)) {
      xacro_ = QString::fromStdString(urdfgen::emitXacro(robot));
      urdf_ = QString::fromStdString(urdfgen::emitUrdf(robot));
      robotName_ = QString::fromStdString(robot.name);
      fillTree(robot);
      lastRobot_ = robot;
      joints_->setJoints(urdfgen::movableJoints(robot));
      view_->setScene(urdfgen::buildScene(robot, joints_->pose()), fitNext_);
      fitNext_ = false;

      const urdfgen::Part flat = robot.flatten();
      double mass = 0;
      for (const auto& l : flat.links) {
        if (l.hasBody) mass += l.mass.value();
      }
      summary_->setText(QString("%1 links, %2 joints, total mass %3 kg")
                            .arg(flat.links.size())
                            .arg(flat.joints.size())
                            .arg(mass, 0, 'g', 4));
    }
  } catch (const std::exception& e) {
    issues.push_back({urdfgen::Issue::Severity::Error, std::string("Internal error: ") + e.what()});
  }

  hasErrors_ = urdfgen::hasErrors(issues);
  if (hasErrors_) summary_->setText("Cannot generate: fix the errors below.");
  setPreview(xacroView_, xacro_);
  setPreview(urdfView_, urdf_);
  stale_->setVisible(hasErrors_ && !urdf_.isEmpty());
  showIssues(issues);
  updateButtons();
}

void MainWindow::onPoseChanged() {
  if (!lastRobot_) return;
  view_->setScene(urdfgen::buildScene(*lastRobot_, joints_->pose()), false, true);
}

void MainWindow::setPreview(QPlainTextEdit* view, const QString& text) {
  if (view->toPlainText() == text) return;
  const int scroll = view->verticalScrollBar()->value();
  const int hscroll = view->horizontalScrollBar()->value();
  view->setPlainText(text);
  view->verticalScrollBar()->setValue(scroll);
  view->horizontalScrollBar()->setValue(hscroll);
}

void MainWindow::showIssues(const std::vector<urdfgen::Issue>& issues) {
  issues_->clear();
  const bool dark = QGuiApplication::palette().color(QPalette::Base).lightness() < 128;
  for (const auto& i : issues) {
    const bool isError = i.severity == urdfgen::Issue::Severity::Error;
    auto* item = new QListWidgetItem((isError ? "Error: " : "Warning: ") + QString::fromStdString(i.message));
    item->setForeground(QColor(isError ? (dark ? "#ff7b72" : "#b42318") : (dark ? "#e3b341" : "#8a5a00")));
    item->setToolTip(item->text());
    issues_->addItem(item);
  }
  issues_->setVisible(!issues.empty());
}

void MainWindow::fillTree(const urdfgen::Robot& robot) {
  tree_->clear();
  const urdfgen::Part part = robot.flatten();
  std::map<std::string, std::vector<const urdfgen::Joint*>> kids;
  std::set<std::string> children;
  for (const auto& j : part.joints) {
    kids[j.parent].push_back(&j);
    children.insert(j.child);
  }
  std::set<std::string> seen;
  std::function<void(QTreeWidgetItem*, const std::string&, const urdfgen::Joint*)> add;
  add = [&](QTreeWidgetItem* parentItem, const std::string& link, const urdfgen::Joint* via) {
    if (!seen.insert(link).second) return;
    auto* item = parentItem ? new QTreeWidgetItem(parentItem) : new QTreeWidgetItem(tree_);
    item->setText(0, QString::fromStdString(link));
    if (via) {
      item->setText(1, QString::fromStdString(via->name));
      item->setText(2, jointTypeLabel(via->type));
      item->setText(3, QString::fromStdString(via->origin.x.urdf() + " " + via->origin.y.urdf() + " " +
                                              via->origin.z.urdf()));
    }
    for (const urdfgen::Joint* j : kids[link]) add(item, j->child, j);
  };
  for (const auto& l : part.links) {
    if (!children.count(l.name)) add(nullptr, l.name, nullptr);
  }
  tree_->expandAll();
}

void MainWindow::updateButtons() {
  const bool ok = canSave();
  for (QWidget* w : {static_cast<QWidget*>(copyButton_), static_cast<QWidget*>(saveXacroButton_),
                     static_cast<QWidget*>(saveUrdfButton_), static_cast<QWidget*>(saveBothButton_)}) {
    w->setEnabled(ok);
  }
  for (QAction* a : {saveXacroAct_, saveUrdfAct_, saveBothAct_, copyAct_}) a->setEnabled(ok);
}

// ------------------------------------------------------------------- saving

QString MainWindow::defaultPath(const QString& extension) const {
  QSettings settings("urdf_generator", "urdf_generator");
  const QString dir = settings.value("lastDir", QDir::homePath()).toString();
  return QDir(dir).filePath(robotName_ + extension);
}

void MainWindow::rememberDir(const QString& path) {
  QSettings settings("urdf_generator", "urdf_generator");
  settings.setValue("lastDir", QFileInfo(path).isDir() ? path : QFileInfo(path).absolutePath());
}

bool MainWindow::writeText(const QString& path, const QString& text, QString* error) {
  QSaveFile f(path);  // writes to a temporary file and renames, so a failure never leaves half a file
  if (!f.open(QIODevice::WriteOnly)) {
    if (error) *error = QString("Cannot write %1: %2").arg(path, f.errorString());
    return false;
  }
  f.write(text.toUtf8());
  if (!f.commit()) {
    if (error) *error = QString("Cannot write %1: %2").arg(path, f.errorString());
    return false;
  }
  return true;
}

bool MainWindow::saveBothTo(const QString& dir, QString* error) {
  if (!canSave()) {
    if (error) *error = "There are errors in the configuration.";
    return false;
  }
  const QDir d(dir);
  const QString xacroPath = d.filePath(robotName_ + ".xacro");
  const QString urdfPath = d.filePath(robotName_ + ".urdf");
  return writeText(xacroPath, xacro_, error) && writeText(urdfPath, urdf_, error);
}

void MainWindow::saveSingle(const QString& title, const QString& filter, const QString& extension,
                            const QString& text) {
  if (!canSave()) return;
  QString path = QFileDialog::getSaveFileName(this, title, defaultPath(extension), filter);
  if (path.isEmpty()) return;
  if (QFileInfo(path).suffix().isEmpty()) path += extension;
  QString error;
  if (!writeText(path, text, &error)) {
    QMessageBox::critical(this, "Save failed", error);
    return;
  }
  rememberDir(path);
  statusBar()->showMessage("Wrote " + path, 8000);
}

void MainWindow::saveXacro() {
  saveSingle("Save xacro", "Xacro files (*.xacro);;All files (*)", ".xacro", xacro_);
}

void MainWindow::saveUrdf() {
  saveSingle("Save URDF", "URDF files (*.urdf);;All files (*)", ".urdf", urdf_);
}

void MainWindow::saveBoth() {
  if (!canSave()) return;
  const QString dir = QFileDialog::getExistingDirectory(this, "Save <name>.xacro and <name>.urdf into",
                                                        QFileInfo(defaultPath("")).absolutePath());
  if (dir.isEmpty()) return;
  const QDir d(dir);
  QStringList existing;
  for (const QString& ext : {QString(".xacro"), QString(".urdf")}) {
    if (QFileInfo::exists(d.filePath(robotName_ + ext))) existing << robotName_ + ext;
  }
  if (!existing.isEmpty() &&
      QMessageBox::question(this, "Overwrite?",
                            "These files already exist and will be replaced:\n" + existing.join("\n")) !=
          QMessageBox::Yes) {
    return;
  }
  QString error;
  if (!saveBothTo(dir, &error)) {
    QMessageBox::critical(this, "Save failed", error);
    return;
  }
  rememberDir(dir);
  statusBar()->showMessage(QString("Wrote %1.xacro and %1.urdf into %2").arg(robotName_, dir), 8000);
}

void MainWindow::copyCurrent() {
  if (!canSave()) return;
  const int tab = tabs_->currentIndex();
  const QString text = tab == 0 ? xacro_ : urdf_;
  if (tab > 1) {  // the structure tab has nothing to copy; copy the URDF instead
    QGuiApplication::clipboard()->setText(urdf_);
    statusBar()->showMessage("Copied the URDF", 4000);
    return;
  }
  QGuiApplication::clipboard()->setText(text);
  statusBar()->showMessage(tab == 0 ? "Copied the xacro" : "Copied the URDF", 4000);
}

}  // namespace urdfgui
