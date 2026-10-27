// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include "main_window.hpp"

#include <QAction>
#include <QApplication>
#include <QClipboard>
#include <QComboBox>
#include <QDir>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QGuiApplication>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QMimeData>
#include <QPalette>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSaveFile>
#include <QScrollArea>
#include <QSettings>
#include <QSplitter>
#include <QStatusBar>
#include <QTabWidget>
#include <QTableWidget>
#include <QTreeWidget>
#include <QUrl>
#include <QVBoxLayout>
#include <map>
#include <optional>
#include <set>

#include "gz_bridge_helper/common.hpp"
#include "gz_bridge_helper/verify.hpp"
#include "highlighter.hpp"
#include "run_panel.hpp"

namespace gzgui {
namespace {

QString q(const std::string& s) { return QString::fromStdString(s); }

bool darkTheme() { return QGuiApplication::palette().color(QPalette::Base).lightness() < 128; }

QColor severityColor(gzb::Severity s) {
  const bool dark = darkTheme();
  switch (s) {
    case gzb::Severity::Error: return QColor(dark ? "#ff7b72" : "#b42318");
    case gzb::Severity::Warning: return QColor(dark ? "#e3b341" : "#8a5a00");
    case gzb::Severity::Info: break;
  }
  return QGuiApplication::palette().color(QPalette::PlaceholderText);
}

/// A faint background for each kind of bridged topic (works on light and dark themes).
QColor groupColor(const std::string& group) {
  QColor c(128, 128, 128);
  if (group == "sensor") c = QColor(59, 130, 246);
  else if (group == "drive") c = QColor(34, 197, 94);
  else if (group == "joint") c = QColor(249, 115, 22);
  else if (group == "odometry") c = QColor(168, 85, 247);
  else if (group == "user") c = QColor(236, 72, 153);
  c.setAlpha(darkTheme() ? 55 : 40);
  return c;
}

QString directionArrow(gzb::Direction d) {
  switch (d) {
    case gzb::Direction::GzToRos: return "←";      // ROS <- Gazebo
    case gzb::Direction::RosToGz: return "→";      // ROS -> Gazebo
    case gzb::Direction::Bidirectional: return "↔";
  }
  return QString();
}

QString directionWords(gzb::Direction d) {
  switch (d) {
    case gzb::Direction::GzToRos: return "Gazebo publishes, the bridge republishes it in ROS";
    case gzb::Direction::RosToGz: return "ROS publishes, the bridge republishes it in Gazebo";
    case gzb::Direction::Bidirectional: return "both ways";
  }
  return QString();
}

QPalette hintPalette(const QWidget* w) {
  QPalette p = w->palette();
  p.setColor(QPalette::WindowText, p.color(QPalette::PlaceholderText));
  return p;
}

const char* const kLastDirKey = "lastDir";

}  // namespace

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
  setWindowTitle("Gazebo Bridge Helper");
  setAcceptDrops(true);
  buildUi();

  timer_.setSingleShot(true);
  timer_.setInterval(120);
  connect(&timer_, &QTimer::timeout, this, &MainWindow::refreshNow);
  refreshNow();
}

void MainWindow::buildUi() {
  // ------------------------------------------------------------- left: inputs
  auto* left = new QWidget;
  auto* lv = new QVBoxLayout(left);

  hint_ = new QLabel(
      "Open a URDF, optionally add YAML files, then tick the sensors and drive the helper suggests. "
      "Your URDF is never changed: the additions go into a copy inside the generated package.");
  hint_->setWordWrap(true);
  hint_->setPalette(hintPalette(hint_));
  lv->addWidget(hint_);

  auto* urdfBox = new QGroupBox("URDF");
  auto* uv = new QVBoxLayout(urdfBox);
  auto* ur = new QHBoxLayout;
  urdfPath_ = new QLineEdit;
  urdfPath_->setObjectName("urdf_path");
  urdfPath_->setReadOnly(true);
  urdfPath_->setPlaceholderText("no file opened (File > Open URDF, or drop one here)");
  auto* openButton = new QPushButton("Open...");
  openButton->setObjectName("open_urdf");
  openButton->setAutoDefault(false);
  reloadButton_ = new QPushButton("Reload");
  reloadButton_->setAutoDefault(false);
  reloadButton_->setToolTip("Read the URDF and the YAML files again (F5). The ticks you set are kept.");
  ur->addWidget(urdfPath_, 1);
  ur->addWidget(openButton);
  ur->addWidget(reloadButton_);
  uv->addLayout(ur);
  urdfSummary_ = new QLabel;
  urdfSummary_->setObjectName("urdf_summary");
  urdfSummary_->setWordWrap(true);
  uv->addWidget(urdfSummary_);
  lv->addWidget(urdfBox);
  connect(openButton, &QPushButton::clicked, this, &MainWindow::chooseUrdf);
  connect(reloadButton_, &QPushButton::clicked, this, &MainWindow::reloadAll);

  auto* yamlBox = new QGroupBox("YAML files (optional)");
  auto* yv = new QVBoxLayout(yamlBox);
  yamlList_ = new QListWidget;
  yamlList_->setObjectName("yaml_list");
  yamlList_->setMaximumHeight(96);
  yamlList_->setToolTip("ros2_control controllers, an existing ros_gz_bridge config, or your overrides: told apart by what is inside");
  yv->addWidget(yamlList_);
  auto* yr = new QHBoxLayout;
  auto* addYamlButton = new QPushButton("Add...");
  addYamlButton->setObjectName("add_yaml");
  removeYamlButton_ = new QPushButton("Remove");
  removeYamlButton_->setObjectName("remove_yaml");
  auto* templateButton = new QPushButton("Save overrides template...");
  templateButton->setToolTip("Writes a commented example of the overrides format; edit it and add it here");
  for (QPushButton* b : {addYamlButton, removeYamlButton_, templateButton}) b->setAutoDefault(false);
  yr->addWidget(addYamlButton);
  yr->addWidget(removeYamlButton_);
  yr->addStretch(1);
  yr->addWidget(templateButton);
  yv->addLayout(yr);
  lv->addWidget(yamlBox);
  connect(addYamlButton, &QPushButton::clicked, this, &MainWindow::chooseYaml);
  connect(removeYamlButton_, &QPushButton::clicked, this, &MainWindow::removeSelectedYaml);
  connect(templateButton, &QPushButton::clicked, this, &MainWindow::saveTemplate);
  connect(yamlList_, &QListWidget::currentRowChanged, this, [this]() { updateButtons(); });

  auto* namesBox = new QGroupBox("Names and spawn pose");
  namesBox->setToolTip("Empty fields use the default shown in grey. Values typed here win over every YAML file.");
  auto* form = new QFormLayout(namesBox);
  auto makeField = [this, form](const char* object, const QString& label, const QString& tip) {
    auto* e = new QLineEdit;
    e->setObjectName(object);
    e->setToolTip(tip);
    e->setProperty("tip", tip);
    e->setClearButtonEnabled(true);
    form->addRow(label, e);
    connect(e, &QLineEdit::textChanged, this, &MainWindow::onSettingsEdited);
    return e;
  };
  package_ = makeField("package_name", "Package name", "Name of the generated ROS 2 package (lowercase letters, digits and _)");
  model_ = makeField("model_name", "Model name", "Name of the model inside Gazebo; it is part of the Gazebo topic names");
  world_ = makeField("world_name", "World name", "Name of the generated world");
  spawnX_ = makeField("spawn_x", "Spawn x [m]", "Where the robot appears in the world");
  spawnY_ = makeField("spawn_y", "Spawn y [m]", "Where the robot appears in the world");
  spawnZ_ = makeField("spawn_z", "Spawn z [m]", "Height of the robot's origin; by default computed so the lowest shape touches the ground");
  spawnYaw_ = makeField("spawn_yaw", "Spawn yaw [rad]", "Heading of the robot at the start");
  lv->addWidget(namesBox);

  auto* itemsBox = new QGroupBox("Detected (tick what to add)");
  auto* iv = new QVBoxLayout(itemsBox);
  modeLabel_ = new QLabel;
  modeLabel_->setObjectName("mode_label");
  modeLabel_->setWordWrap(true);
  iv->addWidget(modeLabel_);
  items_ = new QTreeWidget;
  items_->setObjectName("items_tree");
  items_->setColumnCount(2);
  items_->setHeaderLabels({"Item", ""});
  items_->header()->setStretchLastSection(false);
  items_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
  items_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  items_->setRootIsDecorated(true);
  items_->setMinimumHeight(200);
  iv->addWidget(items_, 1);
  itemDetail_ = new QLabel;
  itemDetail_->setObjectName("item_detail");
  itemDetail_->setWordWrap(true);
  itemDetail_->setMinimumHeight(60);
  itemDetail_->setAlignment(Qt::AlignTop | Qt::AlignLeft);
  iv->addWidget(itemDetail_);
  auto* resetButton = new QPushButton("Reset to suggestions");
  resetButton->setAutoDefault(false);
  iv->addWidget(resetButton, 0, Qt::AlignLeft);
  lv->addWidget(itemsBox, 1);
  connect(items_, &QTreeWidget::itemChanged, this, &MainWindow::onItemChanged);
  connect(items_, &QTreeWidget::currentItemChanged, this, [this](QTreeWidgetItem* cur) {
    itemDetail_->setText(cur ? cur->toolTip(0) : QString());
  });
  connect(resetButton, &QPushButton::clicked, this, &MainWindow::resetSuggestions);

  auto* leftScroll = new QScrollArea;
  leftScroll->setFrameShape(QFrame::NoFrame);
  leftScroll->setWidgetResizable(true);
  leftScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  leftScroll->setWidget(left);
  leftScroll->setMinimumWidth(400);

  // ------------------------------------------------------------ right: bridge table
  auto* tablePane = new QWidget;
  auto* tv = new QVBoxLayout(tablePane);
  tv->setContentsMargins(0, 0, 0, 0);
  auto* bar = new QHBoxLayout;
  bar->addWidget(new QLabel("<b>Bridged topics</b>"));
  bridgeSummary_ = new QLabel;
  bridgeSummary_->setObjectName("bridge_summary");
  bar->addWidget(bridgeSummary_);
  bar->addStretch(1);
  auto* legend = new QLabel("← Gazebo to ROS · → ROS to Gazebo · ↔ both");
  legend->setPalette(hintPalette(legend));
  bar->addWidget(legend);
  tv->addLayout(bar);
  table_ = new QTableWidget;
  table_->setObjectName("bridge_table");
  table_->setColumnCount(7);
  table_->setHorizontalHeaderLabels({"ROS topic", "", "Gazebo topic", "What it carries", "ROS type", "Gazebo type", "From"});
  table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setStretchLastSection(true);
  table_->verticalHeader()->setVisible(false);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  table_->setAlternatingRowColors(false);
  table_->setWordWrap(false);
  tv->addWidget(table_, 1);

  // ------------------------------------------------------------ right: files, run, issues
  auto* bottom = new QWidget;
  auto* rv = new QVBoxLayout(bottom);
  rv->setContentsMargins(0, 0, 0, 0);
  bridgeView_ = makePreview(Language::Yaml);
  bridgeView_->setObjectName("bridge_yaml_view");
  launchView_ = makePreview(Language::Python);
  launchView_->setObjectName("launch_view");
  worldView_ = makePreview(Language::Xml);
  worldView_->setObjectName("world_view");
  changesView_ = makePreview(Language::Xml);
  changesView_->setObjectName("urdf_changes_view");
  urdfView_ = makePreview(Language::Xml);
  urdfView_->setObjectName("urdf_view");

  auto* filesPage = new QWidget;
  auto* fv = new QVBoxLayout(filesPage);
  fv->setContentsMargins(0, 0, 0, 0);
  fileBox_ = new QComboBox;
  fileBox_->setObjectName("package_file_box");
  fileView_ = makePreview(Language::Plain);
  fileView_->setObjectName("package_file_view");
  fv->addWidget(fileBox_);
  fv->addWidget(fileView_, 1);
  connect(fileBox_, &QComboBox::currentIndexChanged, this, [this](int index) {
    if (index < 0 || index >= static_cast<int>(last_.otherFiles.size())) {
      setPreviewText(fileView_, QString());
      return;
    }
    const auto& f = last_.otherFiles[static_cast<size_t>(index)];
    if (auto* h = fileView_->document()->findChild<CodeHighlighter*>()) h->setLanguage(languageForFile(f.first));
    setPreviewText(fileView_, f.second);
  });

  run_ = new RunPanel;
  run_->setObjectName("run_panel");
  connect(run_, &RunPanel::statusMessage, this, [this](const QString& m) { statusBar()->showMessage(m, 8000); });

  tabs_ = new QTabWidget;
  tabs_->setObjectName("tabs");
  tabs_->addTab(bridgeView_, "Bridge YAML");
  tabs_->addTab(launchView_, "Launch");
  tabs_->addTab(worldView_, "World");
  tabs_->addTab(changesView_, "URDF changes");
  tabs_->addTab(urdfView_, "URDF");
  tabs_->addTab(filesPage, "Package files");
  tabs_->addTab(run_, "Run");
  tabs_->setToolTip("");

  stale_ = new QLabel("The previews show the last valid output. Fix the errors below to update them.");
  stale_->setObjectName("stale_label");
  stale_->setWordWrap(true);
  stale_->setVisible(false);
  summary_ = new QLabel;
  summary_->setObjectName("summary_label");
  issues_ = new QListWidget;
  issues_->setObjectName("issues");
  issues_->setMaximumHeight(110);
  issues_->setSelectionMode(QAbstractItemView::NoSelection);
  issues_->setVisible(false);

  auto* buttons = new QHBoxLayout;
  copyButton_ = new QPushButton("Copy to clipboard");
  saveYamlButton_ = new QPushButton("Save bridge YAML...");
  savePackageButton_ = new QPushButton("Save package...");
  savePackageButton_->setObjectName("save_package");
  for (QPushButton* b : {copyButton_, saveYamlButton_, savePackageButton_}) b->setAutoDefault(false);
  buttons->addWidget(copyButton_);
  buttons->addStretch(1);
  buttons->addWidget(saveYamlButton_);
  buttons->addWidget(savePackageButton_);
  connect(copyButton_, &QPushButton::clicked, this, &MainWindow::copyCurrent);
  connect(saveYamlButton_, &QPushButton::clicked, this, &MainWindow::saveBridgeYaml);
  connect(savePackageButton_, &QPushButton::clicked, this, &MainWindow::savePackage);

  rv->addWidget(tabs_, 1);
  rv->addWidget(stale_);
  rv->addWidget(summary_);
  rv->addWidget(issues_);
  rv->addLayout(buttons);

  auto* right = new QSplitter(Qt::Vertical);
  right->addWidget(tablePane);
  right->addWidget(bottom);
  right->setStretchFactor(0, 1);
  right->setStretchFactor(1, 1);
  right->setSizes({260, 540});
  right->setChildrenCollapsible(false);

  auto* split = new QSplitter(Qt::Horizontal);
  split->addWidget(leftScroll);
  split->addWidget(right);
  split->setStretchFactor(0, 0);
  split->setStretchFactor(1, 1);
  split->setSizes({480, 760});
  setCentralWidget(split);
  resize(1280, 840);

  // -------------------------------------------------------------------- menu
  QMenu* file = menuBar()->addMenu("&File");
  auto makeAction = [this](const QString& text, const QKeySequence& key, void (MainWindow::*slot)()) {
    auto* a = new QAction(text, this);
    if (!key.isEmpty()) a->setShortcut(key);
    connect(a, &QAction::triggered, this, slot);
    return a;
  };
  openAct_ = makeAction("&Open URDF...", QKeySequence::Open, &MainWindow::chooseUrdf);
  addYamlAct_ = makeAction("&Add YAML file...", QKeySequence("Ctrl+Shift+O"), &MainWindow::chooseYaml);
  reloadAct_ = makeAction("&Reload files", QKeySequence("F5"), &MainWindow::reloadAll);
  savePackageAct_ = makeAction("Save &package...", QKeySequence::Save, &MainWindow::savePackage);
  saveYamlAct_ = makeAction("Save bridge &YAML...", QKeySequence("Ctrl+Shift+S"), &MainWindow::saveBridgeYaml);
  copyAct_ = makeAction("&Copy current preview", QKeySequence(), &MainWindow::copyCurrent);
  auto* templateAct = makeAction("Save overrides &template...", QKeySequence(), &MainWindow::saveTemplate);
  file->addAction(openAct_);
  file->addAction(addYamlAct_);
  file->addAction(reloadAct_);
  file->addSeparator();
  file->addAction(savePackageAct_);
  file->addAction(saveYamlAct_);
  file->addAction(templateAct);
  file->addSeparator();
  file->addAction(copyAct_);
  file->addSeparator();
  auto* quit = new QAction("&Quit", this);
  quit->setShortcut(QKeySequence::Quit);
  connect(quit, &QAction::triggered, this, &QWidget::close);
  file->addAction(quit);

  statusBar()->showMessage("Open a URDF to start.");
}

// ------------------------------------------------------------------- small helpers

QLineEdit* MainWindow::field(const QString& objectName) const { return findChild<QLineEdit*>(objectName); }

QPlainTextEdit* MainWindow::view(const QString& objectName) const { return findChild<QPlainTextEdit*>(objectName); }

QString MainWindow::startDir() const {
  QSettings settings("gz_bridge_helper", "gz_bridge_helper");
  return settings.value(kLastDirKey, QDir::homePath()).toString();
}

QString MainWindow::lastPackageDir() const {
  QSettings settings("gz_bridge_helper", "gz_bridge_helper");
  const QString ws = run_ ? run_->workspace() : QString();
  if (!ws.isEmpty() && QFileInfo(QDir(ws).filePath("src")).isDir()) return QDir(ws).filePath("src");
  return settings.value("lastPackageDir", startDir()).toString();
}

void MainWindow::rememberDir(const QString& path) {
  QSettings settings("gz_bridge_helper", "gz_bridge_helper");
  settings.setValue(kLastDirKey, QFileInfo(path).isDir() ? path : QFileInfo(path).absolutePath());
}

template <typename F>
void MainWindow::withChoicesKept(F&& change) {
  std::set<std::string> oldIds;
  const std::set<std::string> oldSelected = project_.selected();
  for (const gzb::Item& it : project_.detection().items) oldIds.insert(it.id);
  change();
  std::set<std::string> now;
  for (const gzb::Item& it : project_.detection().items) {
    if (oldIds.count(it.id) ? oldSelected.count(it.id) > 0 : it.selected) now.insert(it.id);
  }
  project_.setSelection(now);
}

// --------------------------------------------------------------------- opening files

void MainWindow::chooseUrdf() {
  const QString path = QFileDialog::getOpenFileName(this, "Open URDF", startDir(), "URDF files (*.urdf *.xml);;All files (*)");
  if (path.isEmpty()) return;
  rememberDir(path);
  openUrdf(path);
}

void MainWindow::chooseYaml() {
  const QStringList paths = QFileDialog::getOpenFileNames(this, "Add YAML files", startDir(), "YAML files (*.yaml *.yml);;All files (*)");
  for (const QString& p : paths) {
    rememberDir(p);
    addYaml(p);
  }
}

bool MainWindow::openUrdf(const QString& path) {
  last_ = Preview();  // never show another robot's output as "last valid"
  clearSettingsFields();
  project_.setWindowOverrides(gzb::Overrides());
  const bool read = project_.loadUrdfFile(path.toStdString());
  reflectUrdf();
  fillYamlList();
  fillItems();
  refreshNow();
  statusBar()->showMessage(read ? (project_.model().valid ? "Opened " + path : "Opened " + path + ", but it is not usable: see the problems below.")
                                : "Could not read " + path,
                           8000);
  return read && project_.model().valid;
}

bool MainWindow::addYaml(const QString& path) {
  for (const gzb::ParsedYaml& y : project_.yamls()) {
    if (y.path == path.toStdString()) {
      statusBar()->showMessage(QFileInfo(path).fileName() + " is already in the list.", 6000);
      return false;
    }
  }
  bool read = false;
  withChoicesKept([&]() { read = project_.addYamlFile(path.toStdString()); });
  fillYamlList();
  fillItems();
  refreshNow();
  return read;
}

void MainWindow::removeYaml(int row) {
  if (row < 0 || row >= static_cast<int>(project_.yamls().size())) return;
  withChoicesKept([&]() { project_.removeYaml(static_cast<size_t>(row)); });
  fillYamlList();
  fillItems();
  refreshNow();
}

void MainWindow::removeSelectedYaml() { removeYaml(yamlList_->currentRow()); }

void MainWindow::reloadAll() {
  if (project_.urdfPath().empty()) return;
  const std::vector<std::string> yamlPaths = [&]() {
    std::vector<std::string> v;
    for (const gzb::ParsedYaml& y : project_.yamls()) v.push_back(y.path);
    return v;
  }();
  withChoicesKept([&]() {
    project_.clearYaml();
    project_.loadUrdfFile(project_.urdfPath());
    for (const std::string& p : yamlPaths) project_.addYamlFile(p);
  });
  useOverridesFromWindow();
  reflectUrdf();
  fillYamlList();
  fillItems();
  refreshNow();
  statusBar()->showMessage("Read the files again.", 5000);
}

void MainWindow::saveTemplate() {
  const QString path = QFileDialog::getSaveFileName(this, "Save overrides template", QDir(startDir()).filePath("overrides.yaml"),
                                                    "YAML files (*.yaml *.yml);;All files (*)");
  if (path.isEmpty()) return;
  QSaveFile f(path);
  if (!f.open(QIODevice::WriteOnly) || f.write(q(gzb::overridesTemplate()).toUtf8()) < 0 || !f.commit()) {
    QMessageBox::critical(this, "Save failed", "Cannot write " + path + ": " + f.errorString());
    return;
  }
  rememberDir(path);
  statusBar()->showMessage("Wrote " + path + ". Edit it, then add it with Add... in the YAML list.", 10000);
}

void MainWindow::dragEnterEvent(QDragEnterEvent* event) {
  if (event->mimeData()->hasUrls()) event->acceptProposedAction();
}

void MainWindow::dropEvent(QDropEvent* event) {
  QStringList paths;
  for (const QUrl& url : event->mimeData()->urls()) {
    if (url.isLocalFile()) paths << url.toLocalFile();
  }
  openPaths(paths);
  event->acceptProposedAction();
}

void MainWindow::openPaths(const QStringList& paths) {
  for (const QString& path : paths) {
    const QString suffix = QFileInfo(path).suffix().toLower();
    if (suffix == "yaml" || suffix == "yml") addYaml(path);
    else openUrdf(path);
  }
}

// ------------------------------------------------------------------ the left column

void MainWindow::reflectUrdf() {
  const gzb::UrdfModel& m = project_.model();
  urdfPath_->setText(q(project_.urdfPath()));
  urdfPath_->setToolTip(q(project_.urdfPath()));
  if (project_.urdfPath().empty()) {
    urdfSummary_->setText(QString());
    setWindowTitle("Gazebo Bridge Helper");
    return;
  }
  setWindowTitle("Gazebo Bridge Helper — " + QFileInfo(q(project_.urdfPath())).fileName());
  if (m.valid) {
    urdfSummary_->setText(QString("<b>%1</b>: %2 links, %3 joints (%4 movable), root link %5")
                              .arg(q(m.name).toHtmlEscaped())
                              .arg(m.links.size())
                              .arg(m.joints.size())
                              .arg(m.movableJoints().size())
                              .arg(q(m.root).toHtmlEscaped()));
  } else {
    urdfSummary_->setText("This file cannot be used yet: see the problems below.");
  }
}

void MainWindow::fillYamlList() {
  yamlList_->clear();
  for (const gzb::ParsedYaml& y : project_.yamls()) {
    auto* item = new QListWidgetItem(QFileInfo(q(y.path)).fileName() + "  —  " + (y.ok ? q(y.kindText()) : QString("not usable")));
    item->setToolTip(q(y.path));
    yamlList_->addItem(item);
  }
  updateButtons();
}

void MainWindow::fillItems() {
  loadingItems_ = true;
  items_->clear();
  const gzb::Detection& d = project_.detection();
  std::map<std::string, QTreeWidgetItem*> categories;
  for (const gzb::Item& it : d.items) {
    QTreeWidgetItem*& cat = categories[it.category];
    if (!cat) {
      cat = new QTreeWidgetItem(items_);
      cat->setText(0, q(it.category));
      QFont f = cat->font(0);
      f.setBold(true);
      cat->setFont(0, f);
      cat->setFlags(Qt::ItemIsEnabled);
      cat->setExpanded(true);
    }
    auto* child = new QTreeWidgetItem(cat);
    child->setText(0, q(it.title));
    child->setText(1, it.existing ? "in the URDF" : "will be added");
    child->setToolTip(0, q(it.detail));
    child->setData(0, Qt::UserRole, q(it.id));
    child->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsUserCheckable);
    child->setCheckState(0, project_.selected().count(it.id) ? Qt::Checked : Qt::Unchecked);
  }
  items_->expandAll();
  loadingItems_ = false;

  if (project_.urdfPath().empty() || !project_.model().valid) {
    modeLabel_->setText(QString());
  } else if (d.mode == gzb::ControlMode::Ros2Control) {
    modeLabel_->setText("The URDF has a <b>ros2_control</b> section: ros2_control moves the joints, so only /clock and the sensors go through the bridge.");
  } else {
    modeLabel_->setText("Gazebo's own systems move the joints (no ros2_control section found).");
  }
  if (d.items.empty() && project_.model().valid) itemDetail_->setText("Nothing to suggest for this URDF.");
  else itemDetail_->setText(QString());
}

void MainWindow::onItemChanged(QTreeWidgetItem* item, int column) {
  if (loadingItems_ || column != 0) return;
  const QString id = item->data(0, Qt::UserRole).toString();
  if (id.isEmpty()) return;
  project_.setSelected(id.toStdString(), item->checkState(0) == Qt::Checked);
  scheduleRefresh();
}

void MainWindow::setItemChecked(const QString& id, bool on) {
  for (int c = 0; c < items_->topLevelItemCount(); ++c) {
    QTreeWidgetItem* cat = items_->topLevelItem(c);
    for (int i = 0; i < cat->childCount(); ++i) {
      if (cat->child(i)->data(0, Qt::UserRole).toString() == id) cat->child(i)->setCheckState(0, on ? Qt::Checked : Qt::Unchecked);
    }
  }
}

bool MainWindow::itemChecked(const QString& id) const {
  return project_.selected().count(id.toStdString()) > 0;
}

void MainWindow::resetSuggestions() {
  project_.setSelection(project_.detection().defaultSelection());
  fillItems();
  refreshNow();
}

void MainWindow::clearSettingsFields() {
  for (QLineEdit* e : {package_, model_, world_, spawnX_, spawnY_, spawnZ_, spawnYaw_}) {
    const QSignalBlocker block(e);
    e->clear();
    e->setStyleSheet(QString());
  }
}

gzb::Overrides MainWindow::windowOverrides() const {
  gzb::Overrides o;
  auto text = [](QLineEdit* e) { return e->text().trimmed().toStdString(); };
  if (!text(package_).empty()) o.packageName = text(package_);
  if (!text(model_).empty()) o.modelName = text(model_);
  if (!text(world_).empty()) o.worldName = text(world_);
  auto number = [](QLineEdit* e, std::optional<double>& out) {
    const QString t = e->text().trimmed();
    e->setStyleSheet(QString());
    e->setToolTip(e->property("tip").toString());
    if (t.isEmpty()) return;
    bool ok = false;
    const double v = t.toDouble(&ok);
    if (ok) {
      out = v;
    } else {
      e->setStyleSheet("QLineEdit { border: 1px solid #d92d20; }");
      e->setToolTip("Not a number: this value is ignored.");
    }
  };
  number(spawnX_, o.spawnX);
  number(spawnY_, o.spawnY);
  number(spawnZ_, o.spawnZ);
  number(spawnYaw_, o.spawnYaw);
  return o;
}

void MainWindow::useOverridesFromWindow() {
  project_.setWindowOverrides(windowOverrides());
}

void MainWindow::onSettingsEdited() {
  if (project_.urdfPath().empty()) return;
  useOverridesFromWindow();
  scheduleRefresh();
}

void MainWindow::updatePlaceholders() {
  const gzb::Plan& p = project_.plan();
  const bool have = project_.model().valid;
  auto set = [&](QLineEdit* e, const QString& text) { e->setPlaceholderText(have ? text : QString()); };
  set(package_, q(p.packageName));
  set(model_, q(p.modelName));
  set(world_, q(p.worldName));
  set(spawnX_, "0");
  set(spawnY_, "0");
  set(spawnZ_, QString::number(p.spawnXyz.z, 'g', 6));
  set(spawnYaw_, "0");
}

// ---------------------------------------------------------------- the right column

void MainWindow::scheduleRefresh() { timer_.start(); }

void MainWindow::refreshNow() {
  timer_.stop();
  const gzb::Plan& plan = project_.plan();
  fillBridgeTable();
  fillPreviews();
  updatePlaceholders();
  showIssues();

  const bool haveFiles = !project_.generated().files.empty();
  if (project_.urdfPath().empty()) {
    summary_->setText("Open a URDF to begin.");
  } else if (!project_.model().valid) {
    summary_->setText("Cannot generate: fix the errors below.");
  } else {
    summary_->setText(QString("Package <b>%1</b>: %2 files, %3 bridged topics%4")
                          .arg(q(plan.packageName).toHtmlEscaped())
                          .arg(project_.generated().files.size())
                          .arg(plan.bridge.size())
                          .arg(project_.ready() ? "" : " — not ready: see the errors below"));
  }
  stale_->setVisible(!haveFiles && last_.have && !project_.urdfPath().empty());

  std::vector<gzb::TopicCheck> checks = project_.model().valid ? gzb::checksFromPlan(plan) : std::vector<gzb::TopicCheck>();
  run_->setPackage(q(plan.packageName), checks, project_.ready(), [this](const QString& dir, QString* error) { return savePackageTo(dir, error); });
  updateButtons();
}

void MainWindow::fillBridgeTable() {
  const gzb::Plan& plan = project_.plan();
  const bool valid = project_.model().valid;
  const auto& entries = plan.bridge;
  table_->setRowCount(valid ? static_cast<int>(entries.size()) : 0);
  int toRos = 0, toGz = 0, both = 0, services = 0;
  for (size_t i = 0; valid && i < entries.size(); ++i) {
    const gzb::BridgeEntry& e = entries[i];
    const int r = static_cast<int>(i);
    QStringList cells;
    if (e.isService()) {
      cells << q(e.serviceName) << "⇄" << q(e.serviceName) << "(service)" << q(e.gzRequestType + " / " + e.gzReplyType);
      ++services;
    } else {
      cells << q(e.rosTopic) << directionArrow(e.direction) << q(e.gzTopic) << q(e.purpose) << q(e.rosType) << q(e.gzType);
      if (e.direction == gzb::Direction::GzToRos) ++toRos;
      else if (e.direction == gzb::Direction::RosToGz) ++toGz;
      else ++both;
    }
    cells << q(e.origin);
    const QColor bg = groupColor(e.group);
    for (int c = 0; c < cells.size(); ++c) {
      auto* item = new QTableWidgetItem(cells[c]);
      item->setBackground(bg);
      QString tip = q(e.purpose);
      if (!e.isService()) tip += "\n" + directionWords(e.direction);
      if (e.lazy) tip += "\nlazy: only bridged while somebody subscribes";
      if (!e.qos.empty()) tip += "\nqos_profile: " + q(e.qos);
      if (!e.frameId.empty()) tip += "\nheader.frame_id is set to " + q(e.frameId);
      item->setToolTip(tip);
      if (c == 1) item->setTextAlignment(Qt::AlignCenter);
      if (!e.enabled) {
        QFont f = item->font();
        f.setItalic(true);
        item->setFont(f);
      }
      table_->setItem(r, c, item);
    }
  }
  if (!valid) {
    bridgeSummary_->setText(QString());
  } else if (entries.empty()) {
    bridgeSummary_->setText("nothing is bridged");
  } else {
    QStringList parts;
    if (toRos) parts << QString("%1 from Gazebo").arg(toRos);
    if (toGz) parts << QString("%1 to Gazebo").arg(toGz);
    if (both) parts << QString("%1 both ways").arg(both);
    if (services) parts << QString("%1 services").arg(services);
    bridgeSummary_->setText(QString("%1 entries: %2").arg(entries.size()).arg(parts.join(", ")));
  }
}

void MainWindow::fillPreviews() {
  const gzb::Generated& g = project_.generated();
  if (!g.files.empty()) {
    Preview p;
    p.have = true;
    p.package = q(project_.plan().packageName);
    for (const gzb::GeneratedFile& f : g.files) {
      const QString path = q(f.path);
      const QString text = q(f.content);
      if (path.startsWith("config/") && path.endsWith("_bridge.yaml")) p.bridgeYaml = text;
      else if (path == "launch/sim.launch.py") p.launch = text;
      else if (path.startsWith("worlds/")) p.world = text;
      else if (path.startsWith("urdf/")) p.urdf = text;
      else p.otherFiles.emplace_back(path, text);
    }
    p.urdfChanges = q(g.urdfAdditions);
    last_ = p;
  }
  setPreviewText(bridgeView_, last_.bridgeYaml);
  setPreviewText(launchView_, last_.launch);
  setPreviewText(worldView_, last_.world);
  setPreviewText(changesView_, last_.urdfChanges.isEmpty() && last_.have ? "<!-- Nothing was added: the URDF already has everything that is ticked. -->" : last_.urdfChanges);
  setPreviewText(urdfView_, last_.urdf);

  // The "Package files" list: keep the file that was shown when it still exists.
  const QString current = fileBox_->currentText();
  const QSignalBlocker block(fileBox_);
  fileBox_->clear();
  for (const auto& f : last_.otherFiles) fileBox_->addItem(f.first);
  const int keep = fileBox_->findText(current);
  fileBox_->setCurrentIndex(keep >= 0 ? keep : (fileBox_->count() ? 0 : -1));
  const int index = fileBox_->currentIndex();
  if (index >= 0) {
    const auto& f = last_.otherFiles[static_cast<size_t>(index)];
    if (auto* h = fileView_->document()->findChild<CodeHighlighter*>()) h->setLanguage(languageForFile(f.first));
    setPreviewText(fileView_, f.second);
  } else {
    setPreviewText(fileView_, QString());
  }
}

void MainWindow::showIssues() {
  issues_->clear();
  const gzb::Issues all = project_.issues();
  // Errors first, then warnings, then information.
  for (gzb::Severity s : {gzb::Severity::Error, gzb::Severity::Warning, gzb::Severity::Info}) {
    for (const gzb::Issue& i : all) {
      if (i.severity != s) continue;
      QString label = q(gzb::severityName(s));
      if (!label.isEmpty()) label[0] = label[0].toUpper();
      auto* item = new QListWidgetItem(QString("%1 [%2]: %3").arg(label, q(i.where), q(i.message)));
      item->setForeground(severityColor(s));
      item->setToolTip(item->text());
      issues_->addItem(item);
    }
  }
  issues_->setVisible(issues_->count() > 0);
}

void MainWindow::updateButtons() {
  const bool ok = canSave();
  for (QWidget* w : {static_cast<QWidget*>(savePackageButton_), static_cast<QWidget*>(saveYamlButton_)}) w->setEnabled(ok);
  copyButton_->setEnabled(project_.model().valid);
  for (QAction* a : {savePackageAct_, saveYamlAct_}) a->setEnabled(ok);
  copyAct_->setEnabled(project_.model().valid);
  reloadButton_->setEnabled(!project_.urdfPath().empty());
  reloadAct_->setEnabled(!project_.urdfPath().empty());
  removeYamlButton_->setEnabled(yamlList_->currentRow() >= 0);
}

bool MainWindow::canSave() const { return project_.ready() && !project_.generated().files.empty(); }

// ------------------------------------------------------------------------ saving

bool MainWindow::savePackageTo(const QString& dir, QString* error) {
  std::string err;
  if (!project_.write(dir.toStdString(), err)) {
    if (error) *error = q(err);
    return false;
  }
  return true;
}

bool MainWindow::saveBridgeYamlTo(const QString& path, QString* error) {
  if (!canSave()) {
    if (error) *error = "There are errors to fix first.";
    return false;
  }
  QSaveFile f(path);  // temporary file + rename: a failure never leaves half a file
  if (!f.open(QIODevice::WriteOnly) || f.write(last_.bridgeYaml.toUtf8()) < 0 || !f.commit()) {
    if (error) *error = QString("Cannot write %1: %2").arg(path, f.errorString());
    return false;
  }
  return true;
}

void MainWindow::savePackage() {
  if (!canSave()) return;
  const QString dir = QFileDialog::getExistingDirectory(
      this, QString("Save the package %1 into").arg(q(project_.plan().packageName)), lastPackageDir());
  if (dir.isEmpty()) return;
  const QString target = QDir(dir).filePath(q(project_.plan().packageName));
  if (QFileInfo::exists(target) &&
      QMessageBox::question(this, "Replace the files?",
                            QString("%1 already exists. The files this helper generates will be replaced; other files in it are left alone.").arg(target)) !=
          QMessageBox::Yes) {
    return;
  }
  QString error;
  if (!savePackageTo(dir, &error)) {
    QMessageBox::critical(this, "Save failed", error);
    return;
  }
  QSettings settings("gz_bridge_helper", "gz_bridge_helper");
  settings.setValue("lastPackageDir", dir);
  const QString pkg = q(project_.plan().packageName);
  statusBar()->showMessage(
      QString("Wrote %1 (%2 files). Build it: colcon build --packages-select %3").arg(target).arg(project_.generated().files.size()).arg(pkg), 15000);
}

void MainWindow::saveBridgeYaml() {
  if (!canSave()) return;
  const QString path = QFileDialog::getSaveFileName(this, "Save bridge YAML", QDir(startDir()).filePath(q(project_.plan().packageName) + "_bridge.yaml"),
                                                    "YAML files (*.yaml *.yml);;All files (*)");
  if (path.isEmpty()) return;
  QString error;
  if (!saveBridgeYamlTo(path, &error)) {
    QMessageBox::critical(this, "Save failed", error);
    return;
  }
  rememberDir(path);
  statusBar()->showMessage("Wrote " + path + ". Use it with: ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=" + path, 15000);
}

void MainWindow::copyCurrent() {
  QString text;
  const int tab = tabs_->currentIndex();
  switch (tab) {
    case 0: text = bridgeView_->toPlainText(); break;
    case 1: text = launchView_->toPlainText(); break;
    case 2: text = worldView_->toPlainText(); break;
    case 3: text = changesView_->toPlainText(); break;
    case 4: text = urdfView_->toPlainText(); break;
    case 5: text = fileView_->toPlainText(); break;
    default: text = run_->log()->toPlainText(); break;
  }
  if (text.isEmpty()) return;
  QGuiApplication::clipboard()->setText(text);
  statusBar()->showMessage("Copied " + tabs_->tabText(tab), 4000);
}

void MainWindow::closeEvent(QCloseEvent* event) {
  run_->shutdown();
  QMainWindow::closeEvent(event);
}

}  // namespace gzgui
