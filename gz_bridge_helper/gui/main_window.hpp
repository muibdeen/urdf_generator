// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// The window. Same layout as the URDF generator: inputs in a column on the left, the result on
// the right (a table of the bridged topics above, the generated files and the Run tab below).
#pragma once

#include <QMainWindow>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <utility>
#include <vector>

#include "gz_bridge_helper/project.hpp"

class QAction;
class QComboBox;
class QDragEnterEvent;
class QDropEvent;
class QLabel;
class QLineEdit;
class QListWidget;
class QPlainTextEdit;
class QPushButton;
class QTabWidget;
class QTableWidget;
class QTreeWidget;
class QTreeWidgetItem;

namespace gzgui {

class RunPanel;

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(QWidget* parent = nullptr);

  // ---- the same actions as the menu, without dialogs (used by main() and by the tests) ----
  /// Loads a URDF and suggests what to add to it. False when the file cannot be read.
  bool openUrdf(const QString& path);
  /// Adds a YAML file (controllers, a bridge config or overrides; told apart by content).
  bool addYaml(const QString& path);
  void removeYaml(int row);
  /// What dropping files on the window does: *.yaml/*.yml are added, anything else is the URDF.
  void openPaths(const QStringList& paths);
  /// Reads the URDF and the YAML files again; the ticks of items that still exist are kept.
  void reloadAll();
  /// Regenerates the previews immediately instead of waiting for the short debounce.
  void refreshNow();
  bool canSave() const;
  /// Writes `<dir>/<package>/...`. False and `error` on failure.
  bool savePackageTo(const QString& dir, QString* error = nullptr);
  bool saveBridgeYamlTo(const QString& path, QString* error = nullptr);

  gzb::Project& project() { return project_; }
  RunPanel* runPanel() const { return run_; }
  QTabWidget* tabs() const { return tabs_; }
  QTableWidget* bridgeTable() const { return table_; }
  QTreeWidget* itemsTree() const { return items_; }
  QListWidget* yamlList() const { return yamlList_; }
  QListWidget* issuesList() const { return issues_; }
  QLineEdit* field(const QString& objectName) const;
  QPlainTextEdit* view(const QString& objectName) const;
  QComboBox* packageFileBox() const { return fileBox_; }
  /// The ticked state of an item of the detected list, by id ("sensor:front_camera").
  void setItemChecked(const QString& id, bool on);
  bool itemChecked(const QString& id) const;
  QString lastPackageDir() const;

 protected:
  void closeEvent(QCloseEvent* event) override;
  void dragEnterEvent(QDragEnterEvent* event) override;
  void dropEvent(QDropEvent* event) override;

 private slots:
  void chooseUrdf();
  void chooseYaml();
  void removeSelectedYaml();
  void saveTemplate();
  void scheduleRefresh();
  void onItemChanged(QTreeWidgetItem* item, int column);
  void onSettingsEdited();
  void resetSuggestions();
  void savePackage();
  void saveBridgeYaml();
  void copyCurrent();

 private:
  struct Preview {
    QString bridgeYaml, launch, world, urdfChanges, urdf;
    std::vector<std::pair<QString, QString>> otherFiles;  // path, text
    QString package;
    bool have = false;
  };

  void buildUi();
  void fillItems();
  void fillYamlList();
  void fillBridgeTable();
  void fillPreviews();
  void showIssues();
  void updatePlaceholders();
  void updateButtons();
  void rememberDir(const QString& path);
  QString startDir() const;
  /// Runs `change` (which modifies the project and re-detects), then restores the ticks.
  template <typename F>
  void withChoicesKept(F&& change);
  gzb::Overrides windowOverrides() const;
  void useOverridesFromWindow();
  void clearSettingsFields();
  void reflectUrdf();

  gzb::Project project_;
  Preview last_;
  QTimer timer_;
  bool loadingItems_ = false;

  // left
  QLabel *hint_ = nullptr, *urdfSummary_ = nullptr, *modeLabel_ = nullptr, *itemDetail_ = nullptr;
  QLineEdit* urdfPath_ = nullptr;
  QListWidget* yamlList_ = nullptr;
  QLineEdit *package_ = nullptr, *model_ = nullptr, *world_ = nullptr, *spawnX_ = nullptr, *spawnY_ = nullptr,
            *spawnZ_ = nullptr, *spawnYaw_ = nullptr;
  QTreeWidget* items_ = nullptr;
  QPushButton *removeYamlButton_ = nullptr, *reloadButton_ = nullptr;

  // right
  QTableWidget* table_ = nullptr;
  QLabel* bridgeSummary_ = nullptr;
  QTabWidget* tabs_ = nullptr;
  QPlainTextEdit *bridgeView_ = nullptr, *launchView_ = nullptr, *worldView_ = nullptr, *changesView_ = nullptr,
                 *urdfView_ = nullptr, *fileView_ = nullptr;
  QComboBox* fileBox_ = nullptr;
  RunPanel* run_ = nullptr;
  QLabel *stale_ = nullptr, *summary_ = nullptr;
  QListWidget* issues_ = nullptr;
  QPushButton *copyButton_ = nullptr, *saveYamlButton_ = nullptr, *savePackageButton_ = nullptr;
  QAction *openAct_ = nullptr, *addYamlAct_ = nullptr, *reloadAct_ = nullptr, *savePackageAct_ = nullptr,
          *saveYamlAct_ = nullptr, *copyAct_ = nullptr;
};

}  // namespace gzgui
