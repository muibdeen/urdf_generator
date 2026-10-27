// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
//   gz_bridge_helper_gui [robot.urdf [more.yaml ...]]
#include <QApplication>
#include <QFileInfo>

#include "main_window.hpp"

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  QApplication::setApplicationName("gz_bridge_helper");
  QApplication::setOrganizationName("gz_bridge_helper");

  gzgui::MainWindow window;
  window.show();

  // Files given on the command line (ros2 run adds its own --ros-args, which are skipped).
  QStringList paths;
  const QStringList args = QApplication::arguments();
  for (int i = 1; i < args.size(); ++i) {
    if (args[i] == "--ros-args") break;
    paths << args[i];
  }
  // The URDF first, then the YAML files, whatever the order on the command line.
  QStringList urdf, yaml;
  for (const QString& p : paths) {
    const QString suffix = QFileInfo(p).suffix().toLower();
    (suffix == "yaml" || suffix == "yml" ? yaml : urdf) << p;
  }
  if (!urdf.isEmpty()) window.openUrdf(urdf.first());
  for (const QString& p : yaml) window.addYaml(p);
  return app.exec();
}
