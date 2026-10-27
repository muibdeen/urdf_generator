// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include <QApplication>

#include "main_window.hpp"

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  QApplication::setApplicationName("urdf_generator");
  QApplication::setOrganizationName("urdf_generator");

  urdfgui::MainWindow window;
  window.show();
  return app.exec();
}
