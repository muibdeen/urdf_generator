// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// 3D view of the robot being generated. Drag to orbit, right-drag (or
// shift-drag) to pan, wheel to zoom, double-click to reset. Drawn with the
// software renderer in urdfgen::renderScene, so it needs no OpenGL.
#pragma once

#include <QImage>
#include <QPoint>
#include <QSize>
#include <QWidget>

#include "urdf_generator/render.hpp"
#include "urdf_generator/scene.hpp"

class QPainter;

namespace urdfgui {

class RobotView : public QWidget {
  Q_OBJECT

 public:
  enum class Preset { Isometric, Front, Side, Top };

  explicit RobotView(QWidget* parent = nullptr);

  /// Shows a new scene. With `fit` the camera is re-aimed at it. Without, the camera stays
  /// where it is (so a changed dimension is visible against the grid), except that it
  /// zooms out if the scene has grown out of view and the user has not moved the camera.
  ///
  /// `poseOnly` marks a change that only moved the joints (the sliders). If that pushes the
  /// robot out of view the camera then just backs away, keeping its aim, instead of re-fitting,
  /// so the picture does not jump around while a slider is dragged.
  void setScene(urdfgen::Scene scene, bool fit, bool poseOnly = false);
  const urdfgen::Scene& scene() const { return scene_; }
  const urdfgen::Camera& camera() const { return camera_; }
  bool userMovedCamera() const { return userMoved_; }

  void setPreset(Preset preset);
  void resetView() { setPreset(Preset::Isometric); }
  /// The x/y/z axes of every link frame: thin lines. Sensor frames are not part of it.
  void setShowFrames(bool on);
  bool showFrames() const { return showFrames_; }
  /// The axes of the sensors (urdfgen::Scene::sensors): thick, outlined arrows with x/y/z
  /// letters and a name tag, drawn so that they cannot be mistaken for a link frame. On by default.
  void setShowSensorAxes(bool on);
  bool showSensorAxes() const { return showSensorAxes_; }

  /// The complete picture (background, grid, robot, overlays) at the given size.
  QImage renderImage(const QSize& size) const;

  QSize sizeHint() const override { return QSize(520, 380); }

 protected:
  void paintEvent(QPaintEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void mouseDoubleClickEvent(QMouseEvent* event) override;

 private:
  void fitCamera();
  void paintContent(QPainter& p, const QSize& size, qreal devicePixelRatio) const;
  void drawGrid(QPainter& p, int w, int h, bool dark) const;
  void drawFrames(QPainter& p, int w, int h) const;
  void drawSensors(QPainter& p, int w, int h, bool dark) const;
  void drawGizmo(QPainter& p, int w, int h, bool dark) const;

  urdfgen::Scene scene_;
  urdfgen::Camera camera_;
  bool showFrames_ = false;
  bool showSensorAxes_ = true;
  bool userMoved_ = false;
  QPoint lastPos_;
};

}  // namespace urdfgui
