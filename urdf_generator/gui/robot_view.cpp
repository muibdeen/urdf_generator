// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include "robot_view.hpp"

#include <QFont>
#include <QFontMetricsF>
#include <QLinearGradient>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPalette>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <vector>

namespace urdfgui {
namespace {

using urdfgen::Camera;
using urdfgen::Vec3;

const QColor kAxisX("#e5484d");
const QColor kAxisY("#30a46c");
const QColor kAxisZ("#3e63dd");

constexpr double kMaxElevation = 89.5;

bool isDark(const QWidget* w) { return w->palette().color(QPalette::Base).lightness() < 128; }

/// 1, 2 or 5 times a power of ten, close to `raw`.
double niceStep(double raw) {
  if (!(raw > 0)) return 1.0;
  const double p = std::pow(10.0, std::floor(std::log10(raw)));
  const double m = raw / p;
  return (m < 1.5 ? 1.0 : m < 3.5 ? 2.0 : m < 7.5 ? 5.0 : 10.0) * p;
}

}  // namespace

RobotView::RobotView(QWidget* parent) : QWidget(parent) {
  setMinimumSize(240, 180);
  setFocusPolicy(Qt::NoFocus);
  setToolTip("Drag: orbit    Right-drag or Shift-drag: pan    Wheel: zoom    Double-click: reset");
  camera_ = Camera::fit(scene_, 1.4, -50.0, 25.0);
}

void RobotView::fitCamera() {
  const double aspect = static_cast<double>(std::max(1, width())) / std::max(1, height());
  camera_ = Camera::fit(scene_, aspect, camera_.azimuthDeg, camera_.elevationDeg);
  update();
}

void RobotView::setScene(urdfgen::Scene scene, bool fit, bool poseOnly) {
  scene_ = std::move(scene);
  if (fit) {
    userMoved_ = false;
    fitCamera();
  } else if (!userMoved_ && !camera_.contains(scene_, width(), height(), 4.0)) {
    if (poseOnly) {
      // Back away until the posed robot is in view again (never closer, never re-aimed).
      const double limit = 30.0 * std::max(scene_.radius(), 1e-3);
      for (int i = 0; i < 60 && camera_.distance < limit && !camera_.contains(scene_, width(), height(), 4.0); ++i) {
        camera_.distance *= 1.1;
      }
    } else {
      fitCamera();  // the model grew out of view and the user has not framed it themselves
    }
  }
  update();
}

void RobotView::setPreset(Preset preset) {
  switch (preset) {
    case Preset::Isometric: camera_.azimuthDeg = -50.0; camera_.elevationDeg = 25.0; break;
    case Preset::Front: camera_.azimuthDeg = 0.0; camera_.elevationDeg = 0.0; break;
    case Preset::Side: camera_.azimuthDeg = -90.0; camera_.elevationDeg = 0.0; break;
    case Preset::Top: camera_.azimuthDeg = -90.0; camera_.elevationDeg = kMaxElevation; break;
  }
  userMoved_ = false;
  fitCamera();
}

void RobotView::setShowFrames(bool on) {
  showFrames_ = on;
  update();
}

void RobotView::setShowSensorAxes(bool on) {
  showSensorAxes_ = on;
  update();
}

void RobotView::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
  if (!userMoved_) fitCamera();
}

// ----------------------------------------------------------------- interaction

void RobotView::mousePressEvent(QMouseEvent* event) { lastPos_ = event->position().toPoint(); }

void RobotView::mouseMoveEvent(QMouseEvent* event) {
  const QPoint pos = event->position().toPoint();
  const QPoint d = pos - lastPos_;
  lastPos_ = pos;
  const bool pan = (event->buttons() & (Qt::RightButton | Qt::MiddleButton)) ||
                   ((event->buttons() & Qt::LeftButton) && (event->modifiers() & Qt::ShiftModifier));
  const bool orbit = (event->buttons() & Qt::LeftButton) && !pan;
  if (orbit) {
    camera_.azimuthDeg -= d.x() * 0.4;
    camera_.elevationDeg = std::max(-kMaxElevation, std::min(kMaxElevation, camera_.elevationDeg + d.y() * 0.4));
  } else if (pan) {
    Vec3 right, up, back;
    camera_.axes(&right, &up, &back);
    const double perPixel = 2.0 * camera_.distance * std::tan(camera_.fovDeg * urdfgen::kPi / 360.0) / std::max(1, height());
    camera_.target = camera_.target - right * (d.x() * perPixel) + up * (d.y() * perPixel);
  } else {
    return;
  }
  userMoved_ = true;
  update();
}

void RobotView::wheelEvent(QWheelEvent* event) {
  const double steps = event->angleDelta().y() / 120.0;
  if (steps == 0.0) return;
  const double r = std::max(scene_.empty() ? 0.0 : scene_.radius(), 1e-3);
  camera_.distance = std::max(0.2 * r, std::min(30.0 * r, camera_.distance * std::pow(0.88, steps)));
  userMoved_ = true;
  update();
  event->accept();
}

void RobotView::mouseDoubleClickEvent(QMouseEvent*) { resetView(); }

// ------------------------------------------------------------------- painting

void RobotView::paintEvent(QPaintEvent*) {
  QPainter p(this);
  paintContent(p, size(), devicePixelRatioF());
}

QImage RobotView::renderImage(const QSize& size) const {
  QImage img(size, QImage::Format_ARGB32_Premultiplied);
  img.fill(Qt::transparent);
  QPainter p(&img);
  paintContent(p, size, 1.0);
  return img;
}

void RobotView::paintContent(QPainter& p, const QSize& size, qreal dpr) const {
  const int w = size.width(), h = size.height();
  if (w < 2 || h < 2) return;
  const bool dark = isDark(this);

  QLinearGradient bg(0, 0, 0, h);
  bg.setColorAt(0.0, dark ? QColor("#2d3139") : QColor("#f4f6f9"));
  bg.setColorAt(1.0, dark ? QColor("#1b1e23") : QColor("#dde2e9"));
  p.fillRect(QRect(0, 0, w, h), bg);

  p.setRenderHint(QPainter::Antialiasing, true);
  drawGrid(p, w, h, dark);

  // The robot: rendered at twice the resolution and scaled down, which smooths the edges.
  // Very large views (high-DPI screens) are rendered at 1x to keep dragging responsive.
  const double physW = w * dpr, physH = h * dpr;
  const int ss = physW * physH <= 1.2e6 ? 2 : 1;
  const urdfgen::Image img = urdfgen::renderScene(scene_, camera_, static_cast<int>(physW * ss), static_cast<int>(physH * ss));
  const QImage q(reinterpret_cast<const uchar*>(img.pixels.data()), img.width, img.height,
                 img.width * 4, QImage::Format_ARGB32_Premultiplied);
  p.setRenderHint(QPainter::SmoothPixmapTransform, true);
  p.drawImage(QRectF(0, 0, w, h), q);

  if (showFrames_) drawFrames(p, w, h);
  if (showSensorAxes_) drawSensors(p, w, h, dark);
  drawGizmo(p, w, h, dark);

  if (scene_.empty()) {
    p.setPen(dark ? QColor("#9aa4b2") : QColor("#5b6572"));
    p.drawText(QRect(0, 0, w, h), Qt::AlignCenter, "Nothing to show yet");
  }
}

void RobotView::drawGrid(QPainter& p, int w, int h, bool dark) const {
  const double groundZ = scene_.empty() ? 0.0 : scene_.groundZ;
  const double step = niceStep(camera_.distance / 6.0);
  constexpr int kHalfLines = 18;
  constexpr int kSegments = 18;
  const double half = step * kHalfLines;
  const double cx = std::round(camera_.target.x / step) * step;
  const double cy = std::round(camera_.target.y / step) * step;
  const QColor base = dark ? QColor(255, 255, 255) : QColor(0, 0, 0);

  auto drawLine = [&](bool alongY, double fixed) {
    const bool isAxis = std::fabs(fixed) < step * 1e-6;
    QColor colour = base;
    if (isAxis) colour = alongY ? kAxisY : kAxisX;  // the line along y at x = 0 is the world y axis
    const double centre = alongY ? cy : cx;
    for (int k = 0; k < kSegments; ++k) {
      const double t0 = centre - half + 2 * half * k / kSegments;
      const double t1 = centre - half + 2 * half * (k + 1) / kSegments;
      const double mid = 0.5 * (t0 + t1) - centre;
      const double fade = std::max(0.0, 1.0 - std::fabs(mid) / half);
      const double along = alongY ? fixed - cx : fixed - cy;
      const double radial = std::max(0.0, 1.0 - std::hypot(mid, along) / (half * 1.05));
      colour.setAlphaF(std::min(1.0, (isAxis ? 0.85 : 0.22) * std::pow(std::min(fade, radial) + 1e-9, 0.8)));
      const Vec3 a = alongY ? Vec3{fixed, t0, groundZ} : Vec3{t0, fixed, groundZ};
      const Vec3 b = alongY ? Vec3{fixed, t1, groundZ} : Vec3{t1, fixed, groundZ};
      Camera::Screen sa, sb;
      if (!camera_.projectSegment(a, b, w, h, &sa, &sb)) continue;
      p.setPen(QPen(colour, isAxis ? 1.6 : 1.0));
      p.drawLine(QPointF(sa.x, sa.y), QPointF(sb.x, sb.y));
    }
  };
  for (int i = -kHalfLines; i <= kHalfLines; ++i) {
    drawLine(true, cx + i * step);
    drawLine(false, cy + i * step);
  }
}

void RobotView::drawFrames(QPainter& p, int w, int h) const {
  const double len = std::max(0.01, 0.1 * scene_.radius());
  const QColor colours[3] = {kAxisX, kAxisY, kAxisZ};
  std::set<std::string> sensorLinks;  // drawn by drawSensors() instead
  for (const urdfgen::SceneSensor& s : scene_.sensors) sensorLinks.insert(s.link);
  for (const urdfgen::SceneFrame& f : scene_.frames) {
    if (sensorLinks.count(f.link)) continue;
    for (int axis = 0; axis < 3; ++axis) {
      const Vec3 dir{f.pose.r[static_cast<std::size_t>(axis)], f.pose.r[static_cast<std::size_t>(3 + axis)],
                     f.pose.r[static_cast<std::size_t>(6 + axis)]};
      Camera::Screen a, b;
      if (!camera_.projectSegment(f.pose.t, f.pose.t + dir * len, w, h, &a, &b)) continue;
      p.setPen(QPen(colours[axis], 1.8));
      p.drawLine(QPointF(a.x, a.y), QPointF(b.x, b.y));
    }
  }
}

// A sensor's frame is drawn as three thick arrows with a dark (light mode) or white (dark mode)
// outline, arrow heads and x / y / z letters, a coloured dot and a name tag in the colour of the
// sensor type, and a dashed line back to the root of the link it is mounted on. A link frame is
// only three thin plain lines, so the two cannot be confused.
void RobotView::drawSensors(QPainter& p, int w, int h, bool dark) const {
  if (scene_.sensors.empty()) return;
  const double len = std::min(0.3, std::max(0.03, 0.18 * scene_.radius()));
  const QColor axisColours[3] = {kAxisX, kAxisY, kAxisZ};
  const char* axisNames[3] = {"x", "y", "z"};
  const QColor halo = dark ? QColor(255, 255, 255, 235) : QColor(24, 28, 34, 235);

  QFont labelFont = p.font();
  labelFont.setBold(true);
  labelFont.setPointSizeF(std::max(8.0, labelFont.pointSizeF()));
  const QFontMetricsF fm(labelFont);

  struct Tag {
    QRectF rect;
    QString text;
    QColor tint;
  };
  std::vector<Tag> tags;

  auto distanceToRect = [](const QRectF& r, const QPointF& q) {
    const double dx = std::max({r.left() - q.x(), 0.0, q.x() - r.right()});
    const double dy = std::max({r.top() - q.y(), 0.0, q.y() - r.bottom()});
    return std::hypot(dx, dy);
  };

  for (const urdfgen::SceneSensor& s : scene_.sensors) {
    const QColor tint = QColor::fromRgbF(s.color[0], s.color[1], s.color[2]);
    Camera::Screen origin;
    if (!camera_.project(s.pose.t, w, h, &origin)) continue;
    const QPointF o(origin.x, origin.y);

    // The way along the link from its root to the sensor.
    if (length(s.pose.t - s.root) > 1e-9) {
      Camera::Screen a, b;
      if (camera_.projectSegment(s.root, s.pose.t, w, h, &a, &b)) {
        p.setPen(QPen(halo, 3.6, Qt::SolidLine, Qt::RoundCap));
        p.drawLine(QPointF(a.x, a.y), QPointF(b.x, b.y));
        p.setPen(QPen(tint, 2.0, Qt::DashLine, Qt::RoundCap));
        p.drawLine(QPointF(a.x, a.y), QPointF(b.x, b.y));
      }
      Camera::Screen r;
      if (camera_.project(s.root, w, h, &r)) {
        p.setBrush(Qt::NoBrush);
        p.setPen(QPen(halo, 3.4));
        p.drawEllipse(QPointF(r.x, r.y), 4.0, 4.0);
        p.setPen(QPen(tint, 1.8));
        p.drawEllipse(QPointF(r.x, r.y), 4.0, 4.0);
      }
    }

    // The three axes, the farthest first so that nearer ones are drawn over them.
    struct Arrow {
      int axis;
      Camera::Screen tip;
    };
    std::vector<Arrow> arrows;
    for (int axis = 0; axis < 3; ++axis) {
      const Vec3 dir{s.pose.r[static_cast<std::size_t>(axis)], s.pose.r[static_cast<std::size_t>(3 + axis)],
                     s.pose.r[static_cast<std::size_t>(6 + axis)]};
      Arrow a;
      a.axis = axis;
      if (camera_.project(s.pose.t + dir * len, w, h, &a.tip)) arrows.push_back(a);
    }
    std::sort(arrows.begin(), arrows.end(), [](const Arrow& a, const Arrow& b) { return a.tip.depth > b.tip.depth; });
    std::vector<QPointF> keepClear;  // places the name tag should stay away from
    for (const Arrow& a : arrows) {
      const QPointF tip(a.tip.x, a.tip.y);
      const QPointF d = tip - o;
      const double pixels = std::hypot(d.x(), d.y());
      const QColor colour = axisColours[a.axis];
      keepClear.push_back(tip);
      if (pixels < 1.0) {  // pointing straight at or away from the camera: just a dot
        p.setPen(Qt::NoPen);
        p.setBrush(colour);
        p.drawEllipse(o, 3.0, 3.0);
        continue;
      }
      const QPointF u = d / pixels;
      const QPointF n(-u.y(), u.x());
      const double headLen = std::min(13.0, 0.6 * pixels), headHalf = 0.45 * headLen;
      const QPointF base = tip - u * headLen;
      keepClear.push_back(o + d * 0.5);
      keepClear.push_back(tip + u * 9.0);  // where the letter goes
      p.setBrush(Qt::NoBrush);
      p.setPen(QPen(halo, 6.5, Qt::SolidLine, Qt::RoundCap));
      p.drawLine(o, base);
      p.setPen(QPen(colour, 3.4, Qt::SolidLine, Qt::RoundCap));
      p.drawLine(o, base);
      const QPointF head[3] = {tip, base + n * headHalf, base - n * headHalf};
      p.setPen(QPen(halo, 1.8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
      p.setBrush(colour);
      p.drawPolygon(head, 3);

      QPainterPath letter;
      const QPointF at = tip + u * 9.0 - QPointF(fm.horizontalAdvance(axisNames[a.axis]) / 2.0, -fm.ascent() / 2.0 + 1.0);
      letter.addText(at, labelFont, axisNames[a.axis]);
      p.setBrush(Qt::NoBrush);
      p.setPen(QPen(halo, 3.2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
      p.drawPath(letter);
      p.fillPath(letter, colour);
    }

    // The origin of the sensor frame.
    p.setPen(QPen(halo, 2.0));
    p.setBrush(tint);
    p.drawEllipse(o, 5.0, 5.0);

    // Name tag, put on the side of the origin that is farthest from the arrows.
    const QString text = QString::fromStdString(s.name);
    const QSizeF size(fm.horizontalAdvance(text) + 12.0, fm.height() + 4.0);
    const double gap = 9.0;
    const QPointF anchors[8] = {{gap, -gap - size.height()},          // up right
                                {gap, gap},                           // down right
                                {-gap - size.width(), -gap - size.height()},  // up left
                                {-gap - size.width(), gap},           // down left
                                {-size.width() / 2, -gap - size.height()},   // above
                                {-size.width() / 2, gap},             // below
                                {gap, -size.height() / 2},            // right
                                {-gap - size.width(), -size.height() / 2}};  // left
    QRectF best;
    double bestScore = -1;
    for (const QPointF& anchor : anchors) {
      QRectF r(o + anchor, size);
      double score = 1e9;
      for (const QPointF& q : keepClear) score = std::min(score, distanceToRect(r, q));
      if (r.left() < 2 || r.top() < 2 || r.right() > w - 2 || r.bottom() > h - 2) score -= 1000;  // inside the view
      if (score > bestScore) {
        bestScore = score;
        best = r;
      }
    }
    best.translate(std::max(0.0, 2.0 - best.left()), std::max(0.0, 2.0 - best.top()));
    if (best.right() > w - 2) best.translate(w - 2 - best.right(), 0);
    if (best.bottom() > h - 2) best.translate(0, h - 2 - best.bottom());
    tags.push_back({best, text, tint});
  }

  // The tags last, so that no arrow is drawn over a name.
  p.setFont(labelFont);
  for (const Tag& t : tags) {
    p.setPen(QPen(halo, 1.6));
    p.setBrush(t.tint);
    p.drawRoundedRect(t.rect, 5, 5);
    p.setPen(t.tint.lightness() > 150 ? QColor(20, 24, 30) : QColor(255, 255, 255));
    p.drawText(t.rect, Qt::AlignCenter, t.text);
  }
}

void RobotView::drawGizmo(QPainter& p, int w, int h, bool dark) const {
  (void)w;
  Vec3 right, up, back;
  camera_.axes(&right, &up, &back);
  const QPointF origin(40, h - 40);
  const double len = 26;
  struct Axis {
    Vec3 dir;
    QColor colour;
    const char* label;
  };
  std::vector<Axis> axes = {{{1, 0, 0}, kAxisX, "X"}, {{0, 1, 0}, kAxisY, "Y"}, {{0, 0, 1}, kAxisZ, "Z"}};
  // Farthest axes first, so the nearer ones are drawn over them.
  std::sort(axes.begin(), axes.end(), [&](const Axis& a, const Axis& b) { return dot(a.dir, back) < dot(b.dir, back); });
  QFont font = p.font();
  font.setBold(true);
  p.setFont(font);
  for (const Axis& a : axes) {
    const QPointF tip = origin + QPointF(dot(a.dir, right), -dot(a.dir, up)) * len;
    p.setPen(QPen(a.colour, 2.2));
    p.drawLine(origin, tip);
    p.setPen(a.colour);
    const QPointF label = origin + QPointF(dot(a.dir, right), -dot(a.dir, up)) * (len + 9);
    p.drawText(QRectF(label.x() - 8, label.y() - 8, 16, 16), Qt::AlignCenter, a.label);
  }
  p.setPen(Qt::NoPen);
  p.setBrush(dark ? QColor(255, 255, 255, 140) : QColor(0, 0, 0, 120));
  p.drawEllipse(origin, 2.0, 2.0);
}

}  // namespace urdfgui
