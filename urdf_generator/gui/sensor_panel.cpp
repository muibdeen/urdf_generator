// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include "sensor_panel.hpp"

#include <QColor>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPalette>
#include <QPointer>
#include <QPushButton>
#include <QRegularExpression>
#include <QRegularExpressionValidator>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include <set>

namespace urdfgui {
namespace {

using namespace urdfgen;

constexpr int kTicks = 1000;  // slider resolution across the length of a link

// Spin boxes and combo boxes ignore the mouse wheel unless they have focus, so scrolling the
// page does not change a value by accident.
template <class T>
class NoWheel : public T {
 public:
  NoWheel() { this->setFocusPolicy(Qt::StrongFocus); }

 protected:
  void wheelEvent(QWheelEvent* e) override {
    if (this->hasFocus()) {
      T::wheelEvent(e);
    } else {
      e->ignore();
    }
  }
};
using Spin = NoWheel<QDoubleSpinBox>;
using Combo = NoWheel<QComboBox>;

QDoubleSpinBox* makeSpin(double lo, double hi, int decimals, double step, const QString& prefix,
                         const QString& suffix) {
  auto* s = new Spin;
  s->setRange(lo, hi);
  s->setDecimals(decimals);
  s->setSingleStep(step);
  s->setPrefix(prefix);
  s->setSuffix(suffix);
  s->setAlignment(Qt::AlignRight);
  return s;
}

QString typeLabel(SensorType t) { return QString::fromLatin1(sensorLabel(t)); }

/// "camera" and "camera_2" are the names the panel makes up for a camera.
bool isGeneratedName(const QString& name, SensorType type) {
  const QString id = QString::fromLatin1(sensorId(type));
  if (name == id) return true;
  if (!name.startsWith(id + "_")) return false;
  const QString rest = name.mid(id.size() + 1);
  if (rest.isEmpty()) return false;
  for (QChar c : rest) {
    if (!c.isDigit()) return false;
  }
  return true;
}

}  // namespace

struct SensorPanel::Row {
  QGroupBox* box = nullptr;
  QLineEdit* name = nullptr;
  QComboBox* type = nullptr;
  QComboBox* link = nullptr;
  QSlider* slider = nullptr;
  QDoubleSpinBox* distance = nullptr;
  QDoubleSpinBox *offX = nullptr, *offY = nullptr, *offZ = nullptr;
  QDoubleSpinBox *roll = nullptr, *pitch = nullptr, *yaw = nullptr;
  QPushButton* remove = nullptr;

  SensorType kind = SensorType::Camera;  // the type shown in `type`
  QString linkName;                      // the link the sensor is on, even if the robot lost it
  double wanted = 0.0;                   // the distance the user asked for (the spin box may show less)
};

SensorPanel::SensorPanel(QWidget* parent) : QGroupBox("Sensors", parent) {
  setObjectName("sensor_panel");
  auto* v = new QVBoxLayout(this);

  auto* note = new QLabel(
      "A sensor is a small link on a fixed joint. It is mounted at the root of a link (the origin of "
      "that link's frame) and slides along it. Its own frame points x forward, y left, z up; "
      "the 3D view draws it with thick, arrow-headed axes.");
  note->setWordWrap(true);
  QPalette notePalette = note->palette();
  notePalette.setColor(QPalette::WindowText, notePalette.color(QPalette::PlaceholderText));
  note->setPalette(notePalette);
  v->addWidget(note);

  auto* add = new QHBoxLayout;
  add->addWidget(new QLabel("Add a"));
  addType_ = new Combo;
  addType_->setObjectName("sensor_add_type");
  for (SensorType t : allSensorTypes()) addType_->addItem(typeLabel(t), static_cast<int>(t));
  add->addWidget(addType_, 1);
  addButton_ = new QPushButton("Add sensor");
  addButton_->setObjectName("sensor_add");
  addButton_->setAutoDefault(false);
  addButton_->setToolTip("Mount a new sensor on top of the main body; then pick its link and position");
  add->addWidget(addButton_);
  v->addLayout(add);
  connect(addButton_, &QPushButton::clicked, this, [this]() {
    emit addRequested(static_cast<SensorType>(addType_->currentData().toInt()));
  });

  empty_ = new QLabel("No sensors.");
  empty_->setObjectName("no_sensors");
  v->addWidget(empty_);

  rowsLayout_ = new QVBoxLayout;
  v->addLayout(rowsLayout_);
}

SensorPanel::~SensorPanel() = default;

// ---------------------------------------------------------------------- lookups

SensorPanel::Row* SensorPanel::rowOf(const QObject* box) const {
  for (const auto& r : rows_) {
    if (r->box == box) return r.get();
  }
  return nullptr;  // a row that was removed
}

int SensorPanel::indexOf(const Row* row) const {
  for (std::size_t i = 0; i < rows_.size(); ++i) {
    if (rows_[i].get() == row) return static_cast<int>(i);
  }
  return -1;
}

std::vector<SensorConfig> SensorPanel::sensors() const {
  std::vector<SensorConfig> out;
  for (const auto& r : rows_) {
    SensorConfig s;
    s.type = r->kind;
    s.name = r->name->text().toStdString();
    s.link = r->linkName.toStdString();
    s.distance = r->distance->value();
    s.offsetX = r->offX->value();
    s.offsetY = r->offY->value();
    s.offsetZ = r->offZ->value();
    s.rollDeg = r->roll->value();
    s.pitchDeg = r->pitch->value();
    s.yawDeg = r->yaw->value();
    out.push_back(s);
  }
  return out;
}

// --------------------------------------------------------------------- the robot

void SensorPanel::setRobot(const Robot& base) {
  links_ = attachableLinks(base);
  axes_.clear();
  for (const Link& l : base.flatten().links) axes_.emplace_back(l.name, linkAxis(l));
  for (const auto& r : rows_) refreshRow(*r);
}

void SensorPanel::refreshRow(Row& row) {
  // The link list: the robot's links, plus the sensor's own link if the robot no longer has it
  // (the sensor keeps it, and the generator reports the problem).
  QStringList wanted;
  for (const std::string& l : links_) wanted << QString::fromStdString(l);
  const bool known = wanted.contains(row.linkName);
  if (!known && !row.linkName.isEmpty()) wanted << row.linkName;

  QStringList have;
  for (int i = 0; i < row.link->count(); ++i) have << row.link->itemText(i);
  {
    const QSignalBlocker blocker(row.link);
    if (have != wanted) {
      row.link->clear();
      row.link->addItems(wanted);
    }
    row.link->setCurrentIndex(wanted.indexOf(row.linkName));
    if (!known && !row.linkName.isEmpty()) {
      const int i = wanted.indexOf(row.linkName);
      row.link->setItemData(i, QColor("#d1242f"), Qt::ForegroundRole);
      row.link->setItemData(i, "The robot has no link with this name any more", Qt::ToolTipRole);
    }
    row.link->setToolTip(known ? QString() : "The robot has no link with this name any more");
  }
  applyRange(row);
}

void SensorPanel::applyRange(Row& row) {
  LinkAxis axis;
  const std::string name = row.linkName.toStdString();
  for (const auto& a : axes_) {
    if (a.first == name) axis = a.second;
  }
  const bool slides = axis.upper - axis.lower > 1e-9;
  const double value = std::max(axis.lower, std::min(axis.upper, row.wanted));
  const QSignalBlocker b1(row.distance), b2(row.slider);
  row.distance->setRange(axis.lower, axis.upper);
  row.distance->setValue(value);
  row.slider->setValue(slides ? static_cast<int>(std::lround((value - axis.lower) / (axis.upper - axis.lower) * kTicks)) : 0);
  row.distance->setEnabled(slides);
  row.slider->setEnabled(slides);
  row.distance->setToolTip(
      slides ? QString("From the root of the link (0) along the link: %1 to %2 m.\n"
                       "A rod runs from its root to its tip; a wheel or a chassis is centred on its root, so "
                       "its range reaches both ways.")
                   .arg(axis.lower, 0, 'f', 3)
                   .arg(axis.upper, 0, 'f', 3)
             : QString("This link has no length to slide along; use the offset."));
}

// ------------------------------------------------------------------- the rows

SensorPanel::Row* SensorPanel::newRow(const SensorConfig& s) {
  auto owned = std::make_unique<Row>();
  Row* row = owned.get();
  row->kind = s.type;
  row->linkName = QString::fromStdString(s.link);
  row->wanted = s.distance;

  row->box = new QGroupBox;
  auto* grid = new QGridLayout(row->box);
  grid->setColumnStretch(1, 1);
  grid->setColumnStretch(2, 1);
  grid->setColumnStretch(3, 1);

  row->name = new QLineEdit(QString::fromStdString(s.name));
  row->name->setValidator(new QRegularExpressionValidator(QRegularExpression("[A-Za-z0-9_]*"), row->name));
  row->name->setToolTip("Letters, digits and '_'. The sensor link is <name>_link and its joint <name>_joint.");
  row->remove = new QPushButton("Remove");
  row->remove->setAutoDefault(false);
  row->remove->setToolTip("Delete this sensor");
  grid->addWidget(new QLabel("Name"), 0, 0);
  grid->addWidget(row->name, 0, 1, 1, 2);
  grid->addWidget(row->remove, 0, 3);

  row->type = new Combo;
  const std::vector<SensorType> kinds = allSensorTypes();
  for (SensorType t : kinds) row->type->addItem(typeLabel(t), static_cast<int>(t));
  row->type->setCurrentIndex(static_cast<int>(std::find(kinds.begin(), kinds.end(), s.type) - kinds.begin()));
  grid->addWidget(new QLabel("Type"), 1, 0);
  grid->addWidget(row->type, 1, 1, 1, 3);

  row->link = new Combo;
  row->link->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
  row->link->setMinimumContentsLength(12);
  row->link->setToolTip(QString());
  grid->addWidget(new QLabel("Mounted on"), 2, 0);
  grid->addWidget(row->link, 2, 1, 1, 3);

  row->slider = new QSlider(Qt::Horizontal);
  row->slider->setRange(0, kTicks);
  row->distance = makeSpin(0, 0, 3, 0.005, QString(), " m");
  grid->addWidget(new QLabel("Distance"), 3, 0);
  grid->addWidget(row->slider, 3, 1, 1, 2);
  grid->addWidget(row->distance, 3, 3);

  row->offX = makeSpin(-10, 10, 3, 0.005, "x ", " m");
  row->offY = makeSpin(-10, 10, 3, 0.005, "y ", " m");
  row->offZ = makeSpin(-10, 10, 3, 0.005, "z ", " m");
  grid->addWidget(new QLabel("Offset"), 4, 0);
  grid->addWidget(row->offX, 4, 1);
  grid->addWidget(row->offY, 4, 2);
  grid->addWidget(row->offZ, 4, 3);
  const QString offsetTip = "Extra shift in the frame of the link the sensor is mounted on, on top of the distance.";
  for (QWidget* w : {static_cast<QWidget*>(row->offX), static_cast<QWidget*>(row->offY), static_cast<QWidget*>(row->offZ)}) {
    w->setToolTip(offsetTip);
  }

  row->roll = makeSpin(-360, 360, 1, 5.0, "r ", " °");
  row->pitch = makeSpin(-360, 360, 1, 5.0, "p ", " °");
  row->yaw = makeSpin(-360, 360, 1, 5.0, "y ", " °");
  grid->addWidget(new QLabel("Rotation"), 5, 0);
  grid->addWidget(row->roll, 5, 1);
  grid->addWidget(row->pitch, 5, 2);
  grid->addWidget(row->yaw, 5, 3);
  const QString rotationTip =
      "Roll, pitch and yaw of the sensor frame relative to the frame of its link (URDF rpy, degrees). "
      "At zero the sensor's x axis points the same way as the link's x axis.";
  for (QWidget* w : {static_cast<QWidget*>(row->roll), static_cast<QWidget*>(row->pitch), static_cast<QWidget*>(row->yaw)}) {
    w->setToolTip(rotationTip);
  }

  row->offX->setValue(s.offsetX);
  row->offY->setValue(s.offsetY);
  row->offZ->setValue(s.offsetZ);
  row->roll->setValue(s.rollDeg);
  row->pitch->setValue(s.pitchDeg);
  row->yaw->setValue(s.yawDeg);

  // ---- connections (made after the initial values, so building a row emits nothing) ----
  QGroupBox* box = row->box;
  connect(row->name, &QLineEdit::textChanged, this, [this, box]() {
    if (Row* r = rowOf(box)) {
      updateTitle(*r);
      emit changed();
    }
  });
  connect(row->type, &QComboBox::currentIndexChanged, this, [this, box](int index) {
    Row* r = rowOf(box);
    if (!r || index < 0) return;
    const SensorType old = r->kind;
    r->kind = static_cast<SensorType>(r->type->itemData(index).toInt());
    if (isGeneratedName(r->name->text(), old)) {  // keep "camera" from being a LiDAR
      std::set<QString> taken;
      for (const auto& other : rows_) {
        if (other.get() != r) taken.insert(other->name->text());
      }
      const QString base = QString::fromLatin1(sensorId(r->kind));
      QString name = base;
      for (int n = 2; taken.count(name); ++n) name = base + "_" + QString::number(n);
      const QSignalBlocker blocker(r->name);
      r->name->setText(name);
    }
    updateTitle(*r);
    emit changed();
  });
  connect(row->link, &QComboBox::currentIndexChanged, this, [this, box](int index) {
    Row* r = rowOf(box);
    if (!r || index < 0) return;
    r->linkName = r->link->itemText(index);
    // Moving to another link puts the sensor at that link's root; the rotation is kept.
    r->wanted = 0.0;
    {
      const QSignalBlocker a(r->offX), b(r->offY), c(r->offZ);
      r->offX->setValue(0.0);
      r->offY->setValue(0.0);
      r->offZ->setValue(0.0);
    }
    applyRange(*r);
    emit changed();
  });
  connect(row->slider, &QSlider::valueChanged, this, [this, box](int tick) {
    Row* r = rowOf(box);
    if (!r) return;
    const double value = r->distance->minimum() + (r->distance->maximum() - r->distance->minimum()) * tick / kTicks;
    r->wanted = value;
    {
      const QSignalBlocker blocker(r->distance);
      r->distance->setValue(value);
    }
    emit changed();
  });
  connect(row->distance, &QDoubleSpinBox::valueChanged, this, [this, box](double value) {
    Row* r = rowOf(box);
    if (!r) return;
    r->wanted = value;
    const double lo = r->distance->minimum(), hi = r->distance->maximum();
    {
      const QSignalBlocker blocker(r->slider);
      r->slider->setValue(hi > lo ? static_cast<int>(std::lround((value - lo) / (hi - lo) * kTicks)) : 0);
    }
    emit changed();
  });
  for (QDoubleSpinBox* spin : {row->offX, row->offY, row->offZ, row->roll, row->pitch, row->yaw}) {
    connect(spin, &QDoubleSpinBox::valueChanged, this, [this]() { emit changed(); });
  }
  connect(row->remove, &QPushButton::clicked, this, [this, box]() {
    if (Row* r = rowOf(box)) removeAt(indexOf(r));
  });

  updateTitle(*row);
  rows_.push_back(std::move(owned));
  rowsLayout_->addWidget(row->box);
  return row;
}

void SensorPanel::updateTitle(Row& row) {
  const int index = indexOf(&row);
  row.box->setTitle(QString("%1  ·  %2  ·  %3")
                        .arg(index < 0 ? static_cast<int>(rows_.size()) + 1 : index + 1)
                        .arg(typeLabel(row.kind), row.name->text()));
}

void SensorPanel::renumber() {
  for (std::size_t i = 0; i < rows_.size(); ++i) {
    Row& r = *rows_[i];
    const QString p = QString("sensor%1").arg(i + 1);
    r.box->setObjectName(p);
    r.name->setObjectName(p + "_name");
    r.type->setObjectName(p + "_type");
    r.link->setObjectName(p + "_link");
    r.slider->setObjectName(p + "_distance_slider");
    r.distance->setObjectName(p + "_distance");
    r.offX->setObjectName(p + "_off_x");
    r.offY->setObjectName(p + "_off_y");
    r.offZ->setObjectName(p + "_off_z");
    r.roll->setObjectName(p + "_roll");
    r.pitch->setObjectName(p + "_pitch");
    r.yaw->setObjectName(p + "_yaw");
    r.remove->setObjectName(p + "_remove");
    updateTitle(r);
  }
}

void SensorPanel::updateEmptyLabel() { empty_->setVisible(rows_.empty()); }

void SensorPanel::showRow(Row& row) {
  // Bring the new sensor into view when the page is inside a scroll area.
  QPointer<QWidget> box = row.box;
  QTimer::singleShot(50, this, [box]() {  // after the page has been laid out again
    if (!box) return;
    for (QWidget* w = box->parentWidget(); w; w = w->parentWidget()) {
      if (auto* area = qobject_cast<QScrollArea*>(w)) {
        area->ensureWidgetVisible(box, 0, 20);
        return;
      }
    }
  });
}

void SensorPanel::append(const SensorConfig& sensor) {
  Row* row = newRow(sensor);
  renumber();
  refreshRow(*row);
  updateEmptyLabel();
  showRow(*row);
  emit changed();
}

void SensorPanel::removeAt(int index) {
  if (index < 0 || index >= count()) return;
  std::unique_ptr<Row> gone = std::move(rows_[static_cast<std::size_t>(index)]);
  rows_.erase(rows_.begin() + index);
  QGroupBox* box = gone->box;
  rowsLayout_->removeWidget(box);
  box->hide();
  box->setParent(nullptr);  // no longer found as a child of the page
  box->deleteLater();
  renumber();
  updateEmptyLabel();
  emit changed();
}

void SensorPanel::clear() {
  const bool any = !rows_.empty();
  while (!rows_.empty()) {
    std::unique_ptr<Row> gone = std::move(rows_.back());
    rows_.pop_back();
    gone->box->hide();
    gone->box->setParent(nullptr);
    gone->box->deleteLater();
  }
  updateEmptyLabel();
  if (any) emit changed();
}

}  // namespace urdfgui
