// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// A small software renderer for the 3D view: an orbit camera and a z-buffered
// triangle rasteriser with per-pixel lighting. It needs no GPU or OpenGL, so it
// works over SSH, in VMs and in headless tests. No Qt in here.
#pragma once

#include <cstdint>
#include <vector>

#include "urdf_generator/scene.hpp"

namespace urdfgen {

/// Orbit camera, z up. At azimuth 0 / elevation 0 the camera sits on +x looking
/// back at the target, so the screen's right is +y.
struct Camera {
  Vec3 target;
  double distance = 1.0;
  double azimuthDeg = -50.0;
  double elevationDeg = 25.0;  // limited to +/-89.5
  double fovDeg = 35.0;        // vertical field of view

  Vec3 eye() const;
  /// Camera basis in world coordinates: screen right, screen up, and back (towards the camera).
  void axes(Vec3* right, Vec3* up, Vec3* back) const;
  double nearPlane() const { return distance * 0.02; }

  struct Screen {
    double x = 0, y = 0;  // pixels, origin top-left
    double depth = 0;     // distance in front of the camera
  };
  /// False if the point is behind the near plane.
  bool project(const Vec3& p, int width, int height, Screen* out) const;
  /// Projects a line segment, clipping it against the near plane. False if nothing is visible.
  bool projectSegment(const Vec3& a, const Vec3& b, int width, int height, Screen* sa,
                      Screen* sb) const;
  /// True if all eight corners of the scene's bounding box are inside the viewport.
  bool contains(const Scene& scene, int width, int height, double marginPx = 0) const;

  /// A camera that frames the whole scene for a viewport with the given aspect ratio (w / h).
  static Camera fit(const Scene& scene, double aspect, double azimuthDeg, double elevationDeg);
};

/// Premultiplied ARGB (0xAARRGGBB); 0 means "nothing drawn here".
struct Image {
  int width = 0, height = 0;
  std::vector<std::uint32_t> pixels;
};

Image renderScene(const Scene& scene, const Camera& camera, int width, int height);

}  // namespace urdfgen
