// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include "urdf_generator/render.hpp"

#include <algorithm>
#include <cmath>

namespace urdfgen {
namespace {

constexpr double kMaxElevation = 89.5;

double clampElevation(double deg) { return std::max(-kMaxElevation, std::min(kMaxElevation, deg)); }

struct CamVec {
  double x, y, z;
};

CamVec toCamera(const Vec3& d, const Vec3& right, const Vec3& up, const Vec3& back) {
  return {dot(d, right), dot(d, up), dot(d, back)};
}

}  // namespace

Vec3 Camera::eye() const {
  const double az = azimuthDeg * kPi / 180.0, el = clampElevation(elevationDeg) * kPi / 180.0;
  return target + Vec3{std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)} * distance;
}

void Camera::axes(Vec3* right, Vec3* up, Vec3* back) const {
  const double az = azimuthDeg * kPi / 180.0, el = clampElevation(elevationDeg) * kPi / 180.0;
  const Vec3 b{std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)};
  const Vec3 r = normalized(cross(Vec3{0, 0, 1}, b));
  *back = b;
  *right = r;
  *up = cross(b, r);
}

bool Camera::project(const Vec3& p, int width, int height, Screen* out) const {
  Vec3 right, up, back;
  axes(&right, &up, &back);
  const CamVec c = toCamera(p - eye(), right, up, back);
  if (c.z > -nearPlane()) return false;
  const double f = 1.0 / std::tan(fovDeg * kPi / 360.0);
  const double aspect = static_cast<double>(width) / std::max(1, height);
  const double iw = 1.0 / -c.z;
  out->x = width * (0.5 + 0.5 * f / aspect * c.x * iw);
  out->y = height * (0.5 - 0.5 * f * c.y * iw);
  out->depth = -c.z;
  return true;
}

bool Camera::projectSegment(const Vec3& a, const Vec3& b, int width, int height, Screen* sa,
                            Screen* sb) const {
  Vec3 right, up, back;
  axes(&right, &up, &back);
  const Vec3 e = eye();
  const double zNear = -nearPlane();
  CamVec ca = toCamera(a - e, right, up, back), cb = toCamera(b - e, right, up, back);
  const bool aIn = ca.z <= zNear, bIn = cb.z <= zNear;
  if (!aIn && !bIn) return false;
  auto lerpTo = [&](const CamVec& from, const CamVec& to) {  // `from` is behind the near plane
    const double t = (zNear - from.z) / (to.z - from.z);
    return CamVec{from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t, zNear};
  };
  if (!aIn) ca = lerpTo(ca, cb);
  if (!bIn) cb = lerpTo(cb, ca);
  const double f = 1.0 / std::tan(fovDeg * kPi / 360.0);
  const double aspect = static_cast<double>(width) / std::max(1, height);
  auto screen = [&](const CamVec& c, Screen* s) {
    const double iw = 1.0 / -c.z;
    s->x = width * (0.5 + 0.5 * f / aspect * c.x * iw);
    s->y = height * (0.5 - 0.5 * f * c.y * iw);
    s->depth = -c.z;
  };
  screen(ca, sa);
  screen(cb, sb);
  return true;
}

bool Camera::contains(const Scene& scene, int width, int height, double marginPx) const {
  if (scene.empty()) return true;
  for (int i = 0; i < 8; ++i) {
    const Vec3 corner{(i & 1) ? scene.max.x : scene.min.x, (i & 2) ? scene.max.y : scene.min.y,
                      (i & 4) ? scene.max.z : scene.min.z};
    Screen s;
    if (!project(corner, width, height, &s)) return false;
    if (s.x < marginPx || s.y < marginPx || s.x > width - marginPx || s.y > height - marginPx) return false;
  }
  return true;
}

Camera Camera::fit(const Scene& scene, double aspect, double azimuthDeg, double elevationDeg) {
  Camera c;
  c.azimuthDeg = azimuthDeg;
  c.elevationDeg = elevationDeg;
  c.target = scene.empty() ? Vec3() : scene.center();
  const double radius = std::max(scene.empty() ? 0.0 : scene.radius(), 1e-3);
  // The bounding sphere must fit the narrower of the two field-of-view angles.
  const double half = std::atan(std::tan(c.fovDeg * kPi / 360.0) * std::min(1.0, std::max(aspect, 0.05)));
  c.distance = radius / std::sin(half) * 1.3;  // safely too far away
  if (scene.empty()) return c;

  // The sphere is a loose bound. Move in as far as the bounding box, seen from this direction,
  // still fits inside the viewport with a margin.
  const int h = 1000;
  const int w = std::max(1, static_cast<int>(std::lround(h * std::max(aspect, 0.05))));
  const double margin = 0.07 * std::min(w, h);
  if (!c.contains(scene, w, h, margin)) return c;
  const double startDistance = c.distance;
  for (int pass = 0; pass < 4; ++pass) {
    double lo = startDistance * 0.05, hi = startDistance * 2.0;
    for (int i = 0; i < 40; ++i) {
      c.distance = 0.5 * (lo + hi);
      if (c.contains(scene, w, h, margin)) {
        hi = c.distance;
      } else {
        lo = c.distance;
      }
    }
    c.distance = hi;

    // In perspective the box does not project symmetrically about the target, so shift the
    // target until the projected box is centred, then fit the distance again.
    double minX = 1e18, maxX = -1e18, minY = 1e18, maxY = -1e18;
    for (int i = 0; i < 8; ++i) {
      const Vec3 corner{(i & 1) ? scene.max.x : scene.min.x, (i & 2) ? scene.max.y : scene.min.y,
                        (i & 4) ? scene.max.z : scene.min.z};
      Screen s;
      if (!c.project(corner, w, h, &s)) return c;
      minX = std::min(minX, s.x); maxX = std::max(maxX, s.x);
      minY = std::min(minY, s.y); maxY = std::max(maxY, s.y);
    }
    const double dx = 0.5 * (minX + maxX) - 0.5 * w, dy = 0.5 * (minY + maxY) - 0.5 * h;
    if (std::fabs(dx) < 0.5 && std::fabs(dy) < 0.5) break;
    Vec3 right, up, back;
    c.axes(&right, &up, &back);
    const double perPixel = 2.0 * c.distance * std::tan(c.fovDeg * kPi / 360.0) / h;
    c.target = c.target + right * (dx * perPixel) - up * (dy * perPixel);
  }
  return c;
}

// ------------------------------------------------------------------ rasteriser

namespace {

struct CV {  // camera-space vertex
  CamVec p;
  CamVec n;
};

struct SV {  // screen-space vertex; the normal is premultiplied by 1/w for perspective-correct interpolation
  float x, y, iw;
  float nx, ny, nz;
};

CV lerp(const CV& a, const CV& b, double t) {
  return {{a.p.x + (b.p.x - a.p.x) * t, a.p.y + (b.p.y - a.p.y) * t, a.p.z + (b.p.z - a.p.z) * t},
          {a.n.x + (b.n.x - a.n.x) * t, a.n.y + (b.n.y - a.n.y) * t, a.n.z + (b.n.z - a.n.z) * t}};
}

struct Target {
  int width, height;
  std::vector<std::uint32_t>* pixels;
  std::vector<float>* zbuf;
  float light[3];
};

void rasterize(const Target& tg, const SV& a, const SV& b, const SV& c, const float rgb[3]) {
  const float area = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
  // Counter-clockwise (front-facing) triangles have a negative area in y-down screen space.
  if (!(area < -1e-9f)) return;
  const int x0 = std::max(0, static_cast<int>(std::floor(std::min({a.x, b.x, c.x}))));
  const int x1 = std::min(tg.width - 1, static_cast<int>(std::ceil(std::max({a.x, b.x, c.x}))));
  const int y0 = std::max(0, static_cast<int>(std::floor(std::min({a.y, b.y, c.y}))));
  const int y1 = std::min(tg.height - 1, static_cast<int>(std::ceil(std::max({a.y, b.y, c.y}))));
  const float invArea = 1.0f / area;
  constexpr float kEps = -1e-5f;
  for (int y = y0; y <= y1; ++y) {
    const float py = y + 0.5f;
    for (int x = x0; x <= x1; ++x) {
      const float px = x + 0.5f;
      const float w0 = ((b.x - px) * (c.y - py) - (c.x - px) * (b.y - py)) * invArea;
      const float w1 = ((c.x - px) * (a.y - py) - (a.x - px) * (c.y - py)) * invArea;
      const float w2 = 1.0f - w0 - w1;
      if (w0 < kEps || w1 < kEps || w2 < kEps) continue;
      const float iw = w0 * a.iw + w1 * b.iw + w2 * c.iw;
      const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(tg.width) + static_cast<std::size_t>(x);
      if (iw <= (*tg.zbuf)[idx]) continue;
      (*tg.zbuf)[idx] = iw;

      float nx = w0 * a.nx + w1 * b.nx + w2 * c.nx;
      float ny = w0 * a.ny + w1 * b.ny + w2 * c.ny;
      float nz = w0 * a.nz + w1 * b.nz + w2 * c.nz;
      const float len = std::sqrt(nx * nx + ny * ny + nz * nz);
      float diffuse = 0.0f;
      if (len > 1e-12f) {
        diffuse = std::max(0.0f, (nx * tg.light[0] + ny * tg.light[1] + nz * tg.light[2]) / len);
      }
      const float k = 0.38f + 0.62f * diffuse;
      auto channel = [&](float v) {
        return static_cast<std::uint32_t>(std::min(255.0f, std::max(0.0f, v * k * 255.0f + 0.5f)));
      };
      (*tg.pixels)[idx] = 0xFF000000u | (channel(rgb[0]) << 16) | (channel(rgb[1]) << 8) | channel(rgb[2]);
    }
  }
}

}  // namespace

Image renderScene(const Scene& scene, const Camera& cam, int width, int height) {
  Image img;
  img.width = std::max(0, width);
  img.height = std::max(0, height);
  img.pixels.assign(static_cast<std::size_t>(img.width) * static_cast<std::size_t>(img.height), 0u);
  if (img.width < 2 || img.height < 2) return img;

  std::vector<float> zbuf(img.pixels.size(), 0.0f);
  Vec3 right, up, back;
  cam.axes(&right, &up, &back);
  const Vec3 eye = cam.eye();
  const double zNear = -cam.nearPlane();
  const double f = 1.0 / std::tan(cam.fovDeg * kPi / 360.0);
  const double aspect = static_cast<double>(img.width) / img.height;

  Target tg{img.width, img.height, &img.pixels, &zbuf, {0, 0, 0}};
  const Vec3 L = normalized(Vec3{-0.35, 0.55, 0.75});  // towards the light, in camera space
  tg.light[0] = static_cast<float>(L.x);
  tg.light[1] = static_cast<float>(L.y);
  tg.light[2] = static_cast<float>(L.z);

  auto toScreen = [&](const CV& v) {
    const double iw = 1.0 / -v.p.z;
    SV s;
    s.x = static_cast<float>(img.width * (0.5 + 0.5 * f / aspect * v.p.x * iw));
    s.y = static_cast<float>(img.height * (0.5 - 0.5 * f * v.p.y * iw));
    s.iw = static_cast<float>(iw);
    s.nx = static_cast<float>(v.n.x * iw);
    s.ny = static_cast<float>(v.n.y * iw);
    s.nz = static_cast<float>(v.n.z * iw);
    return s;
  };

  std::vector<CV> cv;
  for (const SceneBody& body : scene.bodies) {
    const Mesh& m = body.mesh;
    cv.resize(m.vertices.size());
    for (std::size_t i = 0; i < m.vertices.size(); ++i) {
      cv[i].p = toCamera(m.vertices[i] - eye, right, up, back);
      cv[i].n = toCamera(m.normals[i], right, up, back);
    }
    for (const auto& tri : m.triangles) {
      const CV v[3] = {cv[tri[0]], cv[tri[1]], cv[tri[2]]};
      const bool in[3] = {v[0].p.z <= zNear, v[1].p.z <= zNear, v[2].p.z <= zNear};
      if (!in[0] && !in[1] && !in[2]) continue;

      if (in[0] && in[1] && in[2]) {
        rasterize(tg, toScreen(v[0]), toScreen(v[1]), toScreen(v[2]), body.color.data());
        continue;
      }
      // Clip against the near plane (Sutherland-Hodgman), then fan-triangulate.
      CV poly[4];
      int count = 0;
      for (int i = 0; i < 3; ++i) {
        const CV& cur = v[i];
        const CV& nxt = v[(i + 1) % 3];
        const bool curIn = in[i], nxtIn = in[(i + 1) % 3];
        if (curIn) poly[count++] = cur;
        if (curIn != nxtIn) poly[count++] = lerp(cur, nxt, (zNear - cur.p.z) / (nxt.p.z - cur.p.z));
      }
      for (int k = 1; k + 1 < count; ++k) {
        rasterize(tg, toScreen(poly[0]), toScreen(poly[k]), toScreen(poly[k + 1]), body.color.data());
      }
    }
  }
  return img;
}

}  // namespace urdfgen
