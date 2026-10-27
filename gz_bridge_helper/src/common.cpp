// Copyright 2026. Licensed under the Apache License, Version 2.0.
#include "gz_bridge_helper/common.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>

namespace gzb {

const char* severityName(Severity s) {
  switch (s) {
    case Severity::Info: return "info";
    case Severity::Warning: return "warning";
    case Severity::Error: return "error";
  }
  return "info";
}

void addIssue(Issues& issues, Severity s, const std::string& where, const std::string& message) {
  issues.push_back({s, where, message});
}

bool hasErrors(const Issues& issues) { return countIssues(issues, Severity::Error) > 0; }

int countIssues(const Issues& issues, Severity s) {
  return static_cast<int>(std::count_if(issues.begin(), issues.end(), [s](const Issue& i) { return i.severity == s; }));
}

// ------------------------------------------------------------------------------ geometry

Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator*(const Vec3& a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
double norm(const Vec3& a) { return std::sqrt(dot(a, a)); }
Vec3 normalized(const Vec3& a) {
  const double n = norm(a);
  return n > 0 ? a * (1.0 / n) : Vec3{};
}

Transform Transform::fromXyzRpy(const Vec3& xyz, const Vec3& rpy) {
  const double cr = std::cos(rpy.x), sr = std::sin(rpy.x);
  const double cp = std::cos(rpy.y), sp = std::sin(rpy.y);
  const double cy = std::cos(rpy.z), sy = std::sin(rpy.z);
  Transform T;
  T.r[0][0] = cy * cp;
  T.r[0][1] = cy * sp * sr - sy * cr;
  T.r[0][2] = cy * sp * cr + sy * sr;
  T.r[1][0] = sy * cp;
  T.r[1][1] = sy * sp * sr + cy * cr;
  T.r[1][2] = sy * sp * cr - cy * sr;
  T.r[2][0] = -sp;
  T.r[2][1] = cp * sr;
  T.r[2][2] = cp * cr;
  T.t = xyz;
  return T;
}

Vec3 Transform::rotate(const Vec3& v) const {
  return {r[0][0] * v.x + r[0][1] * v.y + r[0][2] * v.z, r[1][0] * v.x + r[1][1] * v.y + r[1][2] * v.z,
          r[2][0] * v.x + r[2][1] * v.y + r[2][2] * v.z};
}

Vec3 Transform::apply(const Vec3& p) const { return rotate(p) + t; }

Transform Transform::operator*(const Transform& o) const {
  Transform out;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out.r[i][j] = r[i][0] * o.r[0][j] + r[i][1] * o.r[1][j] + r[i][2] * o.r[2][j];
    }
  }
  out.t = rotate(o.t) + t;
  return out;
}

// ------------------------------------------------------------------------------ strings

std::string trim(const std::string& s) {
  size_t a = 0, b = s.size();
  while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
  return s.substr(a, b - a);
}

std::string lower(const std::string& s) {
  std::string out = s;
  for (char& c : out) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return out;
}

std::vector<std::string> split(const std::string& s, char sep) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == sep) {
      out.push_back(cur);
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  out.push_back(cur);
  return out;
}

bool startsWith(const std::string& s, const std::string& p) { return s.size() >= p.size() && s.compare(0, p.size(), p) == 0; }
bool endsWith(const std::string& s, const std::string& p) {
  return s.size() >= p.size() && s.compare(s.size() - p.size(), p.size(), p) == 0;
}

std::string num(double v) {
  if (std::fabs(v) < 1e-12) return "0";
  char buf[64];
  std::snprintf(buf, sizeof buf, "%.9g", v);
  return buf;
}

std::string xmlEscape(const std::string& s) {
  std::string out;
  for (char c : s) {
    switch (c) {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '"': out += "&quot;"; break;
      default: out.push_back(c);
    }
  }
  return out;
}

bool validIdentifier(const std::string& s) {
  if (s.empty() || std::isdigit(static_cast<unsigned char>(s[0]))) return false;
  for (char c : s) {
    if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_')) return false;
  }
  return true;
}

int editDistance(const std::string& a, const std::string& b) {
  std::vector<int> prev(b.size() + 1), cur(b.size() + 1);
  for (size_t j = 0; j <= b.size(); ++j) prev[j] = static_cast<int>(j);
  for (size_t i = 1; i <= a.size(); ++i) {
    cur[0] = static_cast<int>(i);
    for (size_t j = 1; j <= b.size(); ++j) {
      const int sub = prev[j - 1] + (a[i - 1] == b[j - 1] ? 0 : 1);
      cur[j] = std::min({prev[j] + 1, cur[j - 1] + 1, sub});
    }
    std::swap(prev, cur);
  }
  return prev[b.size()];
}

std::string closestMatch(const std::string& word, const std::vector<std::string>& candidates) {
  std::string best;
  int bestD = 1 << 30;
  for (const std::string& c : candidates) {
    const int d = editDistance(lower(word), lower(c));
    if (d < bestD) {
      bestD = d;
      best = c;
    }
  }
  const int limit = std::max<int>(2, static_cast<int>(word.size()) / 3);
  return bestD <= limit ? best : std::string();
}

std::string yamlScalar(const std::string& s) {
  bool plain = !s.empty();
  for (char c : s) {
    if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '/' || c == '.' || c == '-')) plain = false;
  }
  if (plain && (s[0] == '-' || s[0] == '.')) plain = false;
  const std::string l = lower(s);
  if (plain && (l == "true" || l == "false" || l == "yes" || l == "no" || l == "on" || l == "off" || l == "null" || l == "~")) {
    plain = false;
  }
  if (plain) {
    // Anything that would read back as a number has to be quoted.
    char* end = nullptr;
    std::strtod(s.c_str(), &end);
    if (end && *end == '\0') plain = false;
  }
  if (plain) return s;
  std::string out = "\"";
  for (char c : s) {
    if (c == '"' || c == '\\') out.push_back('\\');
    out.push_back(c);
  }
  return out + "\"";
}

std::string pyString(const std::string& s) {
  std::string out = "'";
  for (char c : s) {
    if (c == '\'' || c == '\\') out.push_back('\\');
    out.push_back(c);
  }
  return out + "'";
}

}  // namespace gzb

#include <filesystem>
#include <fstream>
#include <sstream>

namespace gzb {

bool readTextFile(const std::string& path, std::string& out, std::string& error) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    error = "cannot open '" + path + "'";
    return false;
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  out = ss.str();
  return true;
}

bool writeTextFile(const std::string& path, const std::string& text, std::string& error) {
  namespace fs = std::filesystem;
  std::error_code ec;
  const fs::path p(path);
  if (p.has_parent_path()) fs::create_directories(p.parent_path(), ec);
  if (ec) {
    error = "cannot create '" + p.parent_path().string() + "': " + ec.message();
    return false;
  }
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) {
    error = "cannot write '" + path + "'";
    return false;
  }
  out << text;
  out.close();
  if (!out) {
    error = "writing '" + path + "' failed";
    return false;
  }
  return true;
}

std::string baseName(const std::string& path) { return std::filesystem::path(path).filename().string(); }
std::string stem(const std::string& path) { return std::filesystem::path(path).stem().string(); }

}  // namespace gzb
