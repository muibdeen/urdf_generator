// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// Small shared pieces: issues (what the helper tells the user), 3D vectors and transforms,
// and a few string helpers. Nothing in this library depends on Qt or on ROS.
#pragma once

#include <string>
#include <vector>

namespace gzb {

constexpr double kPi = 3.14159265358979323846;

// ------------------------------------------------------------------------------ issues

enum class Severity { Info, Warning, Error };

/// One thing the user should know about: where it was found and what is wrong.
struct Issue {
  Severity severity = Severity::Info;
  std::string where;    // "URDF", "YAML <file name>", "Plan", "Package"
  std::string message;
};
using Issues = std::vector<Issue>;

const char* severityName(Severity s);
void addIssue(Issues& issues, Severity s, const std::string& where, const std::string& message);
bool hasErrors(const Issues& issues);
int countIssues(const Issues& issues, Severity s);

// ------------------------------------------------------------------------------ geometry

struct Vec3 {
  double x = 0, y = 0, z = 0;
};
Vec3 operator+(const Vec3& a, const Vec3& b);
Vec3 operator-(const Vec3& a, const Vec3& b);
Vec3 operator*(const Vec3& a, double s);
double dot(const Vec3& a, const Vec3& b);
double norm(const Vec3& a);
/// The unit vector of `a`; (0, 0, 0) stays (0, 0, 0).
Vec3 normalized(const Vec3& a);

/// A rigid transform: p' = R p + t.
struct Transform {
  double r[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  Vec3 t;

  /// URDF convention: R = Rz(yaw) Ry(pitch) Rx(roll), then the translation xyz.
  static Transform fromXyzRpy(const Vec3& xyz, const Vec3& rpy);
  Vec3 rotate(const Vec3& v) const;
  Vec3 apply(const Vec3& p) const;
  Transform operator*(const Transform& o) const;  // (this * o)(p) = this(o(p))
};

// ------------------------------------------------------------------------------ strings

std::string trim(const std::string& s);
std::string lower(const std::string& s);
std::vector<std::string> split(const std::string& s, char sep);
bool startsWith(const std::string& s, const std::string& prefix);
bool endsWith(const std::string& s, const std::string& suffix);
/// Shortest text for a number that still reads back to the same value (9 significant digits).
std::string num(double v);
std::string xmlEscape(const std::string& s);
/// Letters, digits and '_', not starting with a digit.
bool validIdentifier(const std::string& s);
int editDistance(const std::string& a, const std::string& b);
/// The entry of `candidates` closest to `word` (within a small distance), or "".
std::string closestMatch(const std::string& word, const std::vector<std::string>& candidates);
/// Quotes a string for a YAML scalar only when needed.
std::string yamlScalar(const std::string& s);
/// Python string literal in single quotes.
std::string pyString(const std::string& s);

}  // namespace gzb

namespace gzb {
/// Reads a whole file; false (and `error` set) when it cannot be read.
bool readTextFile(const std::string& path, std::string& out, std::string& error);
/// Writes a file, creating the folders above it; false (and `error` set) on failure.
bool writeTextFile(const std::string& path, const std::string& text, std::string& error);
/// "dir/name.ext" -> "name.ext".
std::string baseName(const std::string& path);
/// "dir/name.ext" -> "name".
std::string stem(const std::string& path);
}  // namespace gzb
