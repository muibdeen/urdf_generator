// Copyright 2026. Licensed under the Apache License, Version 2.0.
//
// Num: a number that remembers how it was computed.
//
// Every dimension in a generated robot is a Num. A Num always carries a
// numeric value (used when writing plain URDF) and, when it was built from a
// named xacro property or from arithmetic on one, the expression text (used
// when writing xacro, e.g. "wheel_radius*2").  Writing both outputs from the
// same Num is what keeps the .xacro and the flat .urdf consistent with each
// other.
#pragma once

#include <cmath>
#include <cstdio>
#include <string>

namespace urdfgen {

constexpr double kPi = 3.14159265358979323846;

class Num {
 public:
  // Operator precedence of the expression text (used to decide on parentheses).
  static constexpr int kAdd = 1;
  static constexpr int kMul = 2;
  static constexpr int kUnary = 3;
  static constexpr int kAtom = 4;

  Num() = default;
  Num(double v) : v_(v) {}                       // NOLINT: implicit on purpose
  Num(int v) : v_(static_cast<double>(v)) {}     // NOLINT: implicit on purpose

  /// A reference to a named xacro property (or macro parameter).
  static Num param(const std::string& name, double value) {
    return Num(value, name, kAtom);
  }
  /// A value with an explicit expression text.
  static Num expression(double value, const std::string& expr, int prec) {
    return Num(value, expr, prec);
  }
  static Num pi() { return Num(kPi, "pi", kAtom); }
  /// An angle given in degrees; the value is stored in radians.
  static Num deg(double degrees) {
    if (degrees == 0.0) return Num(0.0);
    return Num(degrees * kPi / 180.0, "radians(" + format(degrees) + ")", kAtom);
  }

  double value() const { return v_; }
  bool isLiteral() const { return expr_.empty(); }
  bool isZeroLiteral() const { return isLiteral() && v_ == 0.0; }

  /// Expression text without the ${ } wrapper.
  std::string expr() const { return isLiteral() ? format(v_) : expr_; }
  /// Text for an xacro attribute: a plain number, or ${expression}.
  std::string xacro() const { return isLiteral() ? format(v_) : "${" + expr_ + "}"; }
  /// Text for a plain URDF attribute: always the number.
  std::string urdf() const { return format(v_); }

  static std::string format(double v) {
    if (std::fabs(v) < 1e-12) return "0";
    char buf[48];
    std::snprintf(buf, sizeof buf, "%.8g", v);
    return buf;
  }

  friend Num operator+(const Num& a, const Num& b) {
    if (a.isLiteral() && b.isLiteral()) return Num(a.v_ + b.v_);
    if (a.isZeroLiteral()) return b;
    if (b.isZeroLiteral()) return a;
    if (b.isLiteral() && b.v_ < 0) return a - Num(-b.v_);
    return Num(a.v_ + b.v_, a.wrap(kAdd) + "+" + b.wrap(kAdd), kAdd);
  }
  friend Num operator-(const Num& a, const Num& b) {
    if (a.isLiteral() && b.isLiteral()) return Num(a.v_ - b.v_);
    if (b.isZeroLiteral()) return a;
    if (b.isLiteral() && b.v_ < 0) return a + Num(-b.v_);
    if (a.isZeroLiteral()) return -b;
    return Num(a.v_ - b.v_, a.wrap(kAdd) + "-" + b.wrap(kAdd + 1), kAdd);
  }
  friend Num operator-(const Num& a) {
    if (a.isLiteral()) return Num(-a.v_);
    // -(-x) is x. A unary-precedence expression is "-" followed by an atom or a bracket.
    if (a.prec_ == kUnary && a.expr_.size() > 1 && a.expr_[0] == '-') return Num(-a.v_, a.expr_.substr(1), kAtom);
    return Num(-a.v_, "-" + a.wrap(kUnary), kUnary);
  }
  friend Num operator*(const Num& a, const Num& b) {
    if (a.isLiteral() && b.isLiteral()) return Num(a.v_ * b.v_);
    if (a.isZeroLiteral() || b.isZeroLiteral()) return Num(0.0);
    if (a.isLiteral() && a.v_ == 1.0) return b;
    if (b.isLiteral() && b.v_ == 1.0) return a;
    if (a.isLiteral() && a.v_ == -1.0) return -b;
    if (b.isLiteral() && b.v_ == -1.0) return -a;
    return Num(a.v_ * b.v_, a.wrap(kMul) + "*" + b.wrap(kMul), kMul);
  }
  friend Num operator/(const Num& a, const Num& b) {
    if (a.isLiteral() && b.isLiteral()) return Num(a.v_ / b.v_);
    if (b.isLiteral() && b.v_ == 1.0) return a;
    if (a.isZeroLiteral()) return Num(0.0);
    return Num(a.v_ / b.v_, a.wrap(kMul) + "/" + b.wrap(kMul + 1), kMul);
  }

 private:
  Num(double v, const std::string& e, int p) : v_(v), expr_(e), prec_(p) {}

  int prec() const { return isLiteral() ? (v_ < 0 ? kUnary : kAtom) : prec_; }
  std::string wrap(int minPrec) const {
    return prec() < minPrec ? "(" + expr() + ")" : expr();
  }

  double v_ = 0.0;
  std::string expr_;  // empty for a plain literal
  int prec_ = kAtom;
};

namespace detail {
inline Num fn1(const char* name, double (*f)(double), const Num& a) {
  double r = f(a.value());
  if (std::fabs(r) < 1e-12) r = 0.0;
  if (a.isLiteral()) return Num(r);
  return Num::expression(r, std::string(name) + "(" + a.expr() + ")", Num::kAtom);
}
}  // namespace detail

inline Num nsin(const Num& a) {
  return detail::fn1("sin", [](double v) { return std::sin(v); }, a);
}
inline Num ncos(const Num& a) {
  return detail::fn1("cos", [](double v) { return std::cos(v); }, a);
}
inline Num nsqrt(const Num& a) {
  return detail::fn1("sqrt", [](double v) { return std::sqrt(v < 0 ? 0.0 : v); }, a);
}
inline Num nacos(const Num& a) {
  return detail::fn1("acos", [](double v) { return std::acos(v < -1 ? -1.0 : (v > 1 ? 1.0 : v)); }, a);
}
inline Num natan2(const Num& y, const Num& x) {
  double r = std::atan2(y.value(), x.value());
  if (std::fabs(r) < 1e-12) r = 0.0;
  if (y.isLiteral() && x.isLiteral()) return Num(r);
  return Num::expression(r, "atan2(" + y.expr() + "," + x.expr() + ")", Num::kAtom);
}

}  // namespace urdfgen
