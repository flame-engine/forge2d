/*******************************************************************************
 * Copyright (c) 2015, Daniel Murphy, Google
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

library box2d.vec2;

import 'dart:math' as Math;
import 'dart:typed_data';

import 'math_utils.dart' as MathUtils;
import 'settings.dart' as Settings;

class Vec2 {
  final Float64List _d = new Float64List(2);

  void set x(double v) {
    _d[0] = v;
  }
  void set y(double v) {
    _d[1] = v;
  }

  double get x => _d[0];
  double get y => _d[1];

  Vec2(double x, double y) {
    _d[0] = x;
    _d[1] = y;
  }

  Vec2.zero();

  Vec2.copy(Vec2 other) {
    _d[0] = other._d[0];
    _d[1] = other._d[1];
  }

  /** Zero out this vector. */
  void setZero() {
    x = 0.0;
    y = 0.0;
  }

  /** Set the vector component-wise. */
  Vec2 setXY(double x, double y) {
    this.x = x;
    this.y = y;
    return this;
  }

  /** Set this vector to another vector. */
  Vec2 set(Vec2 v) {
    this.x = v.x;
    this.y = v.y;
    return this;
  }

  /** Return the sum of this vector and another; does not alter either one. */
  Vec2 add(Vec2 v) {
    return new Vec2(x + v.x, y + v.y);
  }

  /** Return the difference of this vector and another; does not alter either one. */
  Vec2 sub(Vec2 v) {
    return new Vec2(x - v.x, y - v.y);
  }

  /** Return this vector multiplied by a scalar; does not alter this vector. */
  Vec2 mul(double a) {
    return new Vec2(x * a, y * a);
  }

  /** Return the negation of this vector; does not alter this vector. */
  Vec2 negate() {
    return new Vec2(-x, -y);
  }

  /** Flip the vector and return it - alters this vector. */
  Vec2 negateLocal() {
    x = -x;
    y = -y;
    return this;
  }

  /** Add another vector to this one and returns result - alters this vector. */
  Vec2 addLocal(Vec2 v) {
    x += v.x;
    y += v.y;
    return this;
  }

  /** Adds values to this vector and returns result - alters this vector. */
  Vec2 addLocalXY(double x, double y) {
    this.x += x;
    this.y += y;
    return this;
  }

  /** Subtract another vector from this one and return result - alters this vector. */
  Vec2 subLocal(Vec2 v) {
    x -= v.x;
    y -= v.y;
    return this;
  }

  /** Multiply this vector by a number and return result - alters this vector. */
  Vec2 mulLocal(double a) {
    x *= a;
    y *= a;
    return this;
  }

  /** Get the skew vector such that dot(skew_vec, other) == cross(vec, other) */
  Vec2 skew() {
    return new Vec2(-y, x);
  }

  /** Get the skew vector such that dot(skew_vec, other) == cross(vec, other) */
  void skewVec2(Vec2 out) {
    out.x = -y;
    out.y = x;
  }

  /** Return the length of this vector. */
  double length() {
    return Math.sqrt(x * x + y * y);
  }

  /** Return the squared length of this vector. */
  double lengthSquared() {
    return (x * x + y * y);
  }

  /** Normalize this vector and return the length before normalization. Alters this vector. */
  double normalize() {
    double len = length();
    if (len < Settings.EPSILON) {
      return 0.0;
    }

    double invLength = 1.0 / len;
    x *= invLength;
    y *= invLength;
    return len;
  }

  /** True if the vector represents a pair of valid, non-infinite floating point numbers. */
  bool isValid() {
    return !x.isNaN && !x.isInfinite && !y.isNaN && !y.isInfinite;
  }

  /** Return a new vector that has positive components. */
  Vec2 abs() {
    return new Vec2(x.abs(), y.abs());
  }

  void absLocal() {
    x = x.abs();
    y = y.abs();
  }

  // @Override // annotation omitted for GWT-compatibility
  /** Return a copy of this vector. */
  Vec2 clone() {
    return new Vec2(x, y);
  }

  String toString() {
    return "($x,$y)";
  }

  /*
   * Static
   */

  static Vec2 abs_(Vec2 a) {
    return new Vec2(a.x.abs(), a.y.abs());
  }

  static void absToOut(Vec2 a, Vec2 out) {
    out.x = a.x.abs();
    out.y = a.y.abs();
  }

  static double dot(final Vec2 a, final Vec2 b) {
    return a.x * b.x + a.y * b.y;
  }

  static double cross(Vec2 a, Vec2 b) {
    return a.x * b.y - a.y * b.x;
  }

  static Vec2 crossVec2Dbl(Vec2 a, double s) {
    return new Vec2(s * a.y, -s * a.x);
  }

  static void crossToOutVec2Dbl(Vec2 a, double s, Vec2 out) {
    final double tempy = -s * a.x;
    out.x = s * a.y;
    out.y = tempy;
  }

  static void crossToOutUnsafeVec2Dbl(Vec2 a, double s, Vec2 out) {
    assert(out != a);
    out.x = s * a.y;
    out.y = -s * a.x;
  }

  static Vec2 crossDblVec2(double s, Vec2 a) {
    return new Vec2(-s * a.y, s * a.x);
  }

  static void crossToOutDblVec2(double s, Vec2 a, Vec2 out) {
    final double tempY = s * a.x;
    out.x = -s * a.y;
    out.y = tempY;
  }

  static void crossToOutUnsafeDblVec2(double s, Vec2 a, Vec2 out) {
    assert(out != a);
    out.x = -s * a.y;
    out.y = s * a.x;
  }

  static void negateToOut(Vec2 a, Vec2 out) {
    out.x = -a.x;
    out.y = -a.y;
  }

  static Vec2 min(Vec2 a, Vec2 b) {
    return new Vec2(a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y);
  }

  static Vec2 max(Vec2 a, Vec2 b) {
    return new Vec2(a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y);
  }

  static void minToOut(Vec2 a, Vec2 b, Vec2 out) {
    out.x = a.x < b.x ? a.x : b.x;
    out.y = a.y < b.y ? a.y : b.y;
  }

  static void maxToOut(Vec2 a, Vec2 b, Vec2 out) {
    out.x = a.x > b.x ? a.x : b.x;
    out.y = a.y > b.y ? a.y : b.y;
  }

  bool equals(Object obj) {
    if (identical(this, obj)) return true;
    if (obj == null) return false;
    if (obj is! Vec2) return false;
    Vec2 other = obj;
    return ((x == other.x) && (y == other.y));
  }

  bool approxEquals(Object obj) {
    if (identical(this, obj)) return true;
    if (obj == null) return false;
    if (obj is! Vec2) return false;
    Vec2 other = obj;
    return MathUtils.approxEquals(x, other.x) &&
        MathUtils.approxEquals(y, other.y);
  }
}
