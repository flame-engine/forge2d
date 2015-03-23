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

class Vector2 {
  final Float64List _d = new Float64List(2);

  void set x(double v) {
    _d[0] = v;
  }
  void set y(double v) {
    _d[1] = v;
  }

  double get x => _d[0];
  double get y => _d[1];

  Vector2(double x, double y) {
    _d[0] = x;
    _d[1] = y;
  }

  Vector2.zero();

  Vector2.copy(Vector2 other) {
    _d[0] = other._d[0];
    _d[1] = other._d[1];
  }

  /** Zero out this vector. */
  void setZero() {
    x = 0.0;
    y = 0.0;
  }

  /** Set the vector component-wise. */
  Vector2 setValues(double x, double y) {
    this.x = x;
    this.y = y;
    return this;
  }

  /** Set this vector to another vector. */
  Vector2 setFrom(Vector2 v) {
    this.x = v.x;
    this.y = v.y;
    return this;
  }

  /// Negate.
  Vector2 operator -() => new Vector2(-x, -y);

  /// Add two vectors.
  Vector2 operator +(Vector2 other) => new Vector2(x + other.x, y + other.y);

  /// Subtract two vectors.
  Vector2 operator -(Vector2 other) => new Vector2(x - other.x, y - other.y);

  /// Scale.
  Vector2 operator /(double scale) {
    var o = 1.0 / scale;
    return new Vector2(x * o, y * o);
  }

  /// Scale.
  Vector2 operator *(double scale) {
    var o = scale;
    return new Vector2(x * o, y * o);
  }

  /** Flip the vector and return it - alters this vector. */
  Vector2 negate() {
    x = -x;
    y = -y;
    return this;
  }

  /** Add another vector to this one and returns result - alters this vector. */
  Vector2 add(Vector2 v) {
    x += v.x;
    y += v.y;
    return this;
  }

  /** Adds values to this vector and returns result - alters this vector. */
  Vector2 addValues(double x, double y) {
    this.x += x;
    this.y += y;
    return this;
  }

  /** Subtract another vector from this one and return result - alters this vector. */
  Vector2 sub(Vector2 v) {
    x -= v.x;
    y -= v.y;
    return this;
  }

  /** Multiply this vector by a number and return result - alters this vector. */
  Vector2 scale(double a) {
    x *= a;
    y *= a;
    return this;
  }

  /** Get the skew vector such that dot(skew_vec, other) == cross(vec, other) */
  Vector2 skew() {
    return new Vector2(-y, x);
  }

  /** Get the skew vector such that dot(skew_vec, other) == cross(vec, other) */
  void skewVec2(Vector2 out) {
    out.x = -y;
    out.y = x;
  }

  /** Return the length of this vector. */
  double get length {
    return Math.sqrt(x * x + y * y);
  }

  /** Return the squared length of this vector. */
  double get length2 {
    return (x * x + y * y);
  }

  /// Normalize [this].
  Vector2 normalize() {
    double l = length;
    // TODO(johnmccutchan): Use an epsilon.
    if (l == 0.0) {
      return this;
    }
    l = 1.0 / l;
    x *= l;
    y *= l;
    return this;
  }

  /** Normalize this vector and return the length before normalization. Alters this vector. */
  double normalizeLength() {
    double len = length;
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
  Vector2 abs() {
    return new Vector2(x.abs(), y.abs());
  }

  void absolute() {
    x = x.abs();
    y = y.abs();
  }

  // @Override // annotation omitted for GWT-compatibility
  /** Return a copy of this vector. */
  Vector2 clone() {
    return new Vector2(x, y);
  }

  String toString() {
    return "($x,$y)";
  }

  /*
   * Static
   */

  static Vector2 abs_(Vector2 a) {
    return new Vector2(a.x.abs(), a.y.abs());
  }

  static void absToOut(Vector2 a, Vector2 out) {
    out.x = a.x.abs();
    out.y = a.y.abs();
  }

  double dot(final Vector2 b) {
    return x * b.x + y * b.y;
  }

  double cross(Vector2 b) {
    return x * b.y - y * b.x;
  }

  static Vector2 crossVec2Dbl(Vector2 a, double s) {
    return new Vector2(s * a.y, -s * a.x);
  }

  static void crossToOutVec2Dbl(Vector2 a, double s, Vector2 out) {
    final double tempy = -s * a.x;
    out.x = s * a.y;
    out.y = tempy;
  }

  static void crossToOutUnsafeVec2Dbl(Vector2 a, double s, Vector2 out) {
    assert(out != a);
    out.x = s * a.y;
    out.y = -s * a.x;
  }

  static Vector2 crossDblVec2(double s, Vector2 a) {
    return new Vector2(-s * a.y, s * a.x);
  }

  static void crossToOutDblVec2(double s, Vector2 a, Vector2 out) {
    final double tempY = s * a.x;
    out.x = -s * a.y;
    out.y = tempY;
  }

  static void crossToOutUnsafeDblVec2(double s, Vector2 a, Vector2 out) {
    assert(out != a);
    out.x = -s * a.y;
    out.y = s * a.x;
  }

  static void negateToOut(Vector2 a, Vector2 out) {
    out.x = -a.x;
    out.y = -a.y;
  }

  static Vector2 minV2(Vector2 a, Vector2 b) {
    return new Vector2(a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y);
  }

  static Vector2 maxV2(Vector2 a, Vector2 b) {
    return new Vector2(a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y);
  }

  static void min(Vector2 a, Vector2 b, Vector2 out) {
    out.x = a.x < b.x ? a.x : b.x;
    out.y = a.y < b.y ? a.y : b.y;
  }

  static void max(Vector2 a, Vector2 b, Vector2 out) {
    out.x = a.x > b.x ? a.x : b.x;
    out.y = a.y > b.y ? a.y : b.y;
  }
  
  /// Rotate [this] by 90 degrees then scale it. Store result in [out]. Return [out].
  Vector2 scaleOrthogonalInto(double scale, Vector2 out) {
    out.setValues(-scale * y, scale * x);
    return out;
  }

  bool equals(Object obj) {
    if (identical(this, obj)) return true;
    if (obj == null) return false;
    if (obj is! Vector2) return false;
    Vector2 other = obj;
    return ((x == other.x) && (y == other.y));
  }

  bool approxEquals(Object obj) {
    if (identical(this, obj)) return true;
    if (obj == null) return false;
    if (obj is! Vector2) return false;
    Vector2 other = obj;
    return MathUtils.approxEquals(x, other.x) &&
        MathUtils.approxEquals(y, other.y);
  }
}
