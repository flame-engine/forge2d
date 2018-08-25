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

library box2d.math_utils;

import 'dart:math' as Math;

import 'vector_math.dart';

const double TWOPI = Math.pi * 2.0;

double distanceSquared(Vector2 v1, Vector2 v2) => v1.distanceToSquared(v2);

double distance(Vector2 v1, Vector2 v2) => v1.distanceTo(v2);

/** Returns the closest value to 'a' that is in between 'low' and 'high' */
double clampDouble(final double a, final double low, final double high) =>
    Math.max(low, Math.min(a, high));

Vector2 clampVec2(final Vector2 a, final Vector2 low, final Vector2 high) {
  final Vector2 min = new Vector2.zero();
  min.x = a.x < high.x ? a.x : high.x;
  min.y = a.y < high.y ? a.y : high.y;
  min.x = low.x > min.x ? low.x : min.x;
  min.y = low.y > min.y ? low.y : min.y;
  return min;
}

/**
 * Given a value within the range specified by [fromMin] and [fromMax],
 * returns a value with the same relative position in the range specified
 * from [toMin] and [toMax]. For example, given a [val] of 2 in the
 * "from range" of 0-4, and a "to range" of 10-20, would return 15.
 */
double translateAndScale(
    double val, double fromMin, double fromMax, double toMin, double toMax) {
  final double mult = (val - fromMin) / (fromMax - fromMin);
  final double res = toMin + mult * (toMax - toMin);
  return res;
}

bool approxEquals(num expected, num actual, [num tolerance = null]) {
  if (tolerance == null) {
    tolerance = (expected / 1e4).abs();
  }
  return ((expected - actual).abs() <= tolerance);
}

Vector2 crossDblVec2(double s, Vector2 a) {
  return new Vector2(-s * a.y, s * a.x);
}

bool vector2Equals(Vector2 a, Vector2 b) {
  if ((a == null) || (b == null)) return false;
  if (identical(a, b)) return true;
  if (a is! Vector2 || b is! Vector2) return false;
  return ((a.x == b.x) && (a.y == b.y));
}

bool vector2IsValid(Vector2 v) {
  return !v.x.isNaN && !v.x.isInfinite && !v.y.isNaN && !v.y.isInfinite;
}

void matrix3MulToOutUnsafe(Matrix3 A, Vector3 v, Vector3 out) {
  assert(out != v);
  out.x = v.x * A.entry(0, 0) + v.y * A.entry(0, 1) + v.z * A.entry(0, 2);
  out.y = v.x * A.entry(1, 0) + v.y * A.entry(1, 1) + v.z * A.entry(1, 2);
  out.z = v.x * A.entry(2, 0) + v.y * A.entry(2, 1) + v.z * A.entry(2, 2);
}

void matrix3Mul22ToOutUnsafe(Matrix3 A, Vector2 v, Vector2 out) {
  assert(v != out);
  out.y = A.entry(1, 0) * v.x + A.entry(1, 1) * v.y;
  out.x = A.entry(0, 0) * v.x + A.entry(0, 1) * v.y;
}

void matrix3GetInverse22(Matrix3 m, Matrix3 M) {
  double a = m.entry(0, 0),
      b = m.entry(0, 1),
      c = m.entry(1, 0),
      d = m.entry(1, 1);
  double det = a * d - b * c;
  if (det != 0.0) {
    det = 1.0 / det;
  }

  double ex_x = det * d;
  double ey_x = -det * b;
  double ex_z = 0.0;
  double ex_y = -det * c;
  double ey_y = det * a;
  double ey_z = 0.0;
  double ez_x = 0.0;
  double ez_y = 0.0;
  double ez_z = 0.0;
  M.setValues(ex_x, ex_y, ex_z, ey_x, ey_y, ey_z, ez_x, ez_y, ez_z);
}

// / Returns the zero matrix if singular.
void matrix3GetSymInverse33(Matrix3 m, Matrix3 M) {
  double bx = m.entry(1, 1) * m.entry(2, 2) - m.entry(2, 1) * m.entry(1, 2);
  double by = m.entry(2, 1) * m.entry(0, 2) - m.entry(0, 1) * m.entry(2, 2);
  double bz = m.entry(0, 1) * m.entry(1, 2) - m.entry(1, 1) * m.entry(0, 2);
  double det = m.entry(0, 0) * bx + m.entry(1, 0) * by + m.entry(2, 0) * bz;
  if (det != 0.0) {
    det = 1.0 / det;
  }

  double a11 = m.entry(0, 0), a12 = m.entry(0, 1), a13 = m.entry(0, 2);
  double a22 = m.entry(1, 1), a23 = m.entry(1, 2);
  double a33 = m.entry(2, 2);

  double ex_x = det * (a22 * a33 - a23 * a23);
  double ex_y = det * (a13 * a23 - a12 * a33);
  double ex_z = det * (a12 * a23 - a13 * a22);

  double ey_x = M.entry(1, 0);
  double ey_y = det * (a11 * a33 - a13 * a13);
  double ey_z = det * (a13 * a12 - a11 * a23);

  double ez_x = M.entry(2, 0);
  double ez_y = M.entry(2, 1);
  double ez_z = det * (a11 * a22 - a12 * a12);
  M.setValues(ex_x, ex_y, ex_z, ey_x, ey_y, ey_z, ez_x, ez_y, ez_z);
}
