library forge2d.math_utils;

import 'dart:math' as Math;

import 'vector_math.dart';

Vector2 crossDblVec2(double s, Vector2 a) {
  return Vector2(-s * a.y, s * a.x);
}

Vector2 matrix3Mul22(Matrix3 A, Vector2 v) {
  final y = A.entry(1, 0) * v.x + A.entry(1, 1) * v.y;
  final x = A.entry(0, 0) * v.x + A.entry(0, 1) * v.y;
  return Vector2(x, y);
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

/// Returns the zero matrix if singular.
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
