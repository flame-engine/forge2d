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

part of box2d;

/**
 * A 3-by-3 matrix. Stored in column-major order.
 * 
 * @author Daniel Murphy
 */
class Mat33 {

  static final Mat33 IDENTITY = new Mat33(new Vec3(1.0, 0.0, 0.0), new Vec3(0.0, 1.0, 0.0), new Vec3(0.0, 0.0, 1.0));

  final Vec3 ex, ey, ez;

  Mat33(Vec3 argCol1, Vec3 argCol2, Vec3 argCol3)
      : ex = argCol1.clone(),
        ey = argCol2.clone(),
        ez = argCol3.clone();

  Mat33.zero()
      : ex = new Vec3.zero(),
        ey = new Vec3.zero(),
        ez = new Vec3.zero();

  Mat33.withCoords(double exx, double exy, double exz, double eyx, double eyy, double eyz, double ezx, double ezy, double ezz)
      : ex = new Vec3(exx, exy, exz),
        ey = new Vec3(eyx, eyy, eyz),
        ez = new Vec3(ezx, ezy, ezz);

  void setZero() {
    ex.setZero();
    ey.setZero();
    ez.setZero();
  }

  void setCoords(double exx, double exy, double exz, double eyx, double eyy, double eyz, double ezx, double ezy, double ezz) {
    ex.x = exx;
    ex.y = exy;
    ex.z = exz;
    ey.x = eyx;
    ey.y = eyy;
    ey.z = eyz;
    ez.x = eyx;
    ez.y = eyy;
    ez.z = eyz;
  }

  void setMat33(Mat33 mat) {
    Vec3 vec = mat.ex;
    ex.x = vec.x;
    ex.y = vec.y;
    ex.z = vec.z;
    Vec3 vec1 = mat.ey;
    ey.x = vec1.x;
    ey.y = vec1.y;
    ey.z = vec1.z;
    Vec3 vec2 = mat.ez;
    ez.x = vec2.x;
    ez.y = vec2.y;
    ez.z = vec2.z;
  }

  void setIdentity() {
    ex.x = 1.0;
    ex.y = 0.0;
    ex.z = 0.0;
    ey.x = 0.0;
    ey.y = 1.0;
    ey.z = 0.0;
    ez.x = 0.0;
    ez.y = 0.0;
    ez.z = 1.0;
  }

  // / Multiply a matrix times a vector.
  static Vec3 mul(Mat33 A, Vec3 v) {
    return new Vec3(v.x * A.ex.x + v.y * A.ey.x + v.z + A.ez.x, v.x * A.ex.y + v.y * A.ey.y + v.z * A.ez.y, v.x * A.ex.z + v.y * A.ey.z + v.z * A.ez.z);
  }

  static Vec2 mul22(Mat33 A, Vec2 v) {
    return new Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
  }

  static void mul22ToOut(Mat33 A, Vec2 v, Vec2 out) {
    final double tempx = A.ex.x * v.x + A.ey.x * v.y;
    out.y = A.ex.y * v.x + A.ey.y * v.y;
    out.x = tempx;
  }

  static void mul22ToOutUnsafe(Mat33 A, Vec2 v, Vec2 out) {
    assert(v != out);
    out.y = A.ex.y * v.x + A.ey.y * v.y;
    out.x = A.ex.x * v.x + A.ey.x * v.y;
  }

  static void mulToOut(Mat33 A, Vec3 v, Vec3 out) {
    final double tempy = v.x * A.ex.y + v.y * A.ey.y + v.z * A.ez.y;
    final double tempz = v.x * A.ex.z + v.y * A.ey.z + v.z * A.ez.z;
    out.x = v.x * A.ex.x + v.y * A.ey.x + v.z * A.ez.x;
    out.y = tempy;
    out.z = tempz;
  }

  static void mulToOutUnsafe(Mat33 A, Vec3 v, Vec3 out) {
    assert(out != v);
    out.x = v.x * A.ex.x + v.y * A.ey.x + v.z * A.ez.x;
    out.y = v.x * A.ex.y + v.y * A.ey.y + v.z * A.ez.y;
    out.z = v.x * A.ex.z + v.y * A.ey.z + v.z * A.ez.z;
  }

  /**
   * Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse
   * in one-shot cases.
   * 
   * @param b
   * @return
   */
  Vec2 solve22(Vec2 b) {
    Vec2 x = new Vec2.zero();
    solve22ToOut(b, x);
    return x;
  }

  /**
   * Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse
   * in one-shot cases.
   * 
   * @param b
   * @return
   */
  void solve22ToOut(Vec2 b, Vec2 out) {
    final double a11 = ex.x,
        a12 = ey.x,
        a21 = ex.y,
        a22 = ey.y;
    double det = a11 * a22 - a12 * a21;
    if (det != 0.0) {
      det = 1.0 / det;
    }
    out.x = det * (a22 * b.x - a12 * b.y);
    out.y = det * (a11 * b.y - a21 * b.x);
  }


  // djm pooling from below
  /**
   * Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse
   * in one-shot cases.
   * 
   * @param b
   * @return
   */
  Vec3 solve33(Vec3 b) {
    Vec3 x = new Vec3.zero();
    solve33ToOut(b, x);
    return x;
  }

  /**
   * Solve A * x = b, where b is a column vector. This is more efficient than computing the inverse
   * in one-shot cases.
   * 
   * @param b
   * @param out the result
   */
  void solve33ToOut(Vec3 b, Vec3 out) {
    assert(b != out);
    Vec3.crossToOutUnsafe(ey, ez, out);
    double det = Vec3.dot(ex, out);
    if (det != 0.0) {
      det = 1.0 / det;
    }
    Vec3.crossToOutUnsafe(ey, ez, out);
    final double x = det * Vec3.dot(b, out);
    Vec3.crossToOutUnsafe(b, ez, out);
    final double y = det * Vec3.dot(ex, out);
    Vec3.crossToOutUnsafe(ey, b, out);
    double z = det * Vec3.dot(ex, out);
    out.x = x;
    out.y = y;
    out.z = z;
  }

  void getInverse22(Mat33 M) {
    double a = ex.x,
        b = ey.x,
        c = ex.y,
        d = ey.y;
    double det = a * d - b * c;
    if (det != 0.0) {
      det = 1.0 / det;
    }

    M.ex.x = det * d;
    M.ey.x = -det * b;
    M.ex.z = 0.0;
    M.ex.y = -det * c;
    M.ey.y = det * a;
    M.ey.z = 0.0;
    M.ez.x = 0.0;
    M.ez.y = 0.0;
    M.ez.z = 0.0;
  }

  // / Returns the zero matrix if singular.
  void getSymInverse33(Mat33 M) {
    double bx = ey.y * ez.z - ey.z * ez.y;
    double by = ey.z * ez.x - ey.x * ez.z;
    double bz = ey.x * ez.y - ey.y * ez.x;
    double det = ex.x * bx + ex.y * by + ex.z * bz;
    if (det != 0.0) {
      det = 1.0 / det;
    }

    double a11 = ex.x,
        a12 = ey.x,
        a13 = ez.x;
    double a22 = ey.y,
        a23 = ez.y;
    double a33 = ez.z;

    M.ex.x = det * (a22 * a33 - a23 * a23);
    M.ex.y = det * (a13 * a23 - a12 * a33);
    M.ex.z = det * (a12 * a23 - a13 * a22);

    M.ey.x = M.ex.y;
    M.ey.y = det * (a11 * a33 - a13 * a13);
    M.ey.z = det * (a13 * a12 - a11 * a23);

    M.ez.x = M.ex.z;
    M.ez.y = M.ey.z;
    M.ez.z = det * (a11 * a22 - a12 * a12);
  }


  static void setScaleTransform(double scale, Mat33 out) {
    out.ex.x = scale;
    out.ey.y = scale;
  }

  bool equals(Object obj) {
    if (identical(this, obj)) return true;
    if (obj == null) return false;
    if (obj is! Mat33) return false;
    Mat33 other = obj;
    if (ex == null) {
      if (other.ex != null) return false;
    } else if (!ex.equals(other.ex)) {
      return false;
    }
    if (ey == null) {
      if (other.ey != null) return false;
    } else if (!ey.equals(other.ey)) return false;
    if (ez == null) {
      if (other.ez != null) return false;
    } else if (!ez.equals(other.ez)) return false;
    return true;
  }
}
