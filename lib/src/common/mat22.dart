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
 * A 2-by-2 matrix. Stored in column-major order.
 */
class Mat22 {
  final Vec2 ex, ey;

  /** Convert the matrix to printable format. */
  String toString() {
    String s = "";
    s += "[${ex.x},${ey.x}]\n";
    s += "[${ex.y},${ey.y}]";
    return s;
  }

  /**
   * Construct zero matrix. Note: this is NOT an identity matrix! djm fixed double allocation
   * problem
   */
  Mat22.zero()
      : ex = new Vec2.zero(),
        ey = new Vec2.zero();

  /**
   * Create a matrix with given vectors as columns.
   * 
   * @param c1 Column 1 of matrix
   * @param c2 Column 2 of matrix
   */
  Mat22(final Vec2 c1, final Vec2 c2)
      : ex = c1.clone(),
        ey = c2.clone();

  /**
   * Create a matrix from four doubles.
   * 
   * @param exx
   * @param col2x
   * @param exy
   * @param col2y
   */
  Mat22.withCoords(final double exx, final double col2x, final double exy,
      final double col2y)
      : ex = new Vec2(exx, exy),
        ey = new Vec2(col2x, col2y);

  /**
   * Set as a copy of another matrix.
   * 
   * @param m Matrix to copy
   */
  Mat22 setMat22(final Mat22 m) {
    ex.x = m.ex.x;
    ex.y = m.ex.y;
    ey.x = m.ey.x;
    ey.y = m.ey.y;
    return this;
  }

  Mat22 setCoords(final double exx, final double col2x, final double exy,
      final double col2y) {
    ex.x = exx;
    ex.y = exy;
    ey.x = col2x;
    ey.y = col2y;
    return this;
  }

  /**
   * Return a clone of this matrix. djm fixed double allocation
   */
  Mat22 clone() {
    return new Mat22(ex, ey);
  }

  /**
   * Set as a matrix representing a rotation.
   * 
   * @param angle Rotation (in radians) that matrix represents.
   */
  void setAngle(final double angle) {
    final double c = MathUtils.cos(angle),
        s = MathUtils.sin(angle);
    ex.x = c;
    ey.x = -s;
    ex.y = s;
    ey.y = c;
  }

  /**
   * Set as the identity matrix.
   */
  void setIdentity() {
    ex.x = 1.0;
    ey.x = 0.0;
    ex.y = 0.0;
    ey.y = 1.0;
  }

  /**
   * Set as the zero matrix.
   */
  void setZero() {
    ex.x = 0.0;
    ey.x = 0.0;
    ex.y = 0.0;
    ey.y = 0.0;
  }

  /**
   * Extract the angle from this matrix (assumed to be a rotation matrix).
   * 
   * @return
   */
  double getAngle() {
    return MathUtils.atan2(ex.y, ex.x);
  }

  /**
   * Set by column vectors.
   * 
   * @param c1 Column 1
   * @param c2 Column 2
   */
  void setVec2(final Vec2 c1, final Vec2 c2) {
    ex.x = c1.x;
    ey.x = c2.x;
    ex.y = c1.y;
    ey.y = c2.y;
  }

  /** Returns the inverted Mat22 - does NOT invert the matrix locally! */
  Mat22 invert() {
    final double a = ex.x,
        b = ey.x,
        c = ex.y,
        d = ey.y;
    final Mat22 B = new Mat22.zero();
    double det = a * d - b * c;
    if (det != 0.0) {
      det = 1.0 / det;
    }
    B.ex.x = det * d;
    B.ey.x = -det * b;
    B.ex.y = -det * c;
    B.ey.y = det * a;
    return B;
  }

  Mat22 invertLocal() {
    final double a = ex.x,
        b = ey.x,
        c = ex.y,
        d = ey.y;
    double det = a * d - b * c;
    if (det != 0.0) {
      det = 1.0 / det;
    }
    ex.x = det * d;
    ey.x = -det * b;
    ex.y = -det * c;
    ey.y = det * a;
    return this;
  }

  void invertToOut(final Mat22 out) {
    final double a = ex.x,
        b = ey.x,
        c = ex.y,
        d = ey.y;
    double det = a * d - b * c;
    // b2Assert(det != 0.0f);
    det = 1.0 / det;
    out.ex.x = det * d;
    out.ey.x = -det * b;
    out.ex.y = -det * c;
    out.ey.y = det * a;
  }

  /**
   * Return the matrix composed of the absolute values of all elements. djm: fixed double allocation
   * 
   * @return Absolute value matrix
   */
  Mat22 abs() {
    return new Mat22.withCoords(ex.x.abs(), ey.x.abs(), ex.y.abs(), ey.y.abs());
  }

  /* djm: added */
  void absLocal() {
    ex.absLocal();
    ey.absLocal();
  }

  /**
   * Return the matrix composed of the absolute values of all elements.
   * 
   * @return Absolute value matrix
   */
  static Mat22 abs_(final Mat22 R) {
    return R.abs();
  }

  /* djm created */
  void absToOut(final Mat22 R, final Mat22 out) {
    out.ex.x = R.ex.x.abs();
    out.ex.y = R.ex.y.abs();
    out.ey.x = R.ey.x.abs();
    out.ey.y = R.ey.y.abs();
  }

  /**
   * Multiply a vector by this matrix.
   * 
   * @param v Vector to multiply by matrix.
   * @return Resulting vector
   */
  Vec2 mulVec2(final Vec2 v) {
    return new Vec2(ex.x * v.x + ey.x * v.y, ex.y * v.x + ey.y * v.y);
  }

  void mulToOutVec2(final Vec2 v, final Vec2 out) {
    final double tempy = ex.y * v.x + ey.y * v.y;
    out.x = ex.x * v.x + ey.x * v.y;
    out.y = tempy;
  }

  void mulToOutUnsafeVec2(final Vec2 v, final Vec2 out) {
    assert(v != out);
    out.x = ex.x * v.x + ey.x * v.y;
    out.y = ex.y * v.x + ey.y * v.y;
  }

  /**
   * Multiply another matrix by this one (this one on left). djm optimized
   * 
   * @param R
   * @return
   */
  Mat22 mul(final Mat22 R) {
    /*
     * Mat22 C = new Mat22();C.set(this.mul(R.ex), this.mul(R.ey));return C;
     */
    final Mat22 C = new Mat22.zero();
    C.ex.x = ex.x * R.ex.x + ey.x * R.ex.y;
    C.ex.y = ex.y * R.ex.x + ey.y * R.ex.y;
    C.ey.x = ex.x * R.ey.x + ey.x * R.ey.y;
    C.ey.y = ex.y * R.ey.x + ey.y * R.ey.y;
    // C.set(ex,col2);
    return C;
  }

  Mat22 mulLocal(final Mat22 R) {
    mulToOut(R, this);
    return this;
  }

  void mulToOut(final Mat22 R, final Mat22 out) {
    final double tempy1 = this.ex.y * R.ex.x + this.ey.y * R.ex.y;
    final double tempx1 = this.ex.x * R.ex.x + this.ey.x * R.ex.y;
    out.ex.x = tempx1;
    out.ex.y = tempy1;
    final double tempy2 = this.ex.y * R.ey.x + this.ey.y * R.ey.y;
    final double tempx2 = this.ex.x * R.ey.x + this.ey.x * R.ey.y;
    out.ey.x = tempx2;
    out.ey.y = tempy2;
  }

  void mulToOutUnsafe(final Mat22 R, final Mat22 out) {
    assert(out != R);
    assert(out != this);
    out.ex.x = this.ex.x * R.ex.x + this.ey.x * R.ex.y;
    out.ex.y = this.ex.y * R.ex.x + this.ey.y * R.ex.y;
    out.ey.x = this.ex.x * R.ey.x + this.ey.x * R.ey.y;
    out.ey.y = this.ex.y * R.ey.x + this.ey.y * R.ey.y;
  }

  /**
   * Multiply another matrix by the transpose of this one (transpose of this one on left). djm:
   * optimized
   * 
   * @param B
   * @return
   */
  Mat22 mulTrans(final Mat22 B) {
    /*
     * Vec2 c1 = new Vec2(Vec2.dot(this.ex, B.ex), Vec2.dot(this.ey, B.ex)); Vec2 c2 = new
     * Vec2(Vec2.dot(this.ex, B.ey), Vec2.dot(this.ey, B.ey)); Mat22 C = new Mat22(); C.set(c1, c2);
     * return C;
     */
    final Mat22 C = new Mat22.zero();

    C.ex.x = Vec2.dot(this.ex, B.ex);
    C.ex.y = Vec2.dot(this.ey, B.ex);

    C.ey.x = Vec2.dot(this.ex, B.ey);
    C.ey.y = Vec2.dot(this.ey, B.ey);
    return C;
  }

  Mat22 mulTransLocal(final Mat22 B) {
    mulTransToOut(B, this);
    return this;
  }

  void mulTransToOut(final Mat22 B, final Mat22 out) {
    /*
     * out.ex.x = Vec2.dot(this.ex, B.ex); out.ex.y = Vec2.dot(this.ey, B.ex); out.ey.x =
     * Vec2.dot(this.ex, B.ey); out.ey.y = Vec2.dot(this.ey, B.ey);
     */
    final double x1 = this.ex.x * B.ex.x + this.ex.y * B.ex.y;
    final double y1 = this.ey.x * B.ex.x + this.ey.y * B.ex.y;
    final double x2 = this.ex.x * B.ey.x + this.ex.y * B.ey.y;
    final double y2 = this.ey.x * B.ey.x + this.ey.y * B.ey.y;
    out.ex.x = x1;
    out.ey.x = x2;
    out.ex.y = y1;
    out.ey.y = y2;
  }

  void mulTransToOutUnsafe(final Mat22 B, final Mat22 out) {
    assert(B != out);
    assert(this != out);
    out.ex.x = this.ex.x * B.ex.x + this.ex.y * B.ex.y;
    out.ey.x = this.ex.x * B.ey.x + this.ex.y * B.ey.y;
    out.ex.y = this.ey.x * B.ex.x + this.ey.y * B.ex.y;
    out.ey.y = this.ey.x * B.ey.x + this.ey.y * B.ey.y;
  }

  /**
   * Multiply a vector by the transpose of this matrix.
   * 
   * @param v
   * @return
   */
  Vec2 mulTransVec2(final Vec2 v) {
    // return new Vec2(Vec2.dot(v, ex), Vec2.dot(v, col2));
    return new Vec2((v.x * ex.x + v.y * ex.y), (v.x * ey.x + v.y * ey.y));
  }

  /* djm added */
  void mulTransToOutVec2(final Vec2 v, final Vec2 out) {
    /*
     * out.x = Vec2.dot(v, ex); out.y = Vec2.dot(v, col2);
     */
    final double tempx = v.x * ex.x + v.y * ex.y;
    out.y = v.x * ey.x + v.y * ey.y;
    out.x = tempx;
  }

  /**
   * Add this matrix to B, return the result.
   * 
   * @param B
   * @return
   */
  Mat22 add(final Mat22 B) {
    // return new Mat22(ex.add(B.ex), col2.add(B.ey));
    Mat22 m = new Mat22.zero();
    m.ex.x = ex.x + B.ex.x;
    m.ex.y = ex.y + B.ex.y;
    m.ey.x = ey.x + B.ey.x;
    m.ey.y = ey.y + B.ey.y;
    return m;
  }

  /**
   * Add B to this matrix locally.
   * 
   * @param B
   * @return
   */
  Mat22 addLocal(final Mat22 B) {
    // ex.addLocal(B.ex);
    // col2.addLocal(B.ey);
    ex.x += B.ex.x;
    ex.y += B.ex.y;
    ey.x += B.ey.x;
    ey.y += B.ey.y;
    return this;
  }

  /**
   * Solve A * x = b where A = this matrix.
   * 
   * @return The vector x that solves the above equation.
   */
  Vec2 solve(final Vec2 b) {
    final double a11 = ex.x,
        a12 = ey.x,
        a21 = ex.y,
        a22 = ey.y;
    double det = a11 * a22 - a12 * a21;
    if (det != 0.0) {
      det = 1.0 / det;
    }
    final Vec2 x =
        new Vec2(det * (a22 * b.x - a12 * b.y), det * (a11 * b.y - a21 * b.x));
    return x;
  }

  void solveToOut(final Vec2 b, final Vec2 out) {
    final double a11 = ex.x,
        a12 = ey.x,
        a21 = ex.y,
        a22 = ey.y;
    double det = a11 * a22 - a12 * a21;
    if (det != 0.0) {
      det = 1.0 / det;
    }
    final double tempy = det * (a11 * b.y - a21 * b.x);
    out.x = det * (a22 * b.x - a12 * b.y);
    out.y = tempy;
  }

  static Vec2 mulVec2_(final Mat22 R, final Vec2 v) {
    // return R.mul(v);
    return new Vec2(R.ex.x * v.x + R.ey.x * v.y, R.ex.y * v.x + R.ey.y * v.y);
  }

  static void mulToOutVec2_(final Mat22 R, final Vec2 v, final Vec2 out) {
    final double tempy = R.ex.y * v.x + R.ey.y * v.y;
    out.x = R.ex.x * v.x + R.ey.x * v.y;
    out.y = tempy;
  }

  static void mulToOutUnsafeVec2_(final Mat22 R, final Vec2 v, final Vec2 out) {
    assert(v != out);
    out.x = R.ex.x * v.x + R.ey.x * v.y;
    out.y = R.ex.y * v.x + R.ey.y * v.y;
  }

  static Mat22 mul_(final Mat22 A, final Mat22 B) {
    // return A.mul(B);
    final Mat22 C = new Mat22.zero();
    C.ex.x = A.ex.x * B.ex.x + A.ey.x * B.ex.y;
    C.ex.y = A.ex.y * B.ex.x + A.ey.y * B.ex.y;
    C.ey.x = A.ex.x * B.ey.x + A.ey.x * B.ey.y;
    C.ey.y = A.ex.y * B.ey.x + A.ey.y * B.ey.y;
    return C;
  }

  static void mulToOut_(final Mat22 A, final Mat22 B, final Mat22 out) {
    final double tempy1 = A.ex.y * B.ex.x + A.ey.y * B.ex.y;
    final double tempx1 = A.ex.x * B.ex.x + A.ey.x * B.ex.y;
    final double tempy2 = A.ex.y * B.ey.x + A.ey.y * B.ey.y;
    final double tempx2 = A.ex.x * B.ey.x + A.ey.x * B.ey.y;
    out.ex.x = tempx1;
    out.ex.y = tempy1;
    out.ey.x = tempx2;
    out.ey.y = tempy2;
  }

  static void mulToOutUnsafe_(final Mat22 A, final Mat22 B, final Mat22 out) {
    assert(out != A);
    assert(out != B);
    out.ex.x = A.ex.x * B.ex.x + A.ey.x * B.ex.y;
    out.ex.y = A.ex.y * B.ex.x + A.ey.y * B.ex.y;
    out.ey.x = A.ex.x * B.ey.x + A.ey.x * B.ey.y;
    out.ey.y = A.ex.y * B.ey.x + A.ey.y * B.ey.y;
  }

  static Vec2 mulTransVec2_(final Mat22 R, final Vec2 v) {
    return new Vec2(
        (v.x * R.ex.x + v.y * R.ex.y), (v.x * R.ey.x + v.y * R.ey.y));
  }

  static void mulTransToOutVec2_(final Mat22 R, final Vec2 v, final Vec2 out) {
    double outx = v.x * R.ex.x + v.y * R.ex.y;
    out.y = v.x * R.ey.x + v.y * R.ey.y;
    out.x = outx;
  }

  static void mulTransToOutUnsafeVec2(
      final Mat22 R, final Vec2 v, final Vec2 out) {
    assert(out != v);
    out.y = v.x * R.ey.x + v.y * R.ey.y;
    out.x = v.x * R.ex.x + v.y * R.ex.y;
  }

  static Mat22 mulTrans_(final Mat22 A, final Mat22 B) {
    final Mat22 C = new Mat22.zero();
    C.ex.x = A.ex.x * B.ex.x + A.ex.y * B.ex.y;
    C.ex.y = A.ey.x * B.ex.x + A.ey.y * B.ex.y;
    C.ey.x = A.ex.x * B.ey.x + A.ex.y * B.ey.y;
    C.ey.y = A.ey.x * B.ey.x + A.ey.y * B.ey.y;
    return C;
  }

  static void mulTransToOut_(final Mat22 A, final Mat22 B, final Mat22 out) {
    final double x1 = A.ex.x * B.ex.x + A.ex.y * B.ex.y;
    final double y1 = A.ey.x * B.ex.x + A.ey.y * B.ex.y;
    final double x2 = A.ex.x * B.ey.x + A.ex.y * B.ey.y;
    final double y2 = A.ey.x * B.ey.x + A.ey.y * B.ey.y;

    out.ex.x = x1;
    out.ex.y = y1;
    out.ey.x = x2;
    out.ey.y = y2;
  }

  static void mulTransToOutUnsafeMat22(
      final Mat22 A, final Mat22 B, final Mat22 out) {
    assert(A != out);
    assert(B != out);
    out.ex.x = A.ex.x * B.ex.x + A.ex.y * B.ex.y;
    out.ex.y = A.ey.x * B.ex.x + A.ey.y * B.ex.y;
    out.ey.x = A.ex.x * B.ey.x + A.ex.y * B.ey.y;
    out.ey.y = A.ey.x * B.ey.x + A.ey.y * B.ey.y;
  }

  static Mat22 createRotationalTransform(double angle) {
    Mat22 mat = new Mat22.zero();
    final double c = MathUtils.cos(angle);
    final double s = MathUtils.sin(angle);
    mat.ex.x = c;
    mat.ey.x = -s;
    mat.ex.y = s;
    mat.ey.y = c;
    return mat;
  }

  static void createRotationalTransformMat22(double angle, Mat22 out) {
    final double c = MathUtils.cos(angle);
    final double s = MathUtils.sin(angle);
    out.ex.x = c;
    out.ey.x = -s;
    out.ex.y = s;
    out.ey.y = c;
  }

  static Mat22 createScaleTransform(double scale) {
    Mat22 mat = new Mat22.zero();
    mat.ex.x = scale;
    mat.ey.y = scale;
    return mat;
  }

  static void createScaleTransformMat22(double scale, Mat22 out) {
    out.ex.x = scale;
    out.ey.y = scale;
  }

  bool equals(Object obj) {
    if (this == obj) return true;
    if (obj == null) return false;
    if (obj is! Mat22) return false;
    Mat22 other = obj;
    if (ex == null) {
      if (other.ex != null) return false;
    } else if (!ex.equals(other.ex)) return false;
    if (ey == null) {
      if (other.ey != null) return false;
    } else if (!ey.equals(other.ey)) return false;
    return true;
  }
}
