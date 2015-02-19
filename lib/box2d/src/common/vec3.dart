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

class Vec3 {
  // TODO(srdjan): use Float64List.
  double x, y, z;

  Vec3(this.x, this.y, this.z);

  Vec3.zero()
      : x = 0.0,
        y = 0.0,
        z = 0.0;

  Vec3.copy(Vec3 copy)
      : x = copy.x,
        y = copy.y,
        z = copy.z;

  Vec3 set(Vec3 vec) {
    x = vec.x;
    y = vec.y;
    z = vec.z;
    return this;
  }

  Vec3 setXYZ(double argX, double argY, double argZ) {
    x = argX;
    y = argY;
    z = argZ;
    return this;
  }

  Vec3 addLocal(Vec3 argVec) {
    x += argVec.x;
    y += argVec.y;
    z += argVec.z;
    return this;
  }

  Vec3 add(Vec3 argVec) {
    return new Vec3(x + argVec.x, y + argVec.y, z + argVec.z);
  }

  Vec3 subLocal(Vec3 argVec) {
    x -= argVec.x;
    y -= argVec.y;
    z -= argVec.z;
    return this;
  }

  Vec3 sub(Vec3 argVec) {
    return new Vec3(x - argVec.x, y - argVec.y, z - argVec.z);
  }

  Vec3 mulLocal(double argScalar) {
    x *= argScalar;
    y *= argScalar;
    z *= argScalar;
    return this;
  }

  Vec3 mul(double argScalar) {
    return new Vec3(x * argScalar, y * argScalar, z * argScalar);
  }

  Vec3 negate() {
    return new Vec3(-x, -y, -z);
  }

  Vec3 negateLocal() {
    x = -x;
    y = -y;
    z = -z;
    return this;
  }

  void setZero() {
    x = 0.0;
    y = 0.0;
    z = 0.0;
  }

  Vec3 clone() {
    return new Vec3.copy(this);
  }

  String toString() {
    return "($x,$y,$z)";
  }

  bool equals(Object obj) {
    if (identical(this, obj)) return true;
    if (obj == null) return false;
    if (obj is! Vec3) return false;
    Vec3 other = obj;
    return ((x == other.x) && (y == other.y) && (z == other.z));
  }

  bool approxEquals(Object obj) {
    if (identical(this, obj)) return true;
    if (obj == null) return false;
    if (obj is! Vec3) return false;
    Vec3 other = obj;
    return MathUtils.approxEquals(x, other.x) &&
           MathUtils.approxEquals(y, other.y) &&
           MathUtils.approxEquals(z, other.z);
  }

  static double dot(Vec3 a, Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
  }

  static Vec3 cross(Vec3 a, Vec3 b) {
    return new Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
  }

  static void crossToOut(Vec3 a, Vec3 b, Vec3 out) {
    final double tempy = a.z * b.x - a.x * b.z;
    final double tempz = a.x * b.y - a.y * b.x;
    out.x = a.y * b.z - a.z * b.y;
    out.y = tempy;
    out.z = tempz;
  }

  static void crossToOutUnsafe(Vec3 a, Vec3 b, Vec3 out) {
    assert(out != b);
    assert(out != a);
    out.x = a.y * b.z - a.z * b.y;
    out.y = a.z * b.x - a.x * b.z;
    out.z = a.x * b.y - a.y * b.x;
  }
}
