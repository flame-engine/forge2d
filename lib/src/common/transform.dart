/// *****************************************************************************
/// Copyright (c) 2015, Daniel Murphy, Google
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without modification,
/// are permitted provided that the following conditions are met:
///  * Redistributions of source code must retain the above copyright notice,
///    this list of conditions and the following disclaimer.
///  * Redistributions in binary form must reproduce the above copyright notice,
///    this list of conditions and the following disclaimer in the documentation
///    and/or other materials provided with the distribution.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
/// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
/// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
/// IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
/// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
/// NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
/// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
/// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
/// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
/// POSSIBILITY OF SUCH DAMAGE.
/// *****************************************************************************

part of box2d.common;

/// A transform contains translation and rotation. It is used to represent the position and
/// orientation of rigid frames.
class Transform {
  /// The translation caused by the transform
  final Vector2 p;

  /// A matrix representing a rotation
  final Rot q;

  /// The default constructor.
  Transform.zero()
      : p = new Vector2.zero(),
        q = new Rot();

  /// Initialize as a copy of another transform.
  Transform.clone(final Transform xf)
      : p = xf.p.clone(),
        q = xf.q.clone();

  /// Initialize using a position vector and a rotation matrix.
  Transform.from(final Vector2 _position, final Rot _R)
      : p = _position.clone(),
        q = _R.clone();

  /// Set this to equal another transform.
  Transform set(final Transform xf) {
    p.setFrom(xf.p);
    q.set(xf.q);
    return this;
  }

  /// Set this based on the position and angle.
  void setVec2Angle(Vector2 p, double angle) {
    p.setFrom(p);
    q.setAngle(angle);
  }

  /// Set this to the identity transform.
  void setIdentity() {
    p.setZero();
    q.setIdentity();
  }

  static Vector2 mulVec2(final Transform T, final Vector2 v) {
    return new Vector2((T.q.c * v.x - T.q.s * v.y) + T.p.x,
        (T.q.s * v.x + T.q.c * v.y) + T.p.y);
  }

  static void mulToOutVec2(
      final Transform T, final Vector2 v, final Vector2 out) {
    final double tempy = (T.q.s * v.x + T.q.c * v.y) + T.p.y;
    out.x = (T.q.c * v.x - T.q.s * v.y) + T.p.x;
    out.y = tempy;
  }

  static void mulToOutUnsafeVec2(
      final Transform transform, final Vector2 v, final Vector2 out) {
    out.x = (transform.q.c * v.x - transform.q.s * v.y) + transform.p.x;
    out.y = (transform.q.s * v.x + transform.q.c * v.y) + transform.p.y;
  }

  static Vector2 mulTransVec2(final Transform T, final Vector2 v) {
    final double px = v.x - T.p.x;
    final double py = v.y - T.p.y;
    return new Vector2((T.q.c * px + T.q.s * py), (-T.q.s * px + T.q.c * py));
  }

  static void mulTransToOutVec2(
      final Transform T, final Vector2 v, final Vector2 out) {
    final double px = v.x - T.p.x;
    final double py = v.y - T.p.y;
    final double tempy = (-T.q.s * px + T.q.c * py);
    out.x = (T.q.c * px + T.q.s * py);
    out.y = tempy;
  }

  static void mulTransToOutUnsafeVec2(
      final Transform T, final Vector2 v, final Vector2 out) {
    final double px = v.x - T.p.x;
    final double py = v.y - T.p.y;
    out.x = (T.q.c * px + T.q.s * py);
    out.y = (-T.q.s * px + T.q.c * py);
  }

  static Transform mul(final Transform A, final Transform B) {
    Transform C = new Transform.zero();
    Rot.mulUnsafe(A.q, B.q, C.q);
    Rot.mulToOutUnsafe(A.q, B.p, C.p);
    C.p.add(A.p);
    return C;
  }

  static void mulToOut(
      final Transform A, final Transform B, final Transform out) {
    assert(out != A);
    Rot.mul(A.q, B.q, out.q);
    Rot.mulToOut(A.q, B.p, out.p);
    out.p.add(A.p);
  }

  static void mulToOutUnsafe(
      final Transform A, final Transform B, final Transform out) {
    assert(out != B);
    assert(out != A);
    Rot.mulUnsafe(A.q, B.q, out.q);
    Rot.mulToOutUnsafe(A.q, B.p, out.p);
    out.p.add(A.p);
  }

  static Vector2 _pool = new Vector2.zero();

  static Transform mulTrans(final Transform A, final Transform B) {
    Transform C = new Transform.zero();
    Rot.mulTransUnsafe(A.q, B.q, C.q);
    (_pool..setFrom(B.p)).sub(A.p);
    Rot.mulTransUnsafeVec2(A.q, _pool, C.p);
    return C;
  }

  void mulTransToOut(
      final Transform A, final Transform B, final Transform out) {
    assert(out != A);
    Rot.mulTrans(A.q, B.q, out.q);
    (_pool..setFrom(B.p)).sub(A.p);
    Rot.mulTransVec2(A.q, _pool, out.p);
  }

  static void mulTransToOutUnsafe(
      final Transform A, final Transform B, final Transform out) {
    assert(out != A);
    assert(out != B);
    Rot.mulTransUnsafe(A.q, B.q, out.q);
    (_pool..setFrom(B.p)).sub(A.p);
    Rot.mulTransUnsafeVec2(A.q, _pool, out.p);
  }

  String toString() {
    String s = "XForm:\n";
    s += "Position: $p\n";
    s += "R: \t$q\n";
    return s;
  }
}
