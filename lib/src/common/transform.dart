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
      : p = Vector2.zero(),
        q = Rot();

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
    q.setFrom(xf.q);
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
    return Vector2((T.q.c * v.x - T.q.s * v.y) + T.p.x,
        (T.q.s * v.x + T.q.c * v.y) + T.p.y);
  }

  static Vector2 mulTransVec2(final Transform T, final Vector2 v) {
    final double px = v.x - T.p.x;
    final double py = v.y - T.p.y;
    return Vector2((T.q.c * px + T.q.s * py), (-T.q.s * px + T.q.c * py));
  }

  static Transform mul(final Transform A, final Transform B) {
    Transform c = Transform.zero();
    c.q.setFrom(Rot.mul(A.q, B.q));
    c.p.setFrom(Rot.mulVec2(A.q, B.p));
    c.p.add(A.p);
    return c;
  }

  static Vector2 _pool = Vector2.zero();

  static Transform mulTrans(final Transform A, final Transform B) {
    Transform c = Transform.zero();
    c.q.setFrom(Rot.mulTrans(A.q, B.q));
    (_pool..setFrom(B.p)).sub(A.p);
    c.p.setFrom(Rot.mulTransVec2(A.q, _pool));
    return c;
  }

  String toString() {
    String s = "XForm:\n";
    s += "Position: $p\n";
    s += "R: \t$q\n";
    return s;
  }
}
