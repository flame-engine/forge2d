import '../../forge2d.dart';

/// A transform contains translation and rotation. It is used to represent the
/// position and orientation of rigid frames.
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
  Transform.from(final Vector2 position, final Rot r)
      : p = position.clone(),
        q = r.clone();

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

  static Vector2 mulVec2(final Transform t, final Vector2 v) {
    return Vector2(
      (t.q.cos * v.x - t.q.sin * v.y) + t.p.x,
      (t.q.sin * v.x + t.q.cos * v.y) + t.p.y,
    );
  }

  static Vector2 mulTransVec2(final Transform t, final Vector2 v) {
    final pX = v.x - t.p.x;
    final pY = v.y - t.p.y;
    return Vector2(t.q.cos * pX + t.q.sin * pY, -t.q.sin * pX + t.q.cos * pY);
  }

  factory Transform.mul(final Transform a, final Transform b) {
    final c = Transform.zero();
    c.q.setFrom(Rot.mul(a.q, b.q));
    c.p.setFrom(Rot.mulVec2(a.q, b.p));
    c.p.add(a.p);
    return c;
  }

  factory Transform.mulTrans(final Transform a, final Transform b) {
    final v = b.p - a.p;
    return Transform.from(Rot.mulTransVec2(a.q, v), Rot.mulTrans(a.q, b.q));
  }

  @override
  String toString() {
    var s = 'XForm:\n';
    s += 'Position: $p\n';
    s += 'R: \t$q\n';
    return s;
  }
}
