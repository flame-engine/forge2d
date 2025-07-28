import 'dart:math' as math;

import 'package:forge2d/forge2d.dart';

class Rot {
  double sin;
  double cos;

  Rot({this.sin = 0.0, this.cos = 1.0});

  Rot.withAngle(double angle) : sin = math.sin(angle), cos = math.cos(angle);

  void setAngle(double angle) {
    sin = math.sin(angle);
    cos = math.cos(angle);
  }

  void setFrom(Rot other) {
    sin = other.sin;
    cos = other.cos;
  }

  void setIdentity() {
    sin = 0.0;
    cos = 1.0;
  }

  double getSin() => sin;

  @override
  String toString() => 'Rot(s:$sin, c:$cos)';

  double getCos() => cos;

  double getAngle() => math.atan2(sin, cos);

  Vector2 getXAxis({Vector2? out}) {
    final result = out ?? Vector2.zero();
    return result..setValues(cos, sin);
  }

  Vector2 getYAxis({Vector2? out}) {
    final result = out ?? Vector2.zero();
    return result..setValues(-sin, cos);
  }

  Rot clone() => Rot(sin: sin, cos: cos);

  Rot.mul(Rot q, Rot r)
    : this(
        sin: q.sin * r.cos + q.cos * r.sin,
        cos: q.cos * r.cos - q.sin * r.sin,
      );

  Rot.mulTrans(Rot q, Rot r)
    : this(
        sin: q.cos * r.sin - q.sin * r.cos,
        cos: q.cos * r.cos + q.sin * r.sin,
      );

  static Vector2 mulVec2(Rot q, Vector2 v) {
    return Vector2(q.cos * v.x - q.sin * v.y, q.sin * v.x + q.cos * v.y);
  }

  static Vector2 mulTransVec2(Rot q, Vector2 v) {
    return Vector2(q.cos * v.x + q.sin * v.y, -q.sin * v.x + q.cos * v.y);
  }
}
