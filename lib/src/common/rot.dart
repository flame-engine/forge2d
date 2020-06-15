part of box2d.common;

class Rot {
  double s = 0.0, c = 1.0; // sin and cos

  Rot();

  Rot.withAngle(double angle)
      : s = Math.sin(angle),
        c = Math.cos(angle);

  void setAngle(double angle) {
    s = Math.sin(angle);
    c = Math.cos(angle);
  }

  void setFrom(Rot other) {
    s = other.s;
    c = other.c;
  }

  void setIdentity() {
    s = 0.0;
    c = 1.0;
  }

  double getSin() => s;

  String toString() => "Rot(s:$s, c:$c)";

  double getCos() => c;

  double getAngle() => Math.atan2(s, c);

  Vector2 getXAxis(Vector2 xAxis) => Vector2(c, s);

  Vector2 getYAxis(Vector2 yAxis) => Vector2(-s, c);

  Rot clone() {
    return Rot()
      ..s = s
      ..c = c;
  }

  static Rot mul(Rot q, Rot r) {
    return Rot()
      ..s = q.s * r.c + q.c * r.s
      ..c = q.c * r.c - q.s * r.s;
  }

  static Rot mulTrans(Rot q, Rot r) {
    return Rot()
      ..s = q.c * r.s - q.s * r.c
      ..c = q.c * r.c + q.s * r.s;
  }

  static Vector2 mulVec2(Rot q, Vector2 v) {
    return Vector2(q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y);
  }

  static Vector2 mulTransVec2(Rot q, Vector2 v) {
    return Vector2(q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y);
  }
}
