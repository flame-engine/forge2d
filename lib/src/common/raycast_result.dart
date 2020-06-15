part of box2d.common;

class RaycastResult {
  double lambda = 0.0;
  final Vector2 normal = Vector2.zero();

  RaycastResult set(RaycastResult argOther) {
    lambda = argOther.lambda;
    normal.setFrom(argOther.normal);
    return this;
  }
}
