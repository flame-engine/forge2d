part of box2d;

/// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
class RayCastInput {
  final Vector2 p1 = Vector2.zero(), p2 = Vector2.zero();
  double maxFraction = 0.0;

  void set(final RayCastInput rci) {
    p1.setFrom(rci.p1);
    p2.setFrom(rci.p2);
    maxFraction = rci.maxFraction;
  }
}
