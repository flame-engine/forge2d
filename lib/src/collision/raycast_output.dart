part of box2d;

/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
/// come from b2RayCastInput.
class RayCastOutput {
  final Vector2 normal = Vector2.zero();
  double fraction = 0.0;

  void set(final RayCastOutput rco) {
    normal.setFrom(rco.normal);
    fraction = rco.fraction;
  }
}
