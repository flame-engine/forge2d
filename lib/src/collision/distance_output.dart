part of box2d;

/// Output for Distance.
class DistanceOutput {
  /// Closest point on shapeA
  final Vector2 pointA = Vector2.zero();

  /// Closest point on shapeB
  final Vector2 pointB = Vector2.zero();

  double distance = 0.0;

  /// number of gjk iterations used
  int iterations = 0;
}
