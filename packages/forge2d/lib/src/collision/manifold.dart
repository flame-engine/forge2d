import 'package:forge2d/forge2d.dart';
import 'package:forge2d/src/settings.dart' as settings;

/// A manifold for two touching convex shapes. Forge2D supports multiple types
/// of contact:
///
/// - clip point versus plane with radius
/// - point versus point with radius (circles)
///
/// The local point usage depends on the manifold type:
///
/// - e_circles: the local center of circleA
/// - e_faceA: the center of faceA
/// - e_faceB: the center of faceB
///
/// Similarly the local normal usage:
///
/// - e_circles: not used
/// - e_faceA: the normal on polygonA
/// - e_faceB: the normal on polygonB
///
/// We store contacts in this way so that position correction can account for
/// movement, which is critical for continuous physics. All contact scenarios
/// must be expressed in one of these types.
/// This structure is stored across time steps, so we keep it small.

enum ManifoldType { circles, faceA, faceB }

class Manifold {
  /// The points of contact.
  final List<ManifoldPoint> points = List<ManifoldPoint>.generate(
    settings.maxManifoldPoints,
    (_) => ManifoldPoint(),
  );

  /// not use for Type::e_points
  final Vector2 localNormal = Vector2.zero();

  /// usage depends on manifold type
  final Vector2 localPoint = Vector2.zero();

  ManifoldType type = ManifoldType.circles;

  /// The number of manifold points.
  int pointCount = 0;

  /// Initially a manifold with 0 points, with it's points array full of
  /// instantiated ManifoldPoints.
  Manifold();

  /// Creates this manifold as a copy of the other.
  Manifold.copy(Manifold other) {
    localNormal.setFrom(other.localNormal);
    localPoint.setFrom(other.localPoint);
    type = other.type;
    pointCount = other.pointCount;
    // djm: this is correct now
    for (var i = 0; i < settings.maxManifoldPoints; i++) {
      points[i] = ManifoldPoint.copy(other.points[i]);
    }
  }

  /// Set this manifold from the given one.
  void set(Manifold manifold) {
    for (var i = 0; i < manifold.pointCount; i++) {
      points[i].set(manifold.points[i]);
    }

    type = manifold.type;
    localNormal.setFrom(manifold.localNormal);
    localPoint.setFrom(manifold.localPoint);
    pointCount = manifold.pointCount;
  }
}
