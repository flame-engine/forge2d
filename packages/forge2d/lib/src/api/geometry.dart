import 'package:vector_math/vector_math.dart';

/// The geometry used to create a shape on a body.
///
/// Geometry objects are plain immutable data; they are converted to their
/// native counterparts when a shape is created.
sealed class ShapeGeometry {
  const ShapeGeometry();
}

/// A circle with an optional local offset.
///
/// Mirrors the native `b2Circle` type.
final class Circle extends ShapeGeometry {
  /// Creates a circle with the given [radius], optionally offset from the
  /// body origin by [center].
  Circle({required this.radius, Vector2? center})
    : center = center ?? Vector2.zero();

  /// The local center of the circle.
  final Vector2 center;

  /// The radius of the circle.
  final double radius;
}

/// A capsule: a line segment with a radius, forming an extruded circle.
///
/// Mirrors the native `b2Capsule` type.
final class Capsule extends ShapeGeometry {
  /// Creates a capsule between [center1] and [center2] with the given
  /// [radius].
  Capsule({required this.center1, required this.center2, required this.radius});

  /// The local center of the first semicircle.
  final Vector2 center1;

  /// The local center of the second semicircle.
  final Vector2 center2;

  /// The radius of the capsule.
  final double radius;
}

/// A standalone line segment.
///
/// Mirrors the native `b2Segment` type.
final class Segment extends ShapeGeometry {
  /// Creates a segment between [point1] and [point2].
  Segment({required this.point1, required this.point2});

  /// The first endpoint in local coordinates.
  final Vector2 point1;

  /// The second endpoint in local coordinates.
  final Vector2 point2;
}

/// A convex polygon with at most [maxVertices] vertices, with an optional
/// rounding radius.
///
/// Mirrors the native `b2Polygon` type. The native library computes the
/// convex hull of the given points at shape creation time; creating a shape
/// from points whose hull is degenerate (fewer than three distinct,
/// non-collinear points) throws an [ArgumentError].
final class Polygon extends ShapeGeometry {
  /// Creates a polygon from the convex hull of [points].
  Polygon(List<Vector2> this.points, {this.radius = 0})
    : assert(points.length >= 3, 'A polygon needs at least three points'),
      halfWidth = null,
      halfHeight = null,
      center = null,
      rotation = null;

  /// Creates an axis-aligned box with half-width [halfWidth] and half-height
  /// [halfHeight], centered on the body origin.
  Polygon.box(double this.halfWidth, double this.halfHeight, {this.radius = 0})
    : points = null,
      center = null,
      rotation = null;

  /// Creates a square with half-extent [halfExtent], centered on the body
  /// origin.
  Polygon.square(double halfExtent, {double radius = 0})
    : this.box(halfExtent, halfExtent, radius: radius);

  /// Creates a box that is offset by [center] and rotated by [rotation]
  /// radians relative to the body origin.
  Polygon.offsetBox(
    double this.halfWidth,
    double this.halfHeight, {
    required Vector2 this.center,
    this.rotation = 0,
  }) : points = null,
       radius = 0;

  /// The points whose convex hull forms the polygon, or null for boxes.
  final List<Vector2>? points;

  /// The rounding radius applied to the polygon.
  final double radius;

  /// The half-width for box polygons, or null for point-based polygons.
  final double? halfWidth;

  /// The half-height for box polygons, or null for point-based polygons.
  final double? halfHeight;

  /// The local center offset for offset boxes.
  final Vector2? center;

  /// The local rotation in radians for offset boxes.
  final double? rotation;

  /// The maximum number of vertices a polygon can have.
  static const maxVertices = 8;
}
