import 'dart:math' as math;

import 'package:meta/meta.dart';
import 'package:vector_math/vector_math.dart';

/// A 2D rotation, stored as the cosine and sine of an angle.
///
/// Mirrors the native `b2Rot` type.
@immutable
class Rot {
  /// Creates a rotation from a cosine/sine pair.
  ///
  /// The pair must be normalized: `cos * cos + sin * sin == 1`.
  const Rot(this.cos, this.sin);

  /// The identity rotation (an angle of zero).
  const Rot.identity() : cos = 1, sin = 0;

  /// Creates a rotation of [angle] radians.
  Rot.fromAngle(double angle) : cos = math.cos(angle), sin = math.sin(angle);

  /// The cosine of the rotation angle.
  final double cos;

  /// The sine of the rotation angle.
  final double sin;

  /// The rotation angle in radians, in the range `[-pi, pi]`.
  double get angle => math.atan2(sin, cos);

  /// Rotates [v].
  Vector2 rotate(Vector2 v) =>
      Vector2(cos * v.x - sin * v.y, sin * v.x + cos * v.y);

  /// Applies the inverse of this rotation to [v].
  Vector2 inverseRotate(Vector2 v) =>
      Vector2(cos * v.x + sin * v.y, -sin * v.x + cos * v.y);

  /// Multiplies this rotation with [other], composing the two rotations.
  Rot operator *(Rot other) => Rot(
    cos * other.cos - sin * other.sin,
    sin * other.cos + cos * other.sin,
  );

  @override
  bool operator ==(Object other) =>
      other is Rot && other.cos == cos && other.sin == sin;

  @override
  int get hashCode => Object.hash(cos, sin);

  @override
  String toString() => 'Rot(cos: $cos, sin: $sin)';
}

/// A rigid 2D transform: a rotation followed by a translation.
///
/// Mirrors the native `b2Transform` type.
class Transform {
  /// Creates a transform from a translation and a rotation.
  Transform(this.position, this.rotation);

  /// The identity transform.
  Transform.identity()
    : position = Vector2.zero(),
      rotation = const Rot.identity();

  /// The translation part of the transform.
  final Vector2 position;

  /// The rotation part of the transform.
  final Rot rotation;

  /// Transforms [point] from local coordinates to world coordinates.
  Vector2 apply(Vector2 point) => rotation.rotate(point)..add(position);

  /// Transforms [point] from world coordinates to local coordinates.
  Vector2 applyInverse(Vector2 point) =>
      rotation.inverseRotate(point - position);

  @override
  String toString() => 'Transform(position: $position, rotation: $rotation)';
}

/// An axis-aligned bounding box.
///
/// Mirrors the native `b2AABB` type.
class Aabb {
  /// Creates a bounding box from its extreme points.
  Aabb(this.lowerBound, this.upperBound);

  /// Creates a bounding box containing the two given points.
  Aabb.containingPoints(Vector2 a, Vector2 b)
    : lowerBound = Vector2(math.min(a.x, b.x), math.min(a.y, b.y)),
      upperBound = Vector2(math.max(a.x, b.x), math.max(a.y, b.y));

  /// The corner with the smallest coordinates.
  final Vector2 lowerBound;

  /// The corner with the largest coordinates.
  final Vector2 upperBound;

  /// The center point of the box.
  Vector2 get center => (lowerBound + upperBound)..scale(0.5);

  /// The half-width and half-height of the box.
  Vector2 get extents => (upperBound - lowerBound)..scale(0.5);

  /// Whether this box fully contains [other].
  bool contains(Aabb other) =>
      lowerBound.x <= other.lowerBound.x &&
      lowerBound.y <= other.lowerBound.y &&
      other.upperBound.x <= upperBound.x &&
      other.upperBound.y <= upperBound.y;

  /// Whether this box overlaps [other].
  bool overlaps(Aabb other) =>
      lowerBound.x <= other.upperBound.x &&
      lowerBound.y <= other.upperBound.y &&
      other.lowerBound.x <= upperBound.x &&
      other.lowerBound.y <= upperBound.y;

  /// The smallest box containing both this box and [other].
  Aabb union(Aabb other) => Aabb(
    Vector2(
      math.min(lowerBound.x, other.lowerBound.x),
      math.min(lowerBound.y, other.lowerBound.y),
    ),
    Vector2(
      math.max(upperBound.x, other.upperBound.x),
      math.max(upperBound.y, other.upperBound.y),
    ),
  );

  @override
  String toString() => 'Aabb($lowerBound, $upperBound)';
}
