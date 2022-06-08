import 'package:forge2d/forge2d.dart';

/// A shape is used for collision detection. You can create a shape however you
/// like. Shapes used for simulation in World are created automatically when a
/// [Fixture] is created. Shapes may encapsulate a one or more child shapes.
abstract class Shape {
  final ShapeType shapeType;
  double radius = 0.0;

  Shape(this.shapeType);

  /// Get the number of child primitives
  int get childCount;

  /// Test a point for containment in this shape. This only works for convex
  /// shapes.
  ///
  /// [xf] should be the shape world transform.
  /// [point] should be in world coordinates.
  bool testPoint(final Transform xf, final Vector2 point);

  /// Cast a ray against a child shape.
  ///
  /// [output] is the ray-cast results.
  /// [input] the ray-cast input parameters.
  /// [transform] to be applied to the shape.
  /// [childIndex] the child shape index
  /// Returns true if the child shape is hit.
  bool raycast(
    RayCastOutput output,
    RayCastInput input,
    Transform transform,
    int childIndex,
  );

  /// Given a transform, compute the associated axis aligned bounding box for a
  /// child shape.
  void computeAABB(final AABB aabb, final Transform xf, int childIndex);

  /// Compute the mass properties of this shape using its dimensions and
  /// density. The inertia tensor is computed about the local origin.
  /// [density] should be in kilograms per meter squared.
  void computeMass(final MassData massData, final double density);

  /// Compute the distance from the current shape to the specified point.
  /// This only works for convex shapes.
  ///
  /// [xf] is the shape world transform.
  /// [p] is a point in world coordinates.
  /// [normalOut] returns the direction in which the distance increases.
  /// Returns the distance from the current shape.
  double computeDistanceToOut(
    Transform xf,
    Vector2 p,
    int childIndex,
    Vector2 normalOut,
  );

  Shape clone();
}
