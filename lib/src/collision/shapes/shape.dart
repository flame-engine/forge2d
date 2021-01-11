part of forge2d;

/// A shape is used for collision detection. You can create a shape however you like. Shapes used for
/// simulation in World are created automatically when a Fixture is created. Shapes may encapsulate a
/// one or more child shapes.
abstract class Shape {
  final ShapeType shapeType;
  double radius = 0.0;

  Shape(this.shapeType);

  /// Get the number of child primitives
  int getChildCount();

  /// Test a point for containment in this shape. This only works for convex shapes.
  ///
  /// @param xf the shape world transform.
  /// @param p a point in world coordinates.
  bool testPoint(final Transform xf, final Vector2 p);

  /// Cast a ray against a child shape.
  ///
  /// @param argOutput the ray-cast results.
  /// @param argInput the ray-cast input parameters.
  /// @param argTransform the transform to be applied to the shape.
  /// @param argChildIndex the child shape index
  /// @return if hit
  bool raycast(
    RayCastOutput output,
    RayCastInput input,
    Transform transform,
    int childIndex,
  );

  /// Given a transform, compute the associated axis aligned bounding box for a child shape.
  ///
  /// @param argAabb returns the axis aligned box.
  /// @param argXf the world transform of the shape.
  void computeAABB(final AABB aabb, final Transform xf, int childIndex);

  /// Compute the mass properties of this shape using its dimensions and density. The inertia tensor
  /// is computed about the local origin.
  ///
  /// @param massData returns the mass data for this shape.
  /// @param density the density in kilograms per meter squared.
  void computeMass(final MassData massData, final double density);

  /// Compute the distance from the current shape to the specified point. This only works for convex
  /// shapes.
  ///
  /// @param xf the shape world transform.
  /// @param p a point in world coordinates.
  /// @param normalOut returns the direction in which the distance increases.
  /// @return distance returns the distance from the current shape.
  double computeDistanceToOut(
    Transform xf,
    Vector2 p,
    int childIndex,
    Vector2 normalOut,
  );

  Shape clone();
}
