/// The type of a body, which determines how it moves.
///
/// Values match the native `b2BodyType` ordering.
enum BodyType {
  /// Zero mass, zero velocity, may be manually moved.
  static,

  /// Zero mass, velocity set by the user, moved by the solver.
  kinematic,

  /// Positive mass, velocity determined by forces, moved by the solver.
  dynamic,
}

/// The type of a shape's geometry.
///
/// Values match the native `b2ShapeType` ordering.
enum ShapeType {
  /// A circle with an offset.
  circle,

  /// A capsule, an extruded circle.
  capsule,

  /// A standalone line segment.
  segment,

  /// A convex polygon.
  polygon,

  /// A line segment owned by a chain shape.
  chainSegment,
}

/// The type of a joint.
///
/// Values match the native `b2JointType` ordering.
enum JointType {
  /// Keeps two points on two bodies at a fixed distance apart.
  distance,

  /// Disables collision between two bodies without other constraints.
  filter,

  /// Drives the relative transform between two bodies.
  motor,

  /// Pulls a point on a body towards a target point.
  mouse,

  /// Allows relative translation along an axis, prevents relative rotation.
  prismatic,

  /// Allows relative rotation around a point, like a hinge or pin.
  revolute,

  /// Rigidly attaches two bodies with configurable stiffness.
  weld,

  /// Provides wheel suspension: rotation around a point with a spring along
  /// an axis.
  wheel,
}
