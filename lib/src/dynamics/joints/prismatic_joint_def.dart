part of box2d;

/// Prismatic joint definition. This requires defining a line of motion using an axis and an anchor
/// point. The definition uses local anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero when the local anchor points
/// coincide in world space. Using local anchors and a local axis helps when saving and loading a
/// game.
///
/// @warning at least one body should by dynamic with a non-fixed rotation.
class PrismaticJointDef extends JointDef {
  /// The local translation axis in body1.
  final Vector2 localAxisA = Vector2(1.0, 0.0);

  /// The constrained angle between the bodies: body2_angle - body1_angle.
  double referenceAngle = 0.0;

  /// Enable/disable the joint limit.
  bool enableLimit = false;

  /// The lower translation limit, usually in meters.
  double lowerTranslation = 0.0;

  /// The upper translation limit, usually in meters.
  double upperTranslation = 0.0;

  /// Enable/disable the joint motor.
  bool enableMotor = false;

  /// The maximum motor torque, usually in N-m.
  double maxMotorForce = 0.0;

  /// The desired motor speed in radians per second.
  double motorSpeed = 0.0;

  PrismaticJointDef() : super(JointType.PRISMATIC);

  /// Initialize the bodies, anchors, axis, and reference angle using the world anchor and world
  /// axis.
  void initialize(Body b1, Body b2, Vector2 anchor, Vector2 axis) {
    bodyA = b1;
    bodyB = b2;
    localAnchorA.setFrom(bodyA.getLocalPoint(anchor));
    localAnchorB.setFrom(bodyB.getLocalPoint(anchor));
    localAxisA.setFrom(bodyA.getLocalVector(axis));
    referenceAngle = bodyB.getAngle() - bodyA.getAngle();
  }
}
