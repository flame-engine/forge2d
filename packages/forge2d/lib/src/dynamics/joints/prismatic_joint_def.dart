import 'package:forge2d/forge2d.dart';

/// Prismatic joint definition. This requires defining a line of motion using an
/// axis and an anchor point. The definition uses local anchor points and a
/// local axis so that the initial configuration can violate the constraint
/// slightly. The joint translation is zero when the local anchor points
/// coincide in world space. Using local anchors and a local axis helps when
/// saving and loading a game.
///
/// Warning: at least one body should by dynamic with a non-fixed rotation.
class PrismaticJointDef<A extends Body, B extends Body> extends JointDef<A, B> {
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

  /// Initialize the bodies, anchors, axis, and reference angle using the world
  /// anchor and world axis.
  void initialize(A b1, B b2, Vector2 anchor, Vector2 axis) {
    bodyA = b1;
    bodyB = b2;
    localAnchorA.setFrom(bodyA.localPoint(anchor));
    localAnchorB.setFrom(bodyB.localPoint(anchor));
    localAxisA.setFrom(bodyA.localVector(axis));
    referenceAngle = bodyB.angle - bodyA.angle;
  }
}
