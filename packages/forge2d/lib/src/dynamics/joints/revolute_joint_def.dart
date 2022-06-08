import 'package:forge2d/forge2d.dart';

/// Revolute joint definition. This requires defining an anchor point where the
/// bodies are joined. The definition uses local anchor points so that the
/// initial configuration can violate the constraint slightly.
/// You also need to specify the initial relative angle for joint limits. This
/// helps when saving and loading a game. The local anchor points are measured
/// from the body's origin rather than the center of mass because:
///  - You might not know where the center of mass will be.
///  - If you add/remove shapes from a body and recompute the mass,
///    the joints will be broken.
class RevoluteJointDef<A extends Body, B extends Body> extends JointDef<A, B> {
  /// The body2 angle minus body1 angle in the reference state (radians).
  double referenceAngle = 0.0;

  /// A flag to enable joint limits.
  bool enableLimit = false;

  /// The lower angle for the joint limit (radians).
  double lowerAngle = 0.0;

  /// The upper angle for the joint limit (radians).
  double upperAngle = 0.0;

  /// A flag to enable the joint motor.
  bool enableMotor = false;

  /// The desired motor speed. Usually in radians per second.
  double motorSpeed = 0.0;

  /// The maximum motor torque used to achieve the desired motor speed.
  /// Usually in N-m.
  double maxMotorTorque = 0.0;

  /// Initialize the bodies, anchors, and reference angle using the world
  /// anchor.
  void initialize(final A body1, final B body2, final Vector2 anchor) {
    bodyA = body1;
    bodyB = body2;
    localAnchorA.setFrom(bodyA.localPoint(anchor));
    localAnchorB.setFrom(bodyB.localPoint(anchor));
    referenceAngle = bodyB.angle - bodyA.angle;
  }
}
