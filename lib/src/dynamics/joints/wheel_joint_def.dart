part of box2d;

/// Wheel joint definition. This requires defining a line of motion using an axis and an anchor
/// point. The definition uses local anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero when the local anchor points
/// coincide in world space. Using local anchors and a local axis helps when saving and loading a
/// game.
class WheelJointDef extends JointDef {
  /// The local translation axis in body1.
  final Vector2 localAxisA = Vector2.zero();

  /// Enable/disable the joint motor.
  bool enableMotor = false;

  /// The maximum motor torque, usually in N-m.
  double maxMotorTorque = 0.0;

  /// The desired motor speed in radians per second.
  double motorSpeed = 0.0;

  /// Suspension frequency, zero indicates no suspension
  double frequencyHz = 0.0;

  /// Suspension damping ratio, one indicates critical damping
  double dampingRatio = 0.0;

  WheelJointDef() : super(JointType.WHEEL) {
    localAxisA.setValues(1.0, 0.0);
  }

  void initialize(Body b1, Body b2, Vector2 anchor, Vector2 axis) {
    bodyA = b1;
    bodyB = b2;
    localAnchorA.setFrom(b1.getLocalPoint(anchor));
    localAnchorB.setFrom(b2.getLocalPoint(anchor));
    localAxisA.setFrom(bodyA.getLocalVector(axis));
  }
}
