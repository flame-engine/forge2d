part of box2d;

/// Revolute joint definition. This requires defining an anchor point where the bodies are joined.
/// The definition uses local anchor points so that the initial configuration can violate the
/// constraint slightly. You also need to specify the initial relative angle for joint limits. This
/// helps when saving and loading a game. The local anchor points are measured from the body's origin
/// rather than the center of mass because:<br/>
/// <ul>
/// <li>you might not know where the center of mass will be.</li>
/// <li>if you add/remove shapes from a body and recompute the mass, the joints will be broken.</li>
/// </ul>
class RevoluteJointDef extends JointDef {
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

  /// The maximum motor torque used to achieve the desired motor speed. Usually in N-m.
  double maxMotorTorque = 0.0;

  RevoluteJointDef() : super(JointType.REVOLUTE);

  /// Initialize the bodies, anchors, and reference angle using the world anchor.
  ///
  /// @param b1
  /// @param b2
  /// @param anchor
  void initialize(final Body b1, final Body b2, final Vector2 anchor) {
    bodyA = b1;
    bodyB = b2;
    localAnchorA.setFrom(bodyA.getLocalPoint(anchor));
    localAnchorB.setFrom(bodyB.getLocalPoint(anchor));
    referenceAngle = bodyB.getAngle() - bodyA.getAngle();
  }
}
