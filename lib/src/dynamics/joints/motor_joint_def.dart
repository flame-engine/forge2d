part of box2d;

/// Motor joint definition.
class MotorJointDef extends JointDef {
  /// Position of bodyB minus the position of bodyA, in bodyA's frame, in meters.
  final Vector2 linearOffset = Vector2.zero();

  /// The bodyB angle minus bodyA angle in radians.
  double angularOffset = 0.0;

  /// The maximum motor force in N.
  double maxForce = 1.0;

  /// The maximum motor torque in N-m.
  double maxTorque = 1.0;

  /// Position correction factor in the range [0,1].
  double correctionFactor = 0.3;

  MotorJointDef() : super(JointType.MOTOR);

  void initialize(Body bA, Body bB) {
    bodyA = bA;
    bodyB = bB;
    Vector2 xB = bodyB.position;
    linearOffset.setFrom(bodyA.getLocalPoint(xB));

    double angleA = bodyA.getAngle();
    double angleB = bodyB.getAngle();
    angularOffset = angleB - angleA;
  }
}
