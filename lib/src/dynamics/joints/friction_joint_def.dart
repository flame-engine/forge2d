part of box2d;

/// Friction joint definition.
class FrictionJointDef extends JointDef {
  /// The maximum friction force in N.
  double maxForce = 0.0;

  /// The maximum friction torque in N-m.
  double maxTorque = 0.0;

  FrictionJointDef() : super(JointType.FRICTION);

  /// Initialize the bodies, anchors, axis, and reference angle using the world anchor and world
  /// axis.
  void initialize(Body bA, Body bB, Vector2 anchor) {
    bodyA = bA;
    bodyB = bB;
    localAnchorA.setFrom(bA.getLocalPoint(anchor));
    localAnchorB.setFrom(bB.getLocalPoint(anchor));
  }
}
