import 'package:forge2d/forge2d.dart';

/// Friction joint definition.
class FrictionJointDef<A extends Body, B extends Body> extends JointDef<A, B> {
  /// The maximum friction force in N.
  double maxForce = 0.0;

  /// The maximum friction torque in N-m.
  double maxTorque = 0.0;

  /// Initialize the bodies, anchors, axis, and reference angle using the world
  /// anchor and world axis.
  void initialize(A bodyA, B bodyB, Vector2 anchor) {
    this.bodyA = bodyA;
    this.bodyB = bodyB;
    localAnchorA.setFrom(bodyA.localPoint(anchor));
    localAnchorB.setFrom(bodyB.localPoint(anchor));
  }
}
