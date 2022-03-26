import '../../../forge2d.dart';

class WeldJointDef<A extends Body, B extends Body> extends JointDef<A, B> {
  /// The body2 angle minus body1 angle in the reference state (radians).
  double referenceAngle = 0.0;

  /// The mass-spring-damper frequency in Hertz. Rotation only. Disable softness with a value of 0.
  double frequencyHz = 0.0;

  /// The damping ratio. 0 = no damping, 1 = critical damping.
  double dampingRatio = 0.0;

  /// Initialize the bodies, anchors, and reference angle using a world anchor point.
  void initialize(A bodyA, B bodyB, Vector2 anchor) {
    this.bodyA = bodyA;
    this.bodyB = bodyB;
    localAnchorA.setFrom(bodyA.localPoint(anchor));
    localAnchorB.setFrom(bodyB.localPoint(anchor));
    referenceAngle = bodyB.angle - bodyA.angle;
  }
}
