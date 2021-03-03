import '../../../forge2d.dart';

class WeldJointDef extends JointDef {
  /// The body2 angle minus body1 angle in the reference state (radians).
  double referenceAngle = 0.0;

  /// The mass-spring-damper frequency in Hertz. Rotation only. Disable softness with a value of 0.
  double frequencyHz = 0.0;

  /// The damping ratio. 0 = no damping, 1 = critical damping.
  double dampingRatio = 0.0;

  WeldJointDef() : super(JointType.weld);

  /// Initialize the bodies, anchors, and reference angle using a world anchor point.
  void initialize(Body bodyA, Body bodyB, Vector2 anchor) {
    this.bodyA = bodyA;
    this.bodyB = bodyB;
    localAnchorA.setFrom(bodyA.getLocalPoint(anchor));
    localAnchorB.setFrom(bodyB.getLocalPoint(anchor));
    referenceAngle = bodyB.getAngle() - bodyA.getAngle();
  }
}
