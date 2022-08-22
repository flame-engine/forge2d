import 'package:forge2d/forge2d.dart';

/// Motor joint definition.
class MotorJointDef<A extends Body, B extends Body> extends JointDef<A, B> {
  /// Position of bodyB minus the position of bodyA, in bodyA's frame, in
  /// meters.
  final Vector2 linearOffset = Vector2.zero();

  /// The bodyB angle minus bodyA angle in radians.
  double angularOffset = 0.0;

  /// The maximum motor force in N.
  double maxForce = 1.0;

  /// The maximum motor torque in N-m.
  double maxTorque = 1.0;

  /// Position correction factor in the range [0,1].
  double correctionFactor = 0.3;

  void initialize(A bodyA, B bodyB) {
    this.bodyA = bodyA;
    this.bodyB = bodyB;
    final xB = bodyB.position;
    linearOffset.setFrom(bodyA.localPoint(xB));

    final angleA = bodyA.angle;
    final angleB = bodyB.angle;
    angularOffset = angleB - angleA;
  }
}
