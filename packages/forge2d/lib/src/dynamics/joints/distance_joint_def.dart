import 'package:forge2d/forge2d.dart';

/// Distance joint definition. This requires defining an anchor point on both
/// bodies and the non-zero length of the distance joint. The definition uses
/// local anchor points so that the initial configuration can violate the
/// constraint slightly. This helps when saving and loading a game.
///
/// Warning: Do not use a zero or short length.
class DistanceJointDef<A extends Body, B extends Body> extends JointDef<A, B> {
  /// The equilibrium length between the anchor points.
  double length = 1.0;

  /// The mass-spring-damper frequency in Hertz.
  double frequencyHz = 0.0;

  /// The damping ratio. 0 = no damping, 1 = critical damping.
  double dampingRatio = 0.0;

  /// Initialize the bodies, anchors, and length using the world anchors.
  void initialize(
    final A body1,
    final B body2,
    final Vector2 anchor1,
    final Vector2 anchor2,
  ) {
    bodyA = body1;
    bodyB = body2;
    localAnchorA.setFrom(bodyA.localPoint(anchor1));
    localAnchorB.setFrom(bodyB.localPoint(anchor2));
    final d = anchor2 - anchor1;
    length = d.length;
  }
}
