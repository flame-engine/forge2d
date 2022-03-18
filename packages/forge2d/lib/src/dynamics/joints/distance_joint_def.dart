import '../../../forge2d.dart';

/// Distance joint definition. This requires defining an anchor point on both bodies and the non-zero
/// length of the distance joint. The definition uses local anchor points so that the initial
/// configuration can violate the constraint slightly. This helps when saving and loading a game.
///
/// @warning Do not use a zero or short length.
class DistanceJointDef<A extends Body, B extends Body> extends JointDef<A, B> {
  /// The equilibrium length between the anchor points.
  double length = 1.0;

  /// The mass-spring-damper frequency in Hertz.
  double frequencyHz = 0.0;

  /// The damping ratio. 0 = no damping, 1 = critical damping.
  double dampingRatio = 0.0;

  DistanceJointDef() : super(JointType.distance);

  /// Initialize the bodies, anchors, and length using the world anchors.
  ///
  /// @param b1 First body
  /// @param b2 Second body
  /// @param anchor1 World anchor on first body
  /// @param anchor2 World anchor on second body
  void initialize(
    final A b1,
    final B b2,
    final Vector2 anchor1,
    final Vector2 anchor2,
  ) {
    bodyA = b1;
    bodyB = b2;
    localAnchorA.setFrom(bodyA.localPoint(anchor1));
    localAnchorB.setFrom(bodyB.localPoint(anchor2));
    final d = anchor2 - anchor1;
    length = d.length;
  }
}
