import 'package:forge2d/forge2d.dart';
import 'package:forge2d/src/settings.dart' as settings;

/// Pulley joint definition. This requires two ground anchors, two dynamic body
/// anchor points, and a pulley ratio.
class PulleyJointDef<A extends Body, B extends Body> extends JointDef<A, B> {
  /// The first ground anchor in world coordinates. This point never moves.
  Vector2 groundAnchorA = Vector2(-1.0, 1.0);

  /// The second ground anchor in world coordinates. This point never moves.
  Vector2 groundAnchorB = Vector2(1.0, 1.0);

  /// The a reference length for the segment attached to bodyA.
  double lengthA = 0.0;

  /// The a reference length for the segment attached to bodyB.
  double lengthB = 0.0;

  /// The pulley ratio, used to simulate a block-and-tackle.
  double ratio = 1.0;

  PulleyJointDef() : super(collideConnected: true);

  /// Initialize the bodies, anchors, lengths, max lengths, and ratio using the
  /// world anchors.
  void initialize(
    A b1,
    B b2,
    Vector2 ga1,
    Vector2 ga2,
    Vector2 anchor1,
    Vector2 anchor2,
    double r,
  ) {
    bodyA = b1;
    bodyB = b2;
    groundAnchorA = ga1;
    groundAnchorB = ga2;
    localAnchorA.setFrom(bodyA.localPoint(anchor1));
    localAnchorB.setFrom(bodyB.localPoint(anchor2));
    final d1 = anchor1 - ga1;
    lengthA = d1.length;
    final d2 = anchor2 - ga2;
    lengthB = d2.length;
    ratio = r;
    assert(ratio > settings.epsilon);
  }
}
