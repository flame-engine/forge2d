import 'package:forge2d/forge2d.dart';

/// Definition for a {@link ConstantVolumeJoint}, which connects a group a
/// bodies together so they maintain a constant volume within them.
class ConstantVolumeJointDef<A extends Body> extends JointDef<A, A> {
  double frequencyHz = 0.0;
  double dampingRatio = 0.0;

  final List<A> bodies = <A>[];
  final List<DistanceJoint> joints = [];

  /// Adds a body to the group
  void addBody(A body) {
    bodies.add(body);
    if (bodies.length == 1) {
      bodyA = body;
    }
    if (bodies.length == 2) {
      bodyB = body;
    }
  }

  /// Adds a body and the pre-made distance joint. Should only be used for
  /// deserialization.
  void addBodyAndJoint(A argBody, DistanceJoint argJoint) {
    addBody(argBody);
    joints.add(argJoint);
  }
}
