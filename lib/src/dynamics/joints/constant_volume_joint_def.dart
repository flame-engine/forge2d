import '../../../forge2d.dart';

/// Definition for a {@link ConstantVolumeJoint}, which connects a group a bodies together so they
/// maintain a constant volume within them.

class ConstantVolumeJointDef<A extends Body> extends JointDef<A, A> {
  double frequencyHz = 0.0;
  double dampingRatio = 0.0;

  final List<A> bodies = <A>[];
  final List<DistanceJoint> joints = [];

  ConstantVolumeJointDef() : super(JointType.constantVolume) {
    collideConnected = false;
  }

  /// Adds a body to the group
  ///
  /// @param argBody
  void addBody(A argBody) {
    bodies.add(argBody);
    if (bodies.length == 1) {
      bodyA = argBody;
    }
    if (bodies.length == 2) {
      bodyB = argBody;
    }
  }

  /// Adds a body and the pre-made distance joint. Should only be used for deserialization.
  void addBodyAndJoint(A argBody, DistanceJoint argJoint) {
    addBody(argBody);
    joints.add(argJoint);
  }
}
