import 'package:forge2d/forge2d.dart';

/// Gear joint definition. This definition requires two existing revolute or
/// prismatic joints (any combination will work).
/// The provided joints must attach a dynamic body to a static body.
class GearJointDef<A extends Body, B extends Body> extends JointDef<A, B> {
  /// The first revolute/prismatic joint attached to the gear joint.
  late Joint joint1;

  /// The second revolute/prismatic joint attached to the gear joint.
  late Joint joint2;

  /// Gear ratio.
  ///
  /// @see GearJoint
  double ratio = 0.0;
}
