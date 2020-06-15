part of box2d;

/// Gear joint definition. This definition requires two existing revolute or prismatic joints (any
/// combination will work). The provided joints must attach a dynamic body to a static body.
class GearJointDef extends JointDef {
  /// The first revolute/prismatic joint attached to the gear joint.
  Joint joint1;

  /// The second revolute/prismatic joint attached to the gear joint.
  Joint joint2;

  /// Gear ratio.
  ///
  /// @see GearJoint
  double ratio = 0.0;

  GearJointDef() : super(JointType.GEAR);
}
