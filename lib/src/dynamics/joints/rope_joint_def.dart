part of box2d;

/// Rope joint definition. This requires two body anchor points and a maximum lengths. Note: by
/// default the connected objects will not collide. see collideConnected in b2JointDef.
class RopeJointDef extends JointDef {
  /// The maximum length of the rope. Warning: this must be larger than b2_linearSlop or the joint
  /// will have no effect.
  double maxLength = 0.0;

  RopeJointDef() : super(JointType.ROPE) {
    localAnchorA.setValues(-1.0, 0.0);
    localAnchorB.setValues(1.0, 0.0);
  }
}
