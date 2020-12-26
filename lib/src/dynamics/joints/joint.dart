part of forge2d;

/// The base joint class. Joints are used to constrain two bodies together in various fashions. Some
/// joints also feature limits and motors.
abstract class Joint {
  static Joint create(World world, JointDef def) {
    switch (def.type) {
      case JointType.MOUSE:
        return MouseJoint(def as MouseJointDef);
      case JointType.DISTANCE:
        return DistanceJoint(def as DistanceJointDef);
      case JointType.PRISMATIC:
        return PrismaticJoint(def as PrismaticJointDef);
      case JointType.REVOLUTE:
        return RevoluteJoint(def as RevoluteJointDef);
      case JointType.WELD:
        return WeldJoint(def as WeldJointDef);
      case JointType.FRICTION:
        return FrictionJoint(def as FrictionJointDef);
      case JointType.WHEEL:
        return WheelJoint(def as WheelJointDef);
      case JointType.GEAR:
        return GearJoint(def as GearJointDef);
      case JointType.PULLEY:
        return PulleyJoint(def as PulleyJointDef);
      case JointType.CONSTANT_VOLUME:
        return ConstantVolumeJoint(world, def as ConstantVolumeJointDef);
      case JointType.ROPE:
        return RopeJoint(def as RopeJointDef);
      case JointType.MOTOR:
        return MotorJoint(def as MotorJointDef);
      case JointType.UNKNOWN:
      default:
        return null;
    }
  }

  static void destroy(Joint joint) {
    joint.destructor();
  }

  final JointType _type;
  Body bodyA;
  Body bodyB;

  bool _islandFlag = false;
  bool _collideConnected = false;

  final Vector2 localAnchorA;
  final Vector2 localAnchorB;

  Joint(JointDef def)
      : assert(def.bodyA != def.bodyB),
        localAnchorA = def.localAnchorA,
        localAnchorB = def.localAnchorB,
        _type = def.type {
    bodyA = def.bodyA;
    bodyB = def.bodyB;
    _collideConnected = def.collideConnected;
    _islandFlag = false;
  }

  /// get the type of the concrete joint.
  ///
  /// @return
  JointType getType() {
    return _type;
  }

  /// Whether the body is connected to the joint
  bool containsBody(Body body) => body == bodyA || body == bodyB;

  /// Get the other body than the argument in the joint
  Body getOtherBody(Body body) {
    assert(containsBody(body), "Body is not in the joint");
    return body == bodyA ? bodyB : bodyB;
  }

  /// Get the anchor point on bodyA in world coordinates.
  ///
  /// @return
  Vector2 getAnchorA() => bodyA.getWorldPoint(localAnchorA);

  /// Get the anchor point on bodyB in world coordinates.
  ///
  /// @return
  Vector2 getAnchorB() => bodyB.getWorldPoint(localAnchorB);

  /// Get the reaction force on body2 at the joint anchor in Newtons.
  ///
  /// @param invDt
  /// @return
  Vector2 getReactionForce(double invDt);

  /// get the reaction torque on body2 in N*m.
  ///
  /// @param invDt
  /// @return
  double getReactionTorque(double invDt);

  /// Get collide connected. Note: modifying the collide connect flag won't work correctly because
  /// the flag is only checked when fixture AABBs begin to overlap.
  bool getCollideConnected() {
    return _collideConnected;
  }

  /// Short-cut function to determine if either body is inactive.
  ///
  /// @return
  bool isActive() {
    return bodyA.isActive() && bodyB.isActive();
  }

  /// Internal
  void initVelocityConstraints(SolverData data);

  /// Internal
  void solveVelocityConstraints(SolverData data);

  /// This returns true if the position errors are within tolerance. Internal.
  bool solvePositionConstraints(SolverData data);

  /// Override to handle destruction of joint
  void destructor() {}
}
