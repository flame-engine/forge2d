part of box2d;

/// The base joint class. Joints are used to constrain two bodies together in various fashions. Some
/// joints also feature limits and motors.
abstract class Joint {
  static Joint create(World world, JointDef def) {
    switch (def.type) {
      case JointType.MOUSE:
        return MouseJoint(world.getPool(), def as MouseJointDef);
      case JointType.DISTANCE:
        return DistanceJoint(world.getPool(), def as DistanceJointDef);
      case JointType.PRISMATIC:
        return PrismaticJoint(world.getPool(), def as PrismaticJointDef);
      case JointType.REVOLUTE:
        return RevoluteJoint(world.getPool(), def as RevoluteJointDef);
      case JointType.WELD:
        return WeldJoint(world.getPool(), def as WeldJointDef);
      case JointType.FRICTION:
        return FrictionJoint(world.getPool(), def as FrictionJointDef);
      case JointType.WHEEL:
        return WheelJoint(world.getPool(), def as WheelJointDef);
      case JointType.GEAR:
        return GearJoint(world.getPool(), def as GearJointDef);
      case JointType.PULLEY:
        return PulleyJoint(world.getPool(), def as PulleyJointDef);
      case JointType.CONSTANT_VOLUME:
        return ConstantVolumeJoint(world, def as ConstantVolumeJointDef);
      case JointType.ROPE:
        return RopeJoint(world.getPool(), def as RopeJointDef);
      case JointType.MOTOR:
        return MotorJoint(world.getPool(), def as MotorJointDef);
      case JointType.UNKNOWN:
      default:
        return null;
    }
  }

  static void destroy(Joint joint) {
    joint.destructor();
  }

  final JointType _type;
  Joint _prev;
  Joint _next;
  JointEdge _edgeA;
  JointEdge _edgeB;
  Body _bodyA;
  Body _bodyB;

  bool _islandFlag = false;
  bool _collideConnected = false;

  IWorldPool pool;

  final Vector2 localAnchorA;
  final Vector2 localAnchorB;

  Joint(IWorldPool worldPool, JointDef def)
      : localAnchorA = def.localAnchorA,
        localAnchorB = def.localAnchorB,
        _type = def.type {
    assert(def.bodyA != def.bodyB);

    pool = worldPool;
    _prev = null;
    _next = null;
    _bodyA = def.bodyA;
    _bodyB = def.bodyB;
    _collideConnected = def.collideConnected;
    _islandFlag = false;

    _edgeA = JointEdge();
    _edgeA.joint = null;
    _edgeA.other = null;
    _edgeA.prev = null;
    _edgeA.next = null;

    _edgeB = JointEdge();
    _edgeB.joint = null;
    _edgeB.other = null;
    _edgeB.prev = null;
    _edgeB.next = null;
  }

  /// get the type of the concrete joint.
  ///
  /// @return
  JointType getType() {
    return _type;
  }

  /// get the first body attached to this joint.
  Body getBodyA() {
    return _bodyA;
  }

  /// Get the second body attached to this joint.
  ///
  /// @return
  Body getBodyB() {
    return _bodyB;
  }

  /// Get the anchor point on bodyA in world coordinates.
  ///
  /// @return
  Vector2 getAnchorA() => _bodyA.getWorldPoint(localAnchorA);

  /// Get the anchor point on bodyB in world coordinates.
  ///
  /// @return
  Vector2 getAnchorB() => _bodyB.getWorldPoint(localAnchorB);

  /// Get the reaction force on body2 at the joint anchor in Newtons.
  ///
  /// @param inv_dt
  /// @return
  Vector2 getReactionForce(double inv_dt);

  /// get the reaction torque on body2 in N*m.
  ///
  /// @param inv_dt
  /// @return
  double getReactionTorque(double inv_dt);

  /// get the next joint the world joint list.
  Joint getNext() {
    return _next;
  }

  /// Get collide connected. Note: modifying the collide connect flag won't work correctly because
  /// the flag is only checked when fixture AABBs begin to overlap.
  bool getCollideConnected() {
    return _collideConnected;
  }

  /// Short-cut function to determine if either body is inactive.
  ///
  /// @return
  bool isActive() {
    return _bodyA.isActive() && _bodyB.isActive();
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
