import '../../../forge2d.dart';

/// The base joint class. Joints are used to constrain two bodies together in various fashions. Some
/// joints also feature limits and motors.
abstract class Joint {
  static T create<T extends Joint>(World world, JointDef def) {
    if (T == ConstantVolumeJoint || def is ConstantVolumeJointDef) {
      return ConstantVolumeJoint(world, def as ConstantVolumeJointDef) as T;
    } else if (T == DistanceJoint || def is DistanceJointDef) {
      return DistanceJoint(def as DistanceJointDef) as T;
    } else if (T == FrictionJoint || def is FrictionJointDef) {
      return FrictionJoint(def as FrictionJointDef) as T;
    } else if (T == GearJoint || def is GearJointDef) {
      return GearJoint(def as GearJointDef) as T;
    } else if (T == MotorJoint || def is MotorJointDef) {
      return MotorJoint(def as MotorJointDef) as T;
    } else if (T == MouseJoint || def is MouseJointDef) {
      return MouseJoint(def as MouseJointDef) as T;
    } else if (T == PrismaticJoint || def is PrismaticJointDef) {
      return PrismaticJoint(def as PrismaticJointDef) as T;
    } else if (T == PulleyJoint || def is PulleyJointDef) {
      return PulleyJoint(def as PulleyJointDef) as T;
    } else if (T == RevoluteJoint || def is RevoluteJointDef) {
      return RevoluteJoint(def as RevoluteJointDef) as T;
    } else if (T == RopeJoint || def is RopeJointDef) {
      return RopeJoint(def as RopeJointDef) as T;
    } else if (T == WeldJoint || def is WeldJointDef) {
      return WeldJoint(def as WeldJointDef) as T;
    } else if (T == WheelJoint || def is WheelJointDef) {
      return WheelJoint(def as WheelJointDef) as T;
    } else {
      throw ArgumentError(
        'Invalid joint type $T with invalid joint definition $def',
      );
    }
  }

  static void destroy(Joint joint) {
    joint.destructor();
  }

  late Body bodyA;
  late Body bodyB;

  bool islandFlag = false;
  bool _collideConnected = false;

  final Vector2 localAnchorA;
  final Vector2 localAnchorB;

  Joint(JointDef def)
      : assert(def.bodyA != def.bodyB),
        localAnchorA = def.localAnchorA,
        localAnchorB = def.localAnchorB {
    bodyA = def.bodyA;
    bodyB = def.bodyB;
    _collideConnected = def.collideConnected;
    islandFlag = false;
  }

  /// Whether the body is connected to the joint
  bool containsBody(Body body) => body == bodyA || body == bodyB;

  /// Get the other body than the argument in the joint
  Body otherBody(Body body) {
    assert(containsBody(body), 'Body is not in the joint');
    return body == bodyA ? bodyB : bodyA;
  }

  /// Get the anchor point on bodyA in world coordinates.
  ///
  /// @return
  Vector2 get anchorA => bodyA.worldPoint(localAnchorA);

  /// Get the anchor point on bodyB in world coordinates.
  ///
  /// @return
  Vector2 get anchorB => bodyB.worldPoint(localAnchorB);

  /// Get the reaction force on body2 at the joint anchor in Newtons.
  ///
  /// @param invDt
  /// @return
  Vector2 reactionForce(double invDt);

  /// get the reaction torque on body2 in N*m.
  ///
  /// @param invDt
  /// @return
  double reactionTorque(double invDt);

  /// Get collide connected. Note: modifying the collide connect flag won't work correctly because
  /// the flag is only checked when fixture AABBs begin to overlap.
  bool get collideConnected => _collideConnected;

  /// Short-cut function to determine if either body is inactive.
  ///
  /// @return
  bool get isActive {
    return bodyA.isActive && bodyB.isActive;
  }

  /// Internal
  void initVelocityConstraints(SolverData data);

  /// Internal
  void solveVelocityConstraints(SolverData data);

  /// This returns true if the position errors are within tolerance. Internal.
  bool solvePositionConstraints(SolverData data);

  /// Override to handle destruction of joint
  void destructor() {}

  void render(DebugDraw debugDraw) {}
}
