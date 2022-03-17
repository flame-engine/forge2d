import '../../../forge2d.dart';

/// The base joint class. Joints are used to constrain two bodies together in various fashions. Some
/// joints also feature limits and motors.
abstract class Joint {
  static T create<T extends Joint>(World world, JointDef def) {
    if (T == ConstantVolumeJoint) {
      return ConstantVolumeJoint(world, def as ConstantVolumeJointDef) as T;
    } else if (T == DistanceJoint) {
      return DistanceJoint(def as DistanceJointDef) as T;
    } else if (T == FrictionJoint) {
      return FrictionJoint(def as FrictionJointDef) as T;
    } else if (T == GearJoint) {
      return GearJoint(def as GearJointDef) as T;
    } else if (T == MotorJoint) {
      return MotorJoint(def as MotorJointDef) as T;
    } else if (T == MouseJoint) {
      return MouseJoint(def as MouseJointDef) as T;
    } else if (T == PrismaticJoint) {
      return PrismaticJoint(def as PrismaticJointDef) as T;
    } else if (T == PulleyJoint) {
      return PulleyJoint(def as PulleyJointDef) as T;
    } else if (T == RevoluteJoint) {
      return RevoluteJoint(def as RevoluteJointDef) as T;
    } else if (T == RopeJoint) {
      return RopeJoint(def as RopeJointDef) as T;
    } else if (T == WeldJoint) {
      return WeldJoint(def as WeldJointDef) as T;
    } else if (T == WheelJoint) {
      return WheelJoint(def as WheelJointDef) as T;
    } else {
      throw ArgumentError('Unknown joint type: $T');
    }
  }

  static void destroy(Joint joint) {
    joint.destructor();
  }

  final JointType _type;
  late Body bodyA;
  late Body bodyB;

  bool islandFlag = false;
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
    islandFlag = false;
  }

  /// get the type of the concrete joint.
  ///
  /// @return
  JointType get type => _type;

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

  final Color3i _color = Color3i.zero();

  void render(DebugDraw debugDraw) {
    final xf1 = bodyA.transform;
    final xf2 = bodyB.transform;
    final x1 = xf1.p;
    final x2 = xf2.p;
    final p1 = anchorA;
    final p2 = anchorB;

    _color.setFromRGBd(0.5, 0.8, 0.8);

    switch (type) {
      case JointType.distance:
        debugDraw.drawSegment(p1, p2, _color);
        break;

      case JointType.pulley:
        {
          final pulley = this as PulleyJoint;
          final s1 = pulley.getGroundAnchorA();
          final s2 = pulley.getGroundAnchorB();
          debugDraw.drawSegment(s1, p1, _color);
          debugDraw.drawSegment(s2, p2, _color);
          debugDraw.drawSegment(s1, s2, _color);
        }
        break;

      case JointType.friction:
        debugDraw.drawSegment(x1, x2, _color);
        break;

      case JointType.constantVolume:
      case JointType.mouse:
        // don't draw this
        break;
      default:
        debugDraw.drawSegment(x1, p1, _color);
        debugDraw.drawSegment(p1, p2, _color);
        debugDraw.drawSegment(x2, p2, _color);
    }
  }
}
