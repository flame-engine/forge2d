import 'package:forge2d/forge2d.dart';
import 'package:meta/meta.dart';

/// The base joint class. Joints are used to constrain two bodies together in
/// various fashions. Some joints also feature limits and motors.
abstract class Joint {
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
  Vector2 get anchorA => bodyA.worldPoint(localAnchorA);

  /// Get the anchor point on bodyB in world coordinates.
  Vector2 get anchorB => bodyB.worldPoint(localAnchorB);

  /// Get the reaction force on body2 at the joint anchor in Newtons.
  Vector2 reactionForce(double invDt);

  /// Get the reaction torque on body2 in N*m.
  double reactionTorque(double invDt);

  /// Get collide connected. Note: modifying the collide connect flag won't work
  /// correctly because the flag is only checked when fixture AABBs begin to
  /// overlap.
  bool get collideConnected => _collideConnected;

  /// Short-cut function to determine if either body is inactive.
  bool get isActive {
    return bodyA.isActive && bodyB.isActive;
  }

  @internal
  void initVelocityConstraints(SolverData data);

  @internal
  void solveVelocityConstraints(SolverData data);

  /// This returns true if the position errors are within tolerance. Internal.
  bool solvePositionConstraints(SolverData data);

  /// Override to handle destruction of joint
  void destructor() {}

  /// Color used to [render].
  final renderColor = Color3i.zero()..setFromRGBd(0.5, 0.8, 0.8);

  void render(DebugDraw debugDraw) {}
}
