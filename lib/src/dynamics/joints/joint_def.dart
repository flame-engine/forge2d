import '../../../forge2d.dart';

/// {@template dynamics.joints.joint_def}
/// [JointDef]s are used to construct [Joint]s.
///
/// All [Joint]s are connected between two different [Body]ies.
/// One [Body] may [BodyType.static].
///
/// [Joint]s between [BodyType.static] and/or [BodyType.kinematic] are allowed,
/// but have no effect and use some processing time.
///
/// If possible, avoid directly extending [JointDef]. Instead, extend from
/// already defined [JointDef] inplementations.
/// {@endtemplate}
abstract class JointDef {
  /// {@macro dynamics.joints.joint_def.type}
  JointDef(this.type, [this.collideConnected = false]);

  /// The local anchor point relative to body1's origin.
  final Vector2 localAnchorA = Vector2.zero();

  /// The local anchor point relative to body2's origin.
  final Vector2 localAnchorB = Vector2.zero();

  /// The joint type is set automatically for concrete joint types.
  JointType type;

  /// Use this to attach application specific data to your joints.
  Object? userData;

  /// The first attached body.
  late Body bodyA;

  /// The second attached body.
  late Body bodyB;

  /// Set this flag to true if the attached bodies should collide.
  bool collideConnected = false;
}
